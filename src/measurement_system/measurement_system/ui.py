"""PySide6 concept interface for the Diplomka measurement system.

StartMeasurement may command robot motion through the ROS2 controller.
The user must explicitly arm the start button.
"""
import csv
import os
import shutil
from pathlib import Path
import sys

from PySide6.QtCore import Qt, QThread, Signal, QProcess, QTimer
import shutil
from measurement_system.UI.ros_bridge import RosBridge
from measurement_system.database_export import export_table
from measurement_system.drivers.db_setup.connection_settings import load_settings, save_settings, connect_with_settings
from measurement_system.connection_profiles import load_profiles, save_profiles, selected_robot, leica_endpoint
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QFileDialog, QFormLayout, QFrame, QGridLayout,
    QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMainWindow, QLayout, QInputDialog,
    QMessageBox, QPlainTextEdit, QProgressBar, QPushButton, QSpinBox,
    QStackedWidget, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget, QDoubleSpinBox,
)

PAGES = ["Přehled", "Kalibrace", "Import pozic", "Automatické měření", "Ruční ovládání", "Export dat", "Nastavení"]
JOINT_COLUMNS = ["point_id"] + [f"q{i}" for i in range(1, 8)]
CART_COLUMNS = ["point_id", "x", "y", "z", "rx", "ry", "rz"]


def group(title, *widgets):
    frame = QGroupBox(title)
    layout = QVBoxLayout(frame)
    for widget in widgets:
        if isinstance(widget, QLayout):
            layout.addLayout(widget)
        else:
            layout.addWidget(widget)
    return frame


def explanation(text):
    lbl = QLabel(text)
    lbl.setWordWrap(True)
    lbl.setObjectName("description")
    return lbl




class ExportWorker(QThread):
    finished_ok = Signal(str, int)
    failed = Signal(str)

    def __init__(self, path, table="measurement_data"):
        super().__init__()
        self.path = path
        self.table = table

    def run(self):
        try:
            count = export_table(self.path, self.table)
            self.finished_ok.emit(self.path, count)
        except Exception as exc:
            self.failed.emit(str(exc))


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Diplomka | Measurement Control — koncept UI")
        self.resize(1250, 780)
        self.rows = []
        self.headers = []
        self.source_path = None
        self._build()
        self._style()
        self.ros = RosBridge(self)
        self.ros.service_available.connect(self._on_service_status)
        self.ros.start_result.connect(self._on_start_result)
        self.ros.stop_result.connect(self._on_stop_result)
        self.ros.measurement_status.connect(self._on_measurement_status)
        self.ros.modules_status.connect(self._on_modules_status)
        self.ros.calibration_result.connect(self._on_calibration_result)
        self.ros.calibration_stop_result.connect(self._on_calibration_stop_result)
        self.ros.calibration_status.connect(self._on_calibration_status)
        self.ros.manual_move_result.connect(self._manual_move_done)
        self.ros.manual_leica_result.connect(self._manual_leica_done)
        self.ros.table_point_result.connect(self._table_point_done)
        self.ros.table_series_result.connect(self._table_series_done)
        self._module_timer = QTimer(self)
        self._module_timer.timeout.connect(self._refresh_modules)
        self._module_timer.start(1000)
        self._processes = {}
        self._leica_publisher = None
        self._leica_stopping = False
        self._module_terminating = set()
        self._latest_sequence_state = "IDLE"
        self._refresh_modules()
        self._update_start_button()

    def _build(self):
        root = QWidget()
        self.setCentralWidget(root)
        outer = QHBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        side = QFrame()
        side.setObjectName("sidebar")
        side.setFixedWidth(235)
        side_layout = QVBoxLayout(side)
        side_layout.setContentsMargins(18, 28, 18, 20)
        brand = QLabel("DIPLOMKA")
        brand.setObjectName("brand")
        side_layout.addWidget(brand)
        side_layout.addWidget(explanation("KUKA iiwa  ·  Leica Tracker"))
        side_layout.addSpacing(28)
        self.nav = []
        for i, name in enumerate(PAGES):
            btn = QPushButton(name)
            btn.setObjectName("nav")
            btn.setCheckable(True)
            btn.clicked.connect(lambda checked=False, index=i: self._select(index))
            side_layout.addWidget(btn)
            self.nav.append(btn)
        side_layout.addStretch(1)
        side_layout.addWidget(explanation("REŽIM: ROS2 ovládání\nStart může spustit pohyb robota."))
        outer.addWidget(side)
        content = QFrame()
        content.setObjectName("content")
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(34, 25, 34, 25)
        header = QHBoxLayout()
        self.heading = QLabel()
        self.heading.setObjectName("heading")
        header.addWidget(self.heading)
        header.addStretch(1)
        pill = QLabel("● Hardware nepřipojen")
        pill.setObjectName("offline")
        header.addWidget(pill)
        content_layout.addLayout(header)
        self.stack = QStackedWidget()
        content_layout.addWidget(self.stack)
        self.stack.addWidget(self._overview())
        self.stack.addWidget(self._calibration())
        self.stack.addWidget(self._import())
        self.stack.addWidget(self._measurement())
        self.stack.addWidget(self._manual_page())
        self.stack.addWidget(self._export())
        self.stack.addWidget(self._settings())
        outer.addWidget(content, 1)
        self.statusBar().showMessage("UI připraveno; start vyžaduje připojení ROS2 a potvrzení")
        self._select(0)

    def _page(self, info):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 18, 0, 0)
        layout.setSpacing(16)
        layout.addWidget(explanation(info))
        return widget, layout

    def _overview(self):
        page, lay = self._page(
            "Připojení modulů. Start uzlu není start měření. "
            "Dostupnost ROS2 služby sama o sobě nepotvrzuje připojení zařízení."
        )
        grid = QGridLayout()
        self.module_labels = {}
        self.module_buttons = {}
        modules = [
            ("robot", "KUKA LBR iiwa", "IIWA_Py3 / move_robot"),
            ("leica", "Leica Tracker", "ZMQ stream / monitor"),
            ("database", "PostgreSQL", "database_node / save_measurement"),
            ("controller", "Measurement Controller", "start_measurement"),
            ("calibration", "Kalibrace báze", "robot_calibration_node"),
        ]
        for i, (key, title, detail) in enumerate(modules):
            box = QGroupBox(title)
            column = QVBoxLayout(box)
            column.addWidget(explanation(detail))
            label = QLabel("Nespuštěno")
            label.setObjectName("moduleState")
            column.addWidget(label)
            button = QPushButton("Připojit")
            button.clicked.connect(lambda checked=False, k=key: self._toggle_module(k))
            column.addWidget(button)
            self.module_labels[key] = label
            self.module_buttons[key] = button
            grid.addWidget(box, i // 2, i % 2)
        lay.addLayout(grid)
        lay.addWidget(explanation(
            "Leica: Připojit spouští EtherCAT publisher leica_connect.py (eno4) "
            "a zároveň ROS2 monitor ZMQ. Stav Připojeno vyžaduje skutečný příjem dat. "
            "Pro EtherCAT mohou být potřeba administrátorská oprávnění (pkexec)."
        ))
        self.total_label = QLabel("Importované pozice: 0")
        lay.addWidget(group("Aktuální série", self.total_label))
        lay.addStretch()
        return page

    def _sequence_running(self):
        return self._latest_sequence_state not in (
            "IDLE", "FINISHED", "STOPPED", "ERROR"
        )

    def _toggle_module(self, key):
        process = self._processes.get(key)
        publisher = self._leica_publisher if key == "leica" else None
        if ((process is not None and process.state() != QProcess.NotRunning)
                or (publisher is not None and publisher.state() != QProcess.NotRunning)):
            self._disconnect_module(key)
        else:
            self._connect_module(key)

    def _disconnect_module(self, key):
        process = self._processes.get(key)
        publisher = self._leica_publisher if key == "leica" else None
        if ((process is None or process.state() == QProcess.NotRunning)
                and (publisher is None or publisher.state() == QProcess.NotRunning)):
            QMessageBox.information(
                self, "Externě spuštěný uzel",
                "Tento uzel nebyl spuštěn z UI. Nelze jej bezpečně ukončit tímto tlačítkem."
            )
            return
        if self._sequence_running() and key in ("robot", "database", "controller", "calibration", "leica"):
            QMessageBox.warning(
                self, "Probíhá sekvence",
                "Nejdříve požádej o zastavení sekvence a počkej na stav STOPPED nebo FINISHED."
            )
            return
        reply = QMessageBox.question(
            self, "Ukončit modul",
            f"Ukončit modul {key}? Připojení k zařízení může být ukončeno."
        )
        if reply != QMessageBox.Yes:
            return
        self._module_terminating.add(key)
        self.module_labels[key].setText("Ukončuji…")
        self.module_buttons[key].setEnabled(False)
        if key == "leica":
            self._leica_stopping = True
            publisher = self._leica_publisher
            if publisher is not None and publisher.state() != QProcess.NotRunning:
                publisher.terminate()
                QTimer.singleShot(3000, lambda p=publisher: self._check_terminated("leica publisher", p))
        if process is not None and process.state() != QProcess.NotRunning:
            process.terminate()  # asynchronous, not blocking Qt event loop
            QTimer.singleShot(3000, lambda k=key, p=process: self._check_terminated(k, p))
        else:
            self._module_terminating.discard(key)

    def _check_terminated(self, key, process):
        if process.state() != QProcess.NotRunning and (key in self._module_terminating or key == "leica publisher") :
            self.module_labels[key].setText("Ukončení trvá dlouho – zkontroluj uzel v terminálu")
            self.statusBar().showMessage(
                f"{key} stále běží; automatické SIGKILL je zakázáno kvůli stavu robota.", 10000
            )
            self.module_buttons[key].setEnabled(True)

    def _connect_module(self, key):
        if key == "leica":
            self._connect_leica()
            return
        if key == "database":
            try:
                conn = connect_with_settings()
                try:
                    with conn.cursor() as cur:
                        cur.execute("SELECT 1")
                        if cur.fetchone()[0] != 1:
                            raise RuntimeError("Unexpected database health check")
                finally:
                    conn.close()
            except Exception as exc:
                self.module_labels["database"].setText("Připojení PostgreSQL selhalo")
                QMessageBox.warning(self, "PostgreSQL nedostupná",
                    "Databázový uzel nebyl spuštěn. Nastav připojení v záložce Nastavení.\n\n" + str(exc))
                self._select(PAGES.index("Nastavení"))
                return
        if key == "robot":
            settings = selected_robot()
            if not settings:
                QMessageBox.warning(self, "Robot", "Nejdříve vyber robota v Nastavení.")
                self._select(PAGES.index("Nastavení"))
                return
        existing = self._processes.get(key)
        if existing is not None and existing.state() != QProcess.NotRunning:
            return
        # Spouštíme přímo Python proces ROS2 uzlu, nikoliv obalující `ros2 run`.
        # QProcess pak při terminate() ukončuje skutečný uzel, ne jen CLI rodiče.
        names = {
            "robot": "measurement_system.nodes.robot_node",
            "database": "measurement_system.nodes.database_node",
            "controller": "measurement_system.nodes.controller_node",
            "calibration": "measurement_system.nodes.robot_calibration_node",
            "leica": "measurement_system.leica_monitor",
        }
        command = [sys.executable, "-m", names[key]]
        process = QProcess(self)
        process.setProgram(command[0])
        process.setArguments(command[1:])
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda k=key, p=process: self._process_output(k, p))
        process.errorOccurred.connect(lambda err, k=key, p=process: self._process_error(k, p))
        process.finished.connect(lambda code, status, k=key: self._process_finished(k, code))
        self._processes[key] = process
        self.module_labels[key].setText("Spouštím…")
        self.module_buttons[key].setEnabled(False)
        process.start()

    def _connect_leica(self):
        """Start actual EtherCAT/ZMQ publisher, then ROS2 health monitor."""
        existing = self._leica_publisher
        if existing is not None and existing.state() != QProcess.NotRunning:
            return
        monitor = self._processes.get("leica")
        if monitor is not None and monitor.state() != QProcess.NotRunning:
            # Do not start another EtherCAT master if the monitor is already running.
            self.module_labels["leica"].setText("Monitor už běží; odpoj jej a zkus znovu")
            return
        self._leica_stopping = False
        # Default uses PolicyKit authentication in a graphical session. Override
        # with LEICA_LAUNCH_MODE=direct only when pysoem has OS permission already.
        mode = os.environ.get("LEICA_LAUNCH_MODE", "pkexec").lower()
        leica_script = Path(__file__).resolve().parent / "drivers" / "leika_driver" / "leica_connect.py"
        # For --symlink-install source / build paths both work. No venv assumed.
        if mode == "direct":
            command = [sys.executable, str(leica_script)]
        elif mode == "pkexec":
            command = ["pkexec", sys.executable, str(leica_script)]
        else:
            self.module_labels["leica"].setText("Neznámý LEICA_LAUNCH_MODE")
            return
        if shutil.which(command[0]) is None and not os.path.exists(command[0]):
            self.module_labels["leica"].setText(f"Chybí spouštěč: {command[0]}")
            return
        publisher = QProcess(self)
        publisher.setProgram(command[0])
        publisher.setArguments(command[1:])
        publisher.setProcessChannelMode(QProcess.MergedChannels)
        publisher.readyReadStandardOutput.connect(lambda p=publisher: self._process_output("leica publisher", p))
        publisher.errorOccurred.connect(lambda err, p=publisher: self._leica_publisher_error(p))
        publisher.finished.connect(lambda code, status, p=publisher: self._leica_publisher_finished(p, code))
        self._leica_publisher = publisher
        self.module_labels["leica"].setText("Spouštím EtherCAT publisher…")
        self.module_buttons["leica"].setEnabled(False)
        publisher.start()
        # The monitor subscribes immediately, reports ready only after actual data.
        QTimer.singleShot(700, lambda p=publisher: self._start_leica_monitor_if_alive(p))

    def _start_leica_monitor_if_alive(self, publisher):
        if publisher is not self._leica_publisher or publisher.state() == QProcess.NotRunning:
            return
        if self._leica_stopping:
            return
        self._connect_monitor()

    def _connect_monitor(self):
        process = QProcess(self)
        process.setProgram(sys.executable)
        process.setArguments(["-m", "measurement_system.leica_monitor"])
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda p=process: self._process_output("leica monitor", p))
        process.errorOccurred.connect(lambda err, p=process: self._process_error("leica", p))
        process.finished.connect(lambda code, status: self._process_finished("leica", code))
        self._processes["leica"] = process
        process.start()

    def _leica_publisher_error(self, publisher):
        if publisher is self._leica_publisher:
            self.module_labels["leica"].setText(f"Leica publisher: {publisher.errorString()}")
            self.module_buttons["leica"].setEnabled(True)

    def _leica_publisher_finished(self, publisher, code):
        if publisher is not self._leica_publisher:
            return
        monitor = self._processes.get("leica")
        if monitor is not None and monitor.state() != QProcess.NotRunning:
            monitor.terminate()
        if not self._leica_stopping:
            self.module_labels["leica"].setText(f"Leica publisher skončil (kód {code}); zkontroluj pkexec/eno4")
            self.statusBar().showMessage("Leica publisher se ukončil. Ověř oprávnění EtherCAT a výpis procesu.", 10000)
        self._leica_stopping = False
        self._module_terminating.discard("leica")

    def _process_output(self, key, process):
        message = bytes(process.readAllStandardOutput()).decode("utf-8", errors="replace").strip()
        if message and ("ERROR" in message or "Traceback" in message):
            self.statusBar().showMessage(f"{key}: {message[-200:]}", 10000)

    def _process_error(self, key, process):
        self.module_labels[key].setText(f"Chyba spuštění: {process.errorString()}")
        self.module_buttons[key].setEnabled(True)

    def _process_finished(self, key, code):
        self.module_labels[key].setText(f"Proces skončil (kód {code})")
        self._module_terminating.discard(key)
        self.module_buttons[key].setEnabled(True)

    def _refresh_modules(self):
        if hasattr(self, "ros"):
            self.ros.check_modules()

    def _on_modules_status(self, statuses):
        for key, ready in statuses.items():
            if key not in self.module_labels:
                continue
            process = self._processes.get(key)
            alive = process is not None and process.state() != QProcess.NotRunning
            if key == "leica":
                publisher = self._leica_publisher
                pub_alive = publisher is not None and publisher.state() != QProcess.NotRunning
                alive = alive or pub_alive
                ready = bool(ready and pub_alive)
            if ready:
                text = "Leica: publisher běží, ZMQ přijímá data" if key == "leica" else "ROS2 služba dostupná (hardware neověřen)"
            elif alive:
                text = "Proces běží, čekám na službu/data"
            else:
                text = "Není dostupné"
            self.module_labels[key].setText(text)
            terminating = key in self._module_terminating
            if alive:
                self.module_buttons[key].setText("Odpojit / ukončit uzel")
                self.module_buttons[key].setEnabled(not terminating and not self._sequence_running())
            elif ready:
                self.module_buttons[key].setText("Externě spuštěno")
                self.module_buttons[key].setEnabled(False)
            else:
                self.module_buttons[key].setText("Připojit")
                self.module_buttons[key].setEnabled(True)

    def _calibration(self):
        page, lay = self._page("Kalibrace transformací Leica → stůl → báze robota. Měření bodů se doplní po napojení na tracker a ROS2.")
        self.table_series_id = QSpinBox()
        self.table_series_id.setRange(0, 2147483647)
        self.table_series_id.setSpecialValueText("Vyber nebo založ sérii")
        self.table_robot_name = QComboBox()
        self.table_robot_name.addItems([x["name"] for x in load_profiles()["robots"]])
        self.table_series_name = QLineEdit()
        self.table_series_name.setPlaceholderText("Např. Kalibrace stolu 01")
        create_series_button = QPushButton("Založit novou sérii pro kalibraci stolu")
        create_series_button.clicked.connect(self._create_table_series)
        series_form = QFormLayout()
        series_form.addRow("ID existující série", self.table_series_id)
        series_form.addRow("Robot pro novou sérii", self.table_robot_name)
        series_form.addRow("Název nové série", self.table_series_name)
        lay.addWidget(group("Série pro body stolu", series_form, create_series_button,
            explanation("Každý bod P1–P3 se uloží s measurement_id. ID musí existovat v measurements.")))
        table = QTableWidget(3, 4)
        self.table_calibration = table
        self._table_next = 0
        table.setHorizontalHeaderLabels(["Bod", "X [mm]", "Y [mm]", "Z [mm]"])
        for i in range(3):
            table.setItem(i, 0, QTableWidgetItem(f"P{i + 1}"))
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table.setMaximumHeight(170)
        btn = QPushButton("Změřit aktuální bod Leica")
        btn.clicked.connect(self._measure_table_point)
        self.table_calibration_status = QLabel("Připraveno pro P1")
        lay.addWidget(group("1. Souřadnicový systém stolu – 3 body", table, btn,
                            self.table_calibration_status,
                            explanation("P1 = počátek, P1→P2 = osa X, P3 určuje stranu osy Y.")))
        self.calibration_csv = None
        self.calibration_csv_label = QLabel("CSV pro kalibraci: nevybráno")
        calibration_file_button = QPushButton("Načíst samostatné CSV pro kalibraci B")
        calibration_file_button.clicked.connect(self._load_calibration_csv)
        lay.addWidget(group("Vstup kalibrace B", calibration_file_button, self.calibration_csv_label))
        self.calib_robot_name = QComboBox()
        self.calib_robot_name.addItems([x["name"] for x in load_profiles()["robots"]])
        self.calib_series_name = QLineEdit("Kalibrace B")
        calib_metadata = QFormLayout()
        calib_metadata.addRow("Robot pro kalibraci B", self.calib_robot_name)
        calib_metadata.addRow("Název kalibrační série", self.calib_series_name)
        lay.addLayout(calib_metadata)
        self.calib_info = QLabel('Nezahájeno – vyber CSV určené pouze pro kalibraci B.')
        self.calib_info.setWordWrap(True)
        self.calib_progress = QProgressBar()
        self.calib_progress.setRange(0, 1)
        self.calib_progress.setValue(0)
        calib_start = QPushButton('Spustit kalibraci báze z importovaného CSV')
        calib_start.clicked.connect(self._start_robot_calibration)
        calib_stop = QPushButton('Zastavit kalibraci po aktuálním kroku')
        calib_stop.clicked.connect(self._stop_robot_calibration)
        lay.addWidget(group('2. Kalibrace báze – metoda B',
            explanation('Robot projede importované kloubové konfigurace. U každé uložíme příkazové úhly a surové měření Leica do samostatné tabulky. Výpočet transformace proběhne offline.'),
            self.calib_info, self.calib_progress, calib_start, calib_stop))
        lay.addWidget(group('3. Offline zpracování', explanation(
            'Výstup: tabulka robot_base_calibration_points; každá série má calibration_id. '
            'Záznamy obsahují požadované, nikoliv nezávisle změřené kloubové úhly.' )))
        lay.addStretch()
        return page

    def _import(self):
        page, lay = self._page("Import CSV funguje. Zatím se pouze kontroluje formát a zobrazují data; nic se neposílá robotovi.")
        bar = QHBoxLayout()
        self.format_combo = QComboBox()
        self.format_combo.addItems(["Kloubové pozice (q1–q7)", "Kartézské pozice (x,y,z,rx,ry,rz)"])
        self.format_combo.currentIndexChanged.connect(self._refresh_import_hint)
        bar.addWidget(self.format_combo)
        open_btn = QPushButton("Načíst CSV…")
        open_btn.clicked.connect(self._load_csv)
        bar.addWidget(open_btn)
        bar.addStretch()
        lay.addLayout(bar)
        self.import_hint = explanation("")
        lay.addWidget(self.import_hint)
        self.path_label = explanation("Soubor: žádný")
        lay.addWidget(self.path_label)
        self.table = QTableWidget()
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        lay.addWidget(self.table, 1)
        self.import_count = QLabel("0 validovaných pozic")
        lay.addWidget(self.import_count)
        self._refresh_import_hint()
        return page

    def _refresh_import_hint(self):
        cols = JOINT_COLUMNS if self.format_combo.currentIndex() == 0 else CART_COLUMNS
        self.import_hint.setText("Požadované názvy sloupců: " + ", ".join(cols) +
                                 "\nKloubové úhly: stupně; souřadnice: mm; orientace: stupně. "
                                 "Kartézický souřadnicový systém a konvenci rotací bude nutné před provozem potvrdit.")

    def _load_csv(self):
        filename, _ = QFileDialog.getOpenFileName(self, "Import pozic", "", "CSV (*.csv);;Všechny soubory (*)")
        if not filename:
            return
        cols = JOINT_COLUMNS if self.format_combo.currentIndex() == 0 else CART_COLUMNS
        try:
            with open(filename, newline="", encoding="utf-8-sig") as stream:
                sample = stream.read(4096)
                stream.seek(0)
                try:
                    dialect = csv.Sniffer().sniff(sample, delimiters=",;\t")
                    reader = csv.DictReader(stream, dialect=dialect)
                except csv.Error:
                    reader = csv.DictReader(stream)
                if not reader.fieldnames or any(k not in reader.fieldnames for k in cols):
                    raise ValueError("Chybějící sloupce. Očekávám: " + ", ".join(cols))
                data = []
                ids = set()
                for lineno, record in enumerate(reader, start=2):
                    point_id = str(record["point_id"]).strip()
                    if not point_id or point_id in ids:
                        raise ValueError(f"Řádek {lineno}: prázdné nebo duplicitní point_id")
                    ids.add(point_id)
                    values = [point_id]
                    for column in cols[1:]:
                        import math
                        value = float(record[column])
                        if not math.isfinite(value):
                            raise ValueError(f"Řádek {lineno}: neplatné číslo v {column}")
                        values.append(value)
                    data.append(values)
                if not data:
                    raise ValueError("CSV neobsahuje žádné pozice")
        except (OSError, ValueError, TypeError) as exc:
            QMessageBox.warning(self, "Neplatný CSV soubor", str(exc))
            return
        self.headers, self.rows, self.source_path = cols, data, filename
        self.table.setRowCount(len(data))
        self.table.setColumnCount(len(cols))
        self.table.setHorizontalHeaderLabels(cols)
        for i, row in enumerate(data):
            for j, value in enumerate(row):
                self.table.setItem(i, j, QTableWidgetItem(str(value)))
        self.path_label.setText("Soubor: " + str(Path(filename).name))
        self.import_count.setText(f"{len(data)} pozic načteno (dosažitelnost není ověřena)")
        self.total_label.setText(f"Importované pozice: {len(data)}")
        self.measure_count.setText(f"Načteno {len(data)} pozic · žádná zatím nebyla změřena")
        self.statusBar().showMessage(f"Načteno {len(data)} pozic ze souboru {filename}")
        self._update_start_button()

    def _measurement(self):
        page, lay = self._page(
            "Spuštění ROS2 sekvence: Start pouze zahájí controller. "
            "Výsledek služby nepotvrzuje dokončení celé série."
        )
        self.robot_name_edit = QComboBox()
        self.robot_name_edit.addItems([x["name"] for x in load_profiles()["robots"]])
        self.series_name_edit = QLineEdit()
        self.series_name_edit.setPlaceholderText("Např. Opakovatelnost – série 01")
        names = QFormLayout()
        names.addRow("Robot", self.robot_name_edit)
        names.addRow("Název nové série", self.series_name_edit)
        lay.addLayout(names)
        self.measure_count = QLabel("Dosud nebyly importovány pozice")
        self.measure_state = QLabel("Stav: IDLE")
        lay.addWidget(self.measure_state)
        lay.addWidget(group("Připravená sekvence", self.measure_count))
        self.controller_status = QLabel("Controller: nedostupný")
        lay.addWidget(self.controller_status)
        self.measure_progress = QProgressBar()
        self.measure_progress.setRange(0, 100)
        self.measure_progress.setValue(0)
        lay.addWidget(self.measure_progress)
        self.measure_progress.setFormat("%v / %m bodů")
        self.arm_checkbox = QCheckBox(
            "Potvrzuji, že pohybová oblast je bezpečná a robot je připraven."
        )
        self.arm_checkbox.toggled.connect(self._update_start_button)
        lay.addWidget(self.arm_checkbox)
        self.start_button = QPushButton("Spustit automatické měření")
        self.start_button.setEnabled(False)
        self.start_button.clicked.connect(self._start_measurement)
        lay.addWidget(self.start_button)
        self.stop_button = QPushButton("Zastavit sekvenci po aktuálním kroku")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self._stop_measurement)
        lay.addWidget(self.stop_button)
        self.measure_log = QPlainTextEdit()
        self.measure_log.setReadOnly(True)
        self.measure_log.setPlaceholderText("Události měření...")
        lay.addWidget(group("Protokol událostí", self.measure_log))
        lay.addWidget(group("Poznámka", explanation(
            "Stav je načítán z /measurement/status. STOP dokončí již zahájený pohyb nebo měření; nenahrazuje nouzové zastavení KUKA."
        )))
        return page

    def _on_service_status(self, available):
        self._service_ready = available
        self.controller_status.setText(
            "Controller: služba dostupná" if available
            else "Controller: služba nedostupná"
        )
        self._update_start_button()

    def _update_start_button(self):
        if not hasattr(self, "start_button"):
            return
        ready = (getattr(self, "_service_ready", False)
                 and bool(self.source_path)
                 and bool(self.rows)
                 and self.format_combo.currentIndex() == 0
                 and self.arm_checkbox.isChecked()
                 and not getattr(self, "_start_pending", False)
                 and not getattr(self, "_start_accepted", False))
        self.start_button.setEnabled(ready)
        self._update_stop_button()

    def _update_stop_button(self):
        if not hasattr(self, "stop_button") or not hasattr(self, "ros"):
            return
        active = (getattr(self, "_start_pending", False)
                  or getattr(self, "_start_accepted", False))
        self.stop_button.setEnabled(
            active and self.ros.stop_client.service_is_ready()
            and not getattr(self, "_stop_pending", False)
        )

    def _stop_measurement(self):
        answer = QMessageBox.question(
            self, "Zastavit měření",
            "Zastavit sekvenci po dokončení probíhajícího kroku?\n"
            "Toto tlačítko NEZASTAVUJE pohyb robota nouzově.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if answer != QMessageBox.Yes:
            return
        try:
            self._stop_pending = True
            self._update_stop_button()
            self.ros.stop_measurement()
            self.measure_log.appendPlainText("Požadavek STOP odeslán...")
        except Exception as exc:
            self._stop_pending = False
            self._update_stop_button()
            QMessageBox.warning(self, "STOP nebyl odeslán", str(exc))

    def _on_stop_result(self, success, message):
        self._stop_pending = False
        self.measure_log.appendPlainText(("STOP přijat: " if success else "STOP odmítnut: ") + message)
        if not success:
            QMessageBox.warning(self, "STOP odmítnut", message)
        self._update_stop_button()

    def _start_robot_calibration(self):
        if not self.calibration_csv:
            QMessageBox.warning(self, 'Kalibrace', 'Nejprve vyber samostatné kalibrační CSV.')
            return
        answer = QMessageBox.question(self, 'Kalibrace báze',
            f'Spustit kalibrační pohyby ze souboru {self.calibration_csv}? Robot se začne pohybovat. '
            'Zkontroluj limity, trajektorii a bezpečnost pracoviště.',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer != QMessageBox.Yes:
            return
        try:
            if self.calib_robot_name.currentText() != load_profiles()["selected_robot"]:
                raise ValueError("Kalibrace vyžaduje aktivního robota ze záložky Nastavení")
            self.ros.start_robot_calibration(self.calibration_csv, self.calib_robot_name.currentText(), self.calib_series_name.text())
            self.calib_info.setText('Odesílám požadavek na kalibraci…')
        except Exception as exc:
            QMessageBox.warning(self, 'Kalibrace', str(exc))

    def _stop_robot_calibration(self):
        try:
            self.ros.stop_robot_calibration()
            self.calib_info.setText('Požaduji zastavení kalibrace…')
        except Exception as exc:
            QMessageBox.warning(self, 'Zastavení kalibrace', str(exc))

    def _on_calibration_result(self, success, message):
        self.calib_info.setText(('Kalibrace přijata: ' if success else 'Kalibrace odmítnuta: ') + message)

    def _on_calibration_stop_result(self, success, message):
        self.calib_info.setText(('STOP přijat: ' if success else 'STOP odmítnut: ') + message)

    def _on_calibration_status(self, data):
        state = str(data.get('state', '?'))
        done = int(data.get('completed', 0))
        total = int(data.get('total', 0))
        self.calib_progress.setRange(0, max(total, 1))
        self.calib_progress.setValue(min(done, max(total, 1)))
        self.calib_info.setText(f'{state} – {done}/{total} – {data.get("message", "")} '
                                f' (série {data.get("calibration_id", "")})')

    def _start_measurement(self):
        if not self.start_button.isEnabled():
            return
        answer = QMessageBox.question(
            self, "Potvrdit měření",
            f"Opravdu zahájit automatické měření {len(self.rows)} poloh?\n"
            "Požadavek může uvést robota do pohybu.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if answer != QMessageBox.Yes:
            return
        try:
            self._start_pending = True
            self._update_start_button()
            if self.robot_name_edit.currentText() != load_profiles()["selected_robot"]:
                raise ValueError("Pro měření vyber robota nastaveného jako aktivní v záložce Nastavení")
            if not self.robot_name_edit.currentText().strip() or not self.series_name_edit.text().strip():
                raise ValueError("Vyplň robota a název měřicí série")
            self.ros.start_measurement(self.source_path, self.robot_name_edit.currentText().strip(),
                                       self.series_name_edit.text().strip())
            self.measure_log.appendPlainText("Odesílám požadavek na start...")
        except Exception as exc:
            self._start_pending = False
            self._update_start_button()
            QMessageBox.warning(self, "Start se nezdařil", str(exc))

    def _on_start_result(self, success, message):
        self._start_pending = False
        if success:
            self._start_accepted = True
            self.measure_log.appendPlainText("Start přijat: " + message)
            self.statusBar().showMessage("Controller přijal měření; průběh zatím není připojen")
        else:
            self.measure_log.appendPlainText("Start odmítnut: " + message)
            QMessageBox.warning(self, "Start odmítnut", message)
        self._update_start_button()

    def _on_measurement_status(self, data):
        state = str(data.get("state", "UNKNOWN"))
        self._latest_sequence_state = state.upper()
        completed = max(0, int(data.get("completed", 0)))
        total = max(0, int(data.get("total", 0)))
        current = max(0, int(data.get("current_point", 0)))
        message = str(data.get("message", ""))

        self.measure_state.setText(f"Stav: {state}" + (f" – {message}" if message else ""))
        self.measure_progress.setRange(0, max(total, 1))
        self.measure_progress.setValue(min(completed, max(total, 1)))
        self.measure_count.setText(
            f"Dokončeno {completed} / {total} bodů"
            + (f" · aktuální bod: {current}" if current else "")
        )
        signature = (state, completed, total, current, message)
        if signature != getattr(self, "_last_measurement_status", None):
            self._last_measurement_status = signature
            self.measure_log.appendPlainText(
                f"[{state}] {completed}/{total}" + (f" – {message}" if message else "")
            )
        if state in ("FINISHED", "STOPPED", "ERROR", "IDLE"):
            self._start_accepted = False
            self._stop_pending = False
            self._update_start_button()

    def closeEvent(self, event):
        if self._sequence_running():
            QMessageBox.warning(self, "Probíhá sekvence", "Před ukončením UI zastav měřicí sekvenci a počkej na dokončení aktuální operace.")
            event.ignore()
            return
        if hasattr(self, "export_worker") and self.export_worker.isRunning():
            QMessageBox.information(self, "Probíhá export", "Před zavřením počkej na dokončení exportu.")
            event.ignore()
            return
        self._module_timer.stop()
        if self._leica_publisher is not None and self._leica_publisher.state() != QProcess.NotRunning:
            self._leica_stopping = True
            self._leica_publisher.terminate()
            self._leica_publisher.waitForFinished(1500)
            if self._leica_publisher.state() != QProcess.NotRunning:
                QMessageBox.warning(self, "Leica stále běží", "Publisher se nepodařilo ukončit. Zkontroluj jej v terminálu.")
                event.ignore()
                return
        for process in getattr(self, "_processes", {}).values():
            if process.state() != QProcess.NotRunning:
                process.terminate()
                process.waitForFinished(1500)
                if process.state() != QProcess.NotRunning:
                    QMessageBox.warning(self, "Uzel stále běží", "Některý uzel se neukončil. Před uzavřením UI jej bezpečně ukonči.")
                    event.ignore()
                    return
        self.ros.close()
        event.accept()

    def _load_calibration_csv(self):
        filename, _ = QFileDialog.getOpenFileName(self, "Kalibrace B – CSV", "", "CSV (*.csv)")
        if not filename:
            return
        try:
            from measurement_system.measurement.position_importer import load_positions
            positions, point_ids = load_positions(filename)
            if len(positions) < 3:
                raise ValueError("Kalibrace B vyžaduje alespoň 3 polohy")
            self.calibration_csv = filename
            self.calibration_csv_label.setText(f"{filename} ({len(positions)} bodů)")
        except Exception as exc:
            QMessageBox.warning(self, "Chybné kalibrační CSV", str(exc))

    def _create_table_series(self):
        if self._sequence_running():
            QMessageBox.warning(self, "Série", "Nejprve dokonči automatické měření")
            return
        try:
            self.ros.create_table_series(self.table_robot_name.currentText(), self.table_series_name.text())
            self.table_calibration_status.setText("Zakládám sérii…")
        except Exception as exc:
            QMessageBox.warning(self, "Série", str(exc))

    def _table_series_done(self, success, measurement_id, message):
        if success:
            self.table_series_id.setValue(measurement_id)
            self._table_next = 0
            for row in range(3):
                for col in (1, 2, 3):
                    self.table_calibration.setItem(row, col, QTableWidgetItem(""))
            self.table_calibration_status.setText(f"Nová série ID {measurement_id}; připraveno pro P1")
        else:
            QMessageBox.warning(self, "Série", message)

    def _measure_table_point(self):
        if self._sequence_running():
            QMessageBox.warning(self, "Leica", "Probíhá automatické měření")
            return
        if self._table_next >= 3:
            answer = QMessageBox.question(self, "Nová kalibrace stolu",
                                          "Všechny tři body jsou změřené. Začít znovu od P1?")
            if answer != QMessageBox.Yes:
                return
            self._table_next = 0
        if self.table_series_id.value() <= 0:
            QMessageBox.warning(self, "Kalibrace", "Zvol existující ID měřicí série nebo založ novou")
            return
        try:
            self.ros.table_measure(self.table_series_id.value(), self._table_next + 1)
            self.table_calibration_status.setText(f"Čekám na nový Leica vzorek pro P{self._table_next+1}…")
        except Exception as exc:
            QMessageBox.warning(self, "Leica", str(exc))

    def _table_point_done(self, data):
        if not data.get("success"):
            self.table_calibration_status.setText(data.get("message", "Chyba Leica"))
            return
        import math
        d, h, v = data["distance"], data["hz"], data["vt"]
        # Expected Leica convention: Hz and Vt angles in radians, Vt is zenith angle.
        # Validate this against the actual tracker protocol before applying transform.
        x = d * math.sin(v) * math.cos(h)
        y = d * math.sin(v) * math.sin(h)
        z = d * math.cos(v)
        for col, value in enumerate((x, y, z), 1):
            self.table_calibration.setItem(self._table_next, col, QTableWidgetItem(f"{value:.6f}"))
        self._table_next += 1
        self.table_calibration_status.setText(
            f"Uloženo do DB (ID bodu {data.get('point_id')}, série {self.table_series_id.value()}); hotovo {self._table_next}/3; Leica spherical conversion assumes radians / zenith angle. "
            + (f"Další P{self._table_next+1}" if self._table_next < 3 else "Všechny body hotové"))

    def _manual_page(self):
        page, lay = self._page("Ruční pohyb v kloubovém prostoru (stupně), odečet Leica bez uložení do databáze.")
        self.manual_joints = []
        form = QFormLayout()
        for i in range(7):
            editor = QDoubleSpinBox()
            editor.setRange(-360.0, 360.0)
            editor.setDecimals(3)
            editor.setSuffix(" °")
            self.manual_joints.append(editor)
            form.addRow(f"q{i+1}", editor)
        self.manual_velocity = QDoubleSpinBox()
        self.manual_velocity.setRange(0.01, 1.0)
        self.manual_velocity.setSingleStep(0.05)
        self.manual_velocity.setValue(0.1)
        form.addRow("Relativní rychlost", self.manual_velocity)
        lay.addLayout(form)
        self.manual_arm = QCheckBox("Pracovní prostor je bezpečný; potvrzuji ruční pohyb")
        lay.addWidget(self.manual_arm)
        self.manual_move_button = QPushButton("Pohnout robotem")
        self.manual_move_button.clicked.connect(self._manual_move)
        lay.addWidget(self.manual_move_button)
        self.manual_sample_button = QPushButton("Změřit polohu Leica (bez uložení)")
        self.manual_sample_button.clicked.connect(self._manual_measure)
        lay.addWidget(self.manual_sample_button)
        self.manual_result = QPlainTextEdit()
        self.manual_result.setReadOnly(True)
        lay.addWidget(self.manual_result)
        return page

    def _manual_move(self):
        if self._sequence_running():
            QMessageBox.warning(self, "Robot", "Běží automatická sekvence")
            return
        if not self.manual_arm.isChecked():
            QMessageBox.warning(self, "Robot", "Potvrď bezpečný pracovní prostor")
            return
        reply = QMessageBox.question(self, "Potvrdit ruční pohyb", "Opravdu odeslat pohyb robota?",
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply != QMessageBox.Yes:
            return
        try:
            self.ros.manual_move([w.value() for w in self.manual_joints], self.manual_velocity.value())
            self.manual_move_button.setEnabled(False)
            self.manual_arm.setChecked(False)  # Confirm anew for each manual move.
            self.manual_result.appendPlainText("Příkaz pohybu odeslán…")
        except Exception as exc:
            QMessageBox.warning(self, "Robot", str(exc))

    def _manual_move_done(self, success, message):
        self.manual_move_button.setEnabled(True)
        self.manual_result.appendPlainText(("Pohyb hotov: " if success else "Chyba pohybu: ") + message)

    def _manual_measure(self):
        if self._sequence_running():
            QMessageBox.warning(self, "Leica", "Probíhá automatické měření")
            return
        try:
            self.ros.manual_measure()
            self.manual_result.appendPlainText("Čekám na Leica data…")
        except Exception as exc:
            QMessageBox.warning(self, "Leica", str(exc))

    def _manual_leica_done(self, data):
        if data.get("success"):
            self.manual_result.appendPlainText(
                f"Leica t={data['timestamp']}, distance={data['distance']:.6f}, "
                f"Hz={data['hz']:.6f}, Vt={data['vt']:.6f} (do DB neuloženo)")
        else:
            self.manual_result.appendPlainText("Chyba Leica: " + data.get("message", "?"))

    def _export(self):
        page, lay = self._page("Export surových záznamů z PostgreSQL; výpočty citlivosti zůstávají mimo aplikaci.")
        self.export_table_select = QComboBox()
        self.export_table_select.addItems(["measurement_data", "table_calibration_points", "robot_base_calibration_points", "robot_base_calibration_runs"])
        lay.addWidget(group("Databázová tabulka", self.export_table_select))
        buttons = QHBoxLayout()
        self.btn_csv = QPushButton("Export do CSV")
        self.btn_h5 = QPushButton("Export do HDF5")
        self.btn_csv.clicked.connect(lambda: self._start_export("csv"))
        self.btn_h5.clicked.connect(lambda: self._start_export("h5"))
        buttons.addWidget(self.btn_csv)
        buttons.addWidget(self.btn_h5)
        lay.addLayout(buttons)
        self.export_status = QLabel("Připraveno. Připojení k databázi proběhne až po kliknutí.")
        self.export_status.setWordWrap(True)
        lay.addWidget(self.export_status)
        lay.addWidget(explanation("PostgreSQL: PGHOST, PGPORT, PGDATABASE, PGUSER, PGPASSWORD (nebo ~/.pgpass). Exportuje se celá zvolená tabulka."))
        lay.addStretch()
        return page

    def _start_export(self, fmt):
        table = self.export_table_select.currentText()
        ext = ".csv" if fmt == "csv" else ".h5"
        filter_name = "CSV (*.csv)" if fmt == "csv" else "HDF5 (*.h5)"
        path, _ = QFileDialog.getSaveFileName(self, "Uložit měření", f"{table}{ext}", filter_name)
        if not path:
            return
        if not path.lower().endswith(ext):
            path += ext
        self.btn_csv.setEnabled(False)
        self.btn_h5.setEnabled(False)
        self.export_status.setText("Načítám data z PostgreSQL a exportuji…")
        self.export_worker = ExportWorker(path, table)
        self.export_worker.finished_ok.connect(self._export_finished)
        self.export_worker.failed.connect(self._export_failed)
        self.export_worker.start()

    def _export_finished(self, path, count):
        self.btn_csv.setEnabled(True)
        self.btn_h5.setEnabled(True)
        self.export_status.setText(f"Exportováno {count} řádků do {path}")
        QMessageBox.information(self, "Export dokončen", f"Uloženo {count} řádků.\n{path}")

    def _export_failed(self, message):
        self.btn_csv.setEnabled(True)
        self.btn_h5.setEnabled(True)
        self.export_status.setText("Export selhal: " + message)
        QMessageBox.warning(self, "Export se nezdařil", message)

    def _settings(self):
        page, lay = self._page(
            "Nastavení PostgreSQL platí po restartování databázového a kalibračního uzlu. "
            "Heslo se ukládá mimo Git do uživatelského souboru s oprávněním 0600."
        )
        current = load_settings()
        self.pg_fields = {}
        form = QFormLayout()
        for key, label, default in [
            ("host", "Server / IP", "127.0.0.1"),
            ("port", "Port", "5432"),
            ("dbname", "Název databáze", "laser_tracker_data"),
            ("user", "Uživatel", ""),
            ("password", "Heslo", ""),
        ]:
            edit = QLineEdit(str(current.get(key, default)))
            if key == "password":
                edit.setEchoMode(QLineEdit.Password)
            form.addRow(label, edit)
            self.pg_fields[key] = edit
        panel = QWidget()
        panel.setLayout(form)
        lay.addWidget(group("PostgreSQL připojení", panel))
        test_btn = QPushButton("Otestovat připojení")
        test_btn.clicked.connect(self._test_pg_settings)
        save_btn = QPushButton("Uložit nastavení")
        save_btn.clicked.connect(self._save_pg_settings)
        lay.addWidget(test_btn)
        lay.addWidget(save_btn)
        self.pg_status = QLabel("Pro použití změn restartuj databázový i kalibrační uzel.")
        self.pg_status.setWordWrap(True)
        lay.addWidget(self.pg_status)

        profiles = load_profiles()
        self.robot_list = QComboBox()
        self.robot_list.addItems([x["name"] for x in profiles["robots"]])
        self.robot_list.setCurrentText(profiles.get("selected_robot", ""))
        self.robot_list.currentTextChanged.connect(self._robot_profile_changed)
        self.robot_host = QLineEdit()
        self.robot_port = QSpinBox()
        self.robot_port.setRange(1, 65535)
        self.robot_port.setValue(30001)
        robot_form = QFormLayout()
        robot_form.addRow("Robot / typ", self.robot_list)
        robot_form.addRow("IP adresa robota", self.robot_host)
        robot_form.addRow("TCP port KUKA", self.robot_port)
        buttons = QHBoxLayout()
        add_btn = QPushButton("Přidat robota")
        add_btn.clicked.connect(self._add_robot)
        buttons.addWidget(add_btn)
        save_robot_btn = QPushButton("Uložit robota a vybrat")
        save_robot_btn.clicked.connect(self._save_robot)
        buttons.addWidget(save_robot_btn)
        robot_form.addRow(buttons)
        lay.addWidget(group("Robot / připojení KUKA", robot_form))
        self._robot_profile_changed(self.robot_list.currentText())

        leica = profiles["leica"]
        self.leica_interface = QLineEdit(leica.get("interface", "eno4"))
        self.leica_host = QLineEdit(leica.get("host", "127.0.0.1"))
        self.leica_port = QSpinBox()
        self.leica_port.setRange(1, 65535)
        self.leica_port.setValue(int(leica.get("port", 5555)))
        leica_form = QFormLayout()
        leica_form.addRow("EtherCAT rozhraní", self.leica_interface)
        leica_form.addRow("ZMQ TCP host", self.leica_host)
        leica_form.addRow("ZMQ TCP port", self.leica_port)
        save_leica_btn = QPushButton("Uložit Leica")
        save_leica_btn.clicked.connect(self._save_leica)
        leica_form.addRow(save_leica_btn)
        lay.addWidget(group("Leica / ZMQ", leica_form))
        lay.addWidget(explanation("Připojení k zařízením se načítá při spuštění uzlů. Po změně restartuj příslušný uzel a Leica publisher. ZMQ host je adresa publisheru, není to EtherCAT IP."))
        lay.addStretch()
        return page

    def _robot_profile_changed(self, name):
        item = next((x for x in load_profiles()["robots"] if x["name"] == name), None)
        if item is not None:
            self.robot_host.setText(item["host"])
            self.robot_port.setValue(int(item["port"]))

    def _sync_robot_selectors(self, names, chosen):
        for key in ("robot_name_edit", "table_robot_name", "calib_robot_name"):
            editor = getattr(self, key, None)
            if editor is not None:
                editor.blockSignals(True)
                editor.clear()
                editor.addItems(names)
                editor.setCurrentText(chosen)
                editor.blockSignals(False)

    def _add_robot(self):
        name, accepted = QInputDialog.getText(self, "Nový robot", "Název / typ robota:")
        name = name.strip()
        if not accepted or not name:
            return
        data = load_profiles()
        if any(x["name"] == name for x in data["robots"]):
            QMessageBox.warning(self, "Robot", "Robot s tímto názvem už existuje.")
            return
        data["robots"].append({"name": name, "host": "172.31.1.147", "port": 30001})
        data["selected_robot"] = name
        save_profiles(data)
        self.robot_list.addItem(name)
        self.robot_list.setCurrentText(name)
        self._sync_robot_selectors([x["name"] for x in data["robots"]], name)

    def _save_robot(self):
        name = self.robot_list.currentText()
        host = self.robot_host.text().strip()
        if not name or not host:
            QMessageBox.warning(self, "Robot", "Zadej robota a IP/hostname.")
            return
        data = load_profiles()
        item = next((x for x in data["robots"] if x["name"] == name), None)
        if item is None:
            return
        item.update(host=host, port=self.robot_port.value())
        data["selected_robot"] = name
        save_profiles(data)
        self._sync_robot_selectors([x["name"] for x in data["robots"]], name)
        QMessageBox.information(self, "Robot", "Uloženo. Před připojením restartuj robot_node.")

    def _save_leica(self):
        host = self.leica_host.text().strip()
        interface = self.leica_interface.text().strip()
        if not host or not interface:
            QMessageBox.warning(self, "Leica", "Vyplň ZMQ host a EtherCAT rozhraní.")
            return
        data = load_profiles()
        data["leica"] = {"host": host, "port": self.leica_port.value(), "interface": interface}
        save_profiles(data)
        QMessageBox.information(self, "Leica", "Uloženo. Restartuj publisher a databázový uzel.")

    def _pg_values(self):
        return {key: editor.text().strip() for key, editor in self.pg_fields.items()}

    def _test_pg_settings(self):
        try:
            conn = connect_with_settings(self._pg_values())
            try:
                with conn.cursor() as cur:
                    cur.execute("SELECT 1")
                    cur.fetchone()
            finally:
                conn.close()
            self.pg_status.setText("PostgreSQL: připojení je funkční (SELECT 1 OK).")
        except Exception as exc:
            self.pg_status.setText("PostgreSQL: chyba připojení. " + str(exc))

    def _save_pg_settings(self):
        try:
            save_settings(self._pg_values())
            self.pg_status.setText(
                "Nastavení uloženo. Restartuj database_node a robot_calibration_node, "
                "případně UI kvůli exportu."
            )
        except Exception as exc:
            QMessageBox.warning(self, "Nastavení PostgreSQL", str(exc))

    def _select(self, index):
        self.stack.setCurrentIndex(index)
        self.heading.setText(PAGES[index])
        for i, button in enumerate(self.nav):
            button.setChecked(i == index)

    def _style(self):
        checkbox_icon = (Path(__file__).resolve().parent / "black_tick.svg").as_posix()
        QApplication.instance().setStyleSheet("""
            QWidget { color: #151b26; }
            QMainWindow, #content { background: #f5f7fb; }
            #sidebar { background: #152238; }
            #brand { font-size: 21px; font-weight: 800; color: white; letter-spacing: 2px; }
            #sidebar #description { color: #9bb0cc; }
            QPushButton#nav { background: transparent; border: 0; text-align: left;
                              color: #d0d9e8; padding: 13px; border-radius: 7px; }
            QPushButton#nav:checked { background: #2d4e79; color: white; font-weight: bold; }
            QPushButton#nav:hover { background: #28405e; }
            #heading { font-size: 27px; font-weight: 750; color: #162338; }
            #offline { background: #e9edf5; border-radius: 13px; padding: 8px 13px; color: #52627c; }
            #description { color: #66748a; font-size: 12px; }
            QLabel#moduleState { color: #1f2937; font-size: 13px; }
            QGroupBox { background: white; border: 1px solid #dfe5ed; border-radius: 10px;
                        margin-top: 14px; padding: 18px 12px 12px; font-weight: bold; color: #263a55; }
            QGroupBox::title { subcontrol-origin: margin; left: 14px; padding: 0 5px; }
            #statusCard { background: white; border: 1px solid #dfe5ed; border-radius: 10px;
                          padding: 20px; color: #283e5e; font-size: 15px; }
            QPushButton { background: #245caa; color: white; border: none;
                          border-radius: 7px; padding: 11px 15px; font-weight: bold; }
            QPushButton:disabled { background: #dce2ea; color: #7b8797; }
            QPushButton:hover:!disabled { background: #1c4f95; }
            QPlainTextEdit, QTextEdit { background: white; color: #151b26; }
            QTableWidget { background: white; color: #151b26; border: 1px solid #dfe5ed; gridline-color: #edf0f4; }
            QHeaderView::section { color: #151b26; background: #e9edf4; border: none; padding: 9px; font-weight: bold; }
            /* Form controls, including editable lists and popup menus */
            QComboBox, QLineEdit, QDoubleSpinBox, QSpinBox {
                color: #151b26; background-color: #ffffff;
                border: 1px solid #dce2eb; border-radius: 5px; padding: 8px;
                selection-background-color: #dceafd; selection-color: #111827;
            }
            QComboBox:editable { background-color: #ffffff; }
            QComboBox::drop-down { background-color: #f1f5f9; border: none;
                                    border-left: 1px solid #dce2eb; width: 22px; }
            QComboBox QAbstractItemView,
            QComboBox QListView, QListView, QListWidget, QTreeView, QTableView {
                color: #151b26; background-color: #ffffff;
                alternate-background-color: #f6f8fc;
                selection-background-color: #dceafd;
                selection-color: #111827;
                border: 1px solid #dce2eb;
                outline: 0;
            }
            QAbstractItemView::item { color: #151b26; background-color: #ffffff; }
            QAbstractItemView::item:selected { color: #111827; background-color: #dceafd; }
            QAbstractItemView::item:hover { color: #111827; background-color: #edf4ff; }
            QMenu { background-color: #ffffff; color: #151b26;
                    border: 1px solid #dce2eb; }
            QMenu::item { background-color: #ffffff; color: #151b26; padding: 6px 20px; }
            QMenu::item:selected { background-color: #dceafd; color: #111827; }
            /* QMessageBox, QInputDialog and file-selection dialogues */
            QDialog, QMessageBox, QInputDialog, QFileDialog {
                background-color: #ffffff; color: #151b26;
            }
            QDialog QLabel, QMessageBox QLabel, QInputDialog QLabel,
            QFileDialog QLabel { color: #151b26; background: transparent; }
            QDialog QGroupBox { background-color: #ffffff; color: #151b26; }
            QToolTip { background-color: #ffffff; color: #151b26;
                       border: 1px solid #9aa7b7; padding: 5px; }
            QCheckBox { color: #151b26; spacing: 8px; }
            QCheckBox::indicator { width: 18px; height: 18px; background: white;
                                   border: 2px solid black; border-radius: 3px; }
            QCheckBox::indicator:checked { image: url(CHECKBOX_ICON_PATH); background: white;
                                           border: 2px solid black; }
            QProgressBar { border: 1px solid #d7dfe9; border-radius: 6px; height: 20px; text-align: center; }
            QProgressBar::chunk { background: #245caa; }
        """.replace("CHECKBOX_ICON_PATH", checkbox_icon))


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()