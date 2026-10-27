"""PySide6 concept interface for the Diplomka measurement system.

StartMeasurement may command robot motion through the ROS2 controller.
The user must explicitly arm the start button.
"""
import csv
from pathlib import Path
import sys

from PySide6.QtCore import Qt, QThread, Signal, QProcess, QTimer
import shutil
from measurement_system.UI.ros_bridge import RosBridge
from measurement_system.database_export import export_table
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QFileDialog, QFormLayout, QFrame, QGridLayout,
    QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMainWindow,
    QMessageBox, QPlainTextEdit, QProgressBar, QPushButton, QSpinBox,
    QStackedWidget, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

PAGES = ["Přehled", "Kalibrace", "Import pozic", "Automatické měření", "Export dat", "Nastavení"]
JOINT_COLUMNS = ["point_id"] + [f"q{i}" for i in range(1, 8)]
CART_COLUMNS = ["point_id", "x", "y", "z", "rx", "ry", "rz"]


def group(title, *widgets):
    frame = QGroupBox(title)
    layout = QVBoxLayout(frame)
    for widget in widgets:
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
        self._module_timer = QTimer(self)
        self._module_timer.timeout.connect(self._refresh_modules)
        self._module_timer.start(1000)
        self._processes = {}
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
            button.clicked.connect(lambda checked=False, k=key: self._connect_module(k))
            column.addWidget(button)
            self.module_labels[key] = label
            self.module_buttons[key] = button
            grid.addWidget(box, i // 2, i % 2)
        lay.addLayout(grid)
        lay.addWidget(explanation(
            "Leica: Při kliknutí spustíme monitor ZMQ na tcp://localhost:5555. "
            "Pro skutečné spuštění publikačního driveru musí běžet leica-driver "
            "samostatně nebo být nakonfigurován systémově."
        ))
        self.total_label = QLabel("Importované pozice: 0")
        lay.addWidget(group("Aktuální série", self.total_label))
        lay.addStretch()
        return page

    def _connect_module(self, key):
        existing = self._processes.get(key)
        if existing is not None and existing.state() != QProcess.NotRunning:
            return
        if key == "leica":
            command = [sys.executable, "-m", "measurement_system.leica_monitor"]
        else:
            exe = shutil.which("ros2")
            if not exe:
                QMessageBox.warning(self, "ROS2", "Příkaz ros2 není v PATH. Spusť UI ze sourcovaného ROS2 prostředí.")
                return
            names = {"robot": "robot_node", "database": "database_node", "controller": "controller_node", "calibration": "robot_calibration_node"}
            command = [exe, "run", "measurement_system", names[key]]
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

    def _process_output(self, key, process):
        message = bytes(process.readAllStandardOutput()).decode("utf-8", errors="replace").strip()
        if message and ("ERROR" in message or "Traceback" in message):
            self.statusBar().showMessage(f"{key}: {message[-200:]}", 10000)

    def _process_error(self, key, process):
        self.module_labels[key].setText(f"Chyba spuštění: {process.errorString()}")
        self.module_buttons[key].setEnabled(True)

    def _process_finished(self, key, code):
        self.module_labels[key].setText(f"Proces skončil (kód {code})")
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
            if ready:
                text = "Přijímá data ZMQ" if key == "leica" else "ROS2 služba dostupná (hardware neověřen)"
            elif alive:
                text = "Proces běží, čekám na službu/data"
            else:
                text = "Není dostupné"
            self.module_labels[key].setText(text)
            self.module_buttons[key].setEnabled(not alive and not ready)
            self.module_buttons[key].setText("Připojeno" if ready else "Připojit")

    def _calibration(self):
        page, lay = self._page("Kalibrace transformací Leica → stůl → báze robota. Měření bodů se doplní po napojení na tracker a ROS2.")
        table = QTableWidget(3, 4)
        table.setHorizontalHeaderLabels(["Bod", "X [mm]", "Y [mm]", "Z [mm]"])
        for i in range(3):
            table.setItem(i, 0, QTableWidgetItem(f"P{i + 1}"))
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table.setMaximumHeight(170)
        btn = QPushButton("Změřit bod Leica (zatím nedostupné)")
        btn.setEnabled(False)
        lay.addWidget(group("1. Souřadnicový systém stolu – 3 body", table, btn,
                            explanation("P1 = počátek, P1→P2 = osa X, P3 určuje stranu osy Y.")))
        self.calib_info = QLabel('Nezahájeno – nejprve připoj kalibrační uzel a importuj CSV s q1–q7.')
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
        if not self.source_path or not self.rows:
            QMessageBox.warning(self, 'Kalibrace', 'Nejdřív importuj CSV s kloubovými pozicemi q1–q7.')
            return
        if self.headers != JOINT_COLUMNS:
            QMessageBox.warning(self, 'Kalibrace', 'Kalibrace vyžaduje CSV point_id,q1,...,q7.')
            return
        answer = QMessageBox.question(self, 'Kalibrace báze',
            f'Spustit kalibrační pohyby do {len(self.rows)} pozic? Robot se začne pohybovat. '
            'Zkontroluj limity, trajektorii a bezpečnost pracoviště.',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer != QMessageBox.Yes:
            return
        try:
            self.ros.start_robot_calibration(self.source_path)
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
            self.ros.start_measurement(self.source_path)
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
        state = str(data.get("state", "UNKNOWN")).upper()
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
        if hasattr(self, "export_worker") and self.export_worker.isRunning():
            QMessageBox.information(self, "Probíhá export", "Před zavřením počkej na dokončení exportu.")
            event.ignore()
            return
        self._module_timer.stop()
        self.ros.close()
        for process in getattr(self, "_processes", {}).values():
            if process.state() != QProcess.NotRunning:
                process.terminate()
                if not process.waitForFinished(1500):
                    process.kill()
                    process.waitForFinished(1000)
        event.accept()

    def _export(self):
        page, lay = self._page("Export surových záznamů z PostgreSQL; výpočty citlivosti zůstávají mimo aplikaci.")
        self.export_table_select = QComboBox()
        self.export_table_select.addItems(["measurement_data", "table_plane_points", "robot_base_calibration_points"])
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
        page, lay = self._page("Konfigurace připojení bude oddělena od zdrojového kódu a nebude obsahovat hesla uložená v Gitu.")
        form = QFormLayout()
        for label, default in [("ROS2 domain ID", "0"), ("KUKA adresa", "nenastaveno"),
                               ("Leica endpoint", "tcp://localhost:5555"), ("Databáze", "PostgreSQL")]:
            edit = QLineEdit(default)
            edit.setReadOnly(True)
            form.addRow(label, edit)
        panel = QWidget()
        panel.setLayout(form)
        lay.addWidget(group("Konfigurace – pouze náhled", panel))
        lay.addStretch()
        return page

    def _select(self, index):
        self.stack.setCurrentIndex(index)
        self.heading.setText(PAGES[index])
        for i, button in enumerate(self.nav):
            button.setChecked(i == index)

    def _style(self):
        self.setStyleSheet("""
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
            QTableWidget { background: white; border: 1px solid #dfe5ed; gridline-color: #edf0f4; }
            QHeaderView::section { background: #e9edf4; border: none; padding: 9px; font-weight: bold; }
            QComboBox, QLineEdit { background: white; border: 1px solid #dce2eb; border-radius: 5px; padding: 8px; }
            QProgressBar { border: 1px solid #d7dfe9; border-radius: 6px; height: 20px; text-align: center; }
            QProgressBar::chunk { background: #245caa; }
        """)


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()