"""PySide6 concept interface for the Diplomka measurement system.

StartMeasurement may command robot motion through the ROS2 controller.
The user must explicitly arm the start button.
"""
import csv
from pathlib import Path
import sys

from PySide6.QtCore import Qt, QThread, Signal
from measurement_system.ros_bridge import RosBridge
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
        self.setWindowTitle("Diplomka | Measurement Control")
        self.resize(1250, 780)
        self.rows = []
        self.headers = []
        self.source_path = None
        self._start_pending = False
        self._start_accepted = False
        self._service_ready = False
        self._build()
        self._style()
        self.ros = RosBridge(self)
        self.ros.service_available.connect(self._on_service_status)
        self.ros.start_result.connect(self._on_start_result)
        self.ros.measurement_status.connect(self._on_measurement_status)
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
        page, lay = self._page("Centrální panel měřicího systému. Stavové informace budou načítány z ROS2 uzlů.")
        grid = QGridLayout()
        for i, (name, desc) in enumerate([
            ("KUKA LBR iiwa", "IIWA_Py3 · nezjištěno"),
            ("Leica Tracker", "ZeroMQ · nezjištěno"),
            ("PostgreSQL", "Připojení · nezjištěno"),
            ("ROS2", "Uzly · nezjištěno"),
        ]):
            title = QLabel(f"{name}\n\n{desc}")
            title.setObjectName("statusCard")
            grid.addWidget(title, i // 2, i % 2)
        lay.addLayout(grid)
        self.total_label = QLabel("Importované pozice: 0")
        lay.addWidget(group("Aktuální série", self.total_label,
                            explanation("Žádná měřicí série zatím nebyla vytvořena.")))
        lay.addStretch()
        return page

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
        base_btn = QPushButton("Spustit kalibraci báze (zatím nedostupné)")
        base_btn.setEnabled(False)
        lay.addWidget(group("2. Kalibrace báze – metoda B", explanation(
            "Porovnáme polohy reflektoru z dopředné kinematiky se souřadnicemi Leica v několika konfiguracích. "
            "Je nutné zohlednit offset reflektoru vůči přírubě/TCP."), base_btn))
        lay.addWidget(group("3. Výsledné transformace", explanation(
            "T_leica_table: neznámá     |     T_leica_robot: neznámá     |     T_robot_table: neznámá")))
        lay.addStretch()
        return page

    def _import(self):
        page, lay = self._page("Import CSV funguje. Zatím se pouze kontroluje formát a zobrazují data; nic se neposílá robotovi.")
        bar = QHBoxLayout()
        self.format_combo = QComboBox()
        self.format_combo.addItems(["Kloubové pozice (q1–q7)", "Kartézské pozice (x,y,z,rx,ry,rz)"])
        self.format_combo.currentIndexChanged.connect(self._refresh_import_hint)
        self.format_combo.currentIndexChanged.connect(lambda _index: self._update_start_button())
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
        self.start_requirements = QLabel("")
        self.start_requirements.setWordWrap(True)
        self.start_requirements.setObjectName("startRequirements")
        lay.addWidget(self.start_requirements)
        check_btn = QPushButton("Ověřit připojení ke controlleru")
        check_btn.setObjectName("secondaryButton")
        check_btn.clicked.connect(self._check_controller)
        lay.addWidget(check_btn)
        self.measure_log = QPlainTextEdit()
        self.measure_log.setReadOnly(True)
        self.measure_log.setPlaceholderText("Události měření...")
        lay.addWidget(group("Protokol událostí", self.measure_log))
        lay.addWidget(group("Poznámka", explanation(
            "Stav je načítán z /measurement/status. Zastavení sekvence dosud není implementováno."
        )))
        return page

    def _on_service_status(self, available):
        self._service_ready = available
        self.controller_status.setText(
            "Controller: služba dostupná" if available
            else "Controller: služba nedostupná"
        )
        self._update_start_button()

    def _check_controller(self):
        self.ros.check_service()
        self.measure_log.appendPlainText("Kontroluji službu /start_measurement…")

    def _update_start_button(self):
        if not hasattr(self, "start_button"):
            return

        reasons = []
        if not bool(self.source_path) or not bool(self.rows):
            reasons.append("Načti CSV s kloubovými pozicemi.")
        if self.format_combo.currentIndex() != 0:
            reasons.append("Pro řízení robota vyber kloubové pozice q1–q7.")
        if not getattr(self, "_service_ready", False):
            reasons.append("ROS2 služba /start_measurement není dostupná. Spusť controller_node a ověř definici StartMeasurement.srv.")
        if not self.arm_checkbox.isChecked():
            reasons.append("Potvrď připravenost robota a bezpečnou pracovní oblast.")
        if getattr(self, "_start_pending", False):
            reasons.append("Čekám na odpověď controlleru.")
        if getattr(self, "_start_accepted", False):
            reasons.append("Sekvence už byla přijata – další start až po dokončení.")

        self.start_button.setEnabled(not reasons)
        self.start_requirements.setText(
            "Připraveno ke spuštění." if not reasons
            else "Proč nelze spustit měření:\n• " + "\n• ".join(reasons)
        )
        self.start_button.setToolTip("\n".join(reasons))

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
            self.statusBar().showMessage("Controller přijal požadavek; sleduji /measurement/status")
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
        if state in ("FINISHED", "ERROR", "STOPPED"):
            self._start_accepted = False
            self._update_start_button()

    def closeEvent(self, event):
        if hasattr(self, "export_worker") and self.export_worker.isRunning():
            QMessageBox.information(self, "Probíhá export", "Před zavřením počkej na dokončení exportu.")
            event.ignore()
            return
        self.ros.close()
        event.accept()

    def _export(self):
        page, lay = self._page("Export surových záznamů z PostgreSQL; výpočty citlivosti zůstávají mimo aplikaci.")
        self.export_table_select = QComboBox()
        self.export_table_select.addItems(["measurement_data", "table_plane_points"])
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
        # Explicit text colors override Qt/OS dark-palette defaults.
        self.setStyleSheet("""
            QMainWindow, QWidget, QFrame#content, QStackedWidget {
                background: #f5f7fb; color: #18283d;
            }
            QLabel, QCheckBox, QGroupBox, QRadioButton {
                color: #18283d; background: transparent;
            }
            QFrame#sidebar { background: #152238; }
            #brand { font-size: 21px; font-weight: 800; color: #ffffff; letter-spacing: 2px; }
            #sidebar QLabel { color: #d5e0f0; background: transparent; }
            #sidebar QLabel#description { color: #b8cbe4; }
            QPushButton#nav { background: transparent; border: 0; text-align: left;
                              color: #e2e9f3; padding: 13px; border-radius: 7px; }
            QPushButton#nav:checked { background: #2d4e79; color: #ffffff; font-weight: bold; }
            QPushButton#nav:hover { background: #28405e; }
            #heading { font-size: 27px; font-weight: 750; color: #162338; }
            #offline { background: #e9edf5; border-radius: 13px; padding: 8px 13px; color: #3a4b63; }
            QLabel#description { color: #43536a; font-size: 12px; }
            QGroupBox { background: #ffffff; border: 1px solid #cfd8e5; border-radius: 10px;
                        margin-top: 14px; padding: 18px 12px 12px; font-weight: bold; color: #21344f; }
            QGroupBox::title { subcontrol-origin: margin; left: 14px; padding: 0 5px;
                               color: #21344f; background: #ffffff; }
            #statusCard { background: #ffffff; border: 1px solid #cfd8e5; border-radius: 10px;
                          padding: 20px; color: #203651; font-size: 15px; }
            QPushButton { background: #245caa; color: #ffffff; border: none;
                          border-radius: 7px; padding: 11px 15px; font-weight: bold; }
            QPushButton:disabled { background: #d6dfe9; color: #4b5870; }
            QPushButton:hover:!disabled { background: #1c4f95; }
            QPushButton#secondaryButton { background: #e5ebf4; color: #182f50;
                                          border: 1px solid #b9c9dc; }
            QPushButton#secondaryButton:hover { background: #d4e0ef; }
            QTableWidget, QTableView { background: #ffffff; color: #18283d;
                alternate-background-color: #eef3f9; selection-background-color: #d3e5ff;
                selection-color: #112c55; border: 1px solid #cfd8e5;
                gridline-color: #e2e8f0; }
            QTableWidget QTableCornerButton::section { background: #e9edf4; }
            QHeaderView::section { background: #e9edf4; color: #253c59; border: none;
                                   padding: 9px; font-weight: bold; }
            QComboBox, QLineEdit, QSpinBox, QPlainTextEdit, QTextEdit {
                background: #ffffff; color: #15263f;
                border: 1px solid #c4cfdd; border-radius: 5px; padding: 8px;
                selection-background-color: #c6ddfb; selection-color: #112c55;
            }
            QComboBox QAbstractItemView { background: #ffffff; color: #15263f;
                selection-background-color: #d3e5ff; selection-color: #112c55; }
            QPlainTextEdit { color: #15263f; }
            QProgressBar { background: #e8eef5; color: #172d49; border: 1px solid #c4cfdd;
                border-radius: 6px; height: 20px; text-align: center; }
            QProgressBar::chunk { background: #245caa; }
            QStatusBar { background: #e8edf5; color: #1d3452; }
            QLabel#startRequirements { background: #fff9e8; color: #62420a;
                border: 1px solid #e0cd94; border-radius: 6px; padding: 9px; }
            QMessageBox, QFileDialog { background: #f5f7fb; color: #18283d; }
        """)


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()