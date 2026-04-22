"""
ВПТК – Волновая передача с телами качения
UI-оболочка для генератора профиля редуктора.

Зависимости:
    pip install pyside6 matplotlib numpy ezdxf
"""

import sys
import numpy as np
import ezdxf

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QFormLayout, QGroupBox, QLabel, QSpinBox, QDoubleSpinBox,
    QCheckBox, QLineEdit, QPushButton, QStatusBar, QSizePolicy,
    QScrollArea, QFrame,
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont, QColor, QPalette

import matplotlib
matplotlib.use("QtAgg")
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


# ──────────────────────────────────────────────────────────────────────────────
# Ядро расчёта (отделено от GUI)
# ──────────────────────────────────────────────────────────────────────────────

class GearParams:
    """Хранит входные и расчётные параметры редуктора."""

    def __init__(self, i, dsh, Rout, D, resolution, u=1):
        self.i = i
        self.dsh = dsh
        self.Rout = Rout
        self.D = D
        self.resolution = resolution
        self.u = u

        self.e = 0.2 * dsh
        self.zg = (i + 1) * u
        self.zsh = i
        self.Rin = Rout - 2 * self.e
        self.rsh = dsh / 2
        self.rd = self.Rin + self.e - dsh
        self.hc = 2.2 * self.e
        self.Rsep_m = self.rd + self.rsh
        self.Rsep_out = self.Rsep_m + self.hc / 2
        self.Rsep_in = self.Rsep_m - self.hc / 2

    def validate(self):
        """Возвращает (ok: bool, message: str)."""
        limit = (1.03 * self.dsh) / np.sin(np.pi / self.zg)
        if self.Rin <= limit:
            return False, (
                f"Rin = {self.Rin:.2f} мм ≤ {limit:.2f} мм — "
                f"увеличьте Rout или уменьшите i"
            )
        if self.rd <= 0:
            return False, f"Радиус эксцентрика rd = {self.rd:.2f} мм ≤ 0 — увеличьте Rout"
        if self.Rsep_in <= 0:
            return False, "Внутренний радиус сепаратора ≤ 0"
        return True, "OK"

    def profile(self):
        """Возвращает (x, y) профиля жёсткого колеса."""
        theta = np.linspace(0, 2 * np.pi, self.resolution)
        S = np.sqrt((self.rsh + self.rd) ** 2 - (self.e * np.sin(self.zg * theta)) ** 2)
        l = self.e * np.cos(self.zg * theta) + S
        Xi = np.arctan2(self.e * self.zg * np.sin(self.zg * theta), S)
        x = l * np.sin(theta) + self.rsh * np.sin(theta + Xi)
        y = l * np.cos(theta) + self.rsh * np.cos(theta + Xi)
        return x, y

    def ball_centers(self):
        """Возвращает (x_sh, y_sh) центров шариков."""
        sh_angle = np.linspace(0, 1, self.zsh + 1) * 2 * np.pi
        S_sh = np.sqrt((self.rsh + self.rd) ** 2 - (self.e * np.sin(self.zg * sh_angle)) ** 2)
        l_sh = self.e * np.cos(self.zg * sh_angle) + S_sh
        return l_sh * np.sin(sh_angle), l_sh * np.cos(sh_angle)


def save_dxf(params: GearParams, path: str,
             show_base=True, show_sep=True, show_ecc=True,
             show_balls=False, show_out=True):
    x, y = params.profile()
    xy = np.stack((x, y), axis=1)
    x_sh, y_sh = params.ball_centers()

    doc = ezdxf.new("R2000")
    msp = doc.modelspace()

    if show_base:
        msp.add_point([0, 0])
        msp.add_lwpolyline(xy)

    if show_sep:
        msp.add_circle((0, 0), radius=params.Rsep_out)
        msp.add_circle((0, 0), radius=params.Rsep_in)

    if show_ecc:
        msp.add_point([0, params.e])
        msp.add_lwpolyline([[0, 0], [0, params.e]])
        msp.add_lwpolyline([[-6, 0], [6, 0]])
        msp.add_lwpolyline([[-3, params.e], [3, params.e]])
        msp.add_circle((0, params.e), radius=params.rd)

    if show_balls:
        for k in range(params.zsh):
            msp.add_circle((x_sh[k], y_sh[k]), radius=params.rsh)

    if show_out:
        msp.add_circle((0, 0), radius=params.D / 2)

    doc.saveas(path)


# ──────────────────────────────────────────────────────────────────────────────
# Matplotlib canvas
# ──────────────────────────────────────────────────────────────────────────────

class GearCanvas(FigureCanvas):
    def __init__(self):
        self.fig = Figure(figsize=(7, 7), tight_layout=True)
        self.ax = self.fig.add_subplot(111)
        super().__init__(self.fig)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.fig.patch.set_facecolor("#1e1e2e")
        self.ax.set_facecolor("#1e1e2e")

    def draw_gear(self, params: GearParams,
                  show_base=True, show_sep=True, show_ecc=True,
                  show_balls=True, show_out=True):
        self.ax.clear()
        self.ax.set_facecolor("#1e1e2e")
        self.ax.tick_params(colors="#cdd6f4")
        self.ax.spines[:].set_color("#45475a")
        self.ax.set_aspect("equal")
        self.ax.grid(True, color="#313244", linewidth=0.5)

        x, y = params.profile()
        x_sh, y_sh = params.ball_centers()

        if show_base:
            self.ax.plot(x, y, color="#89b4fa", linewidth=1.2, label="Профиль колеса")

        if show_sep:
            self.ax.add_patch(matplotlib.patches.Circle(
                (0, 0), params.Rsep_out, fill=False, color="#a6e3a1", linewidth=1.0, label="Сепаратор"))
            self.ax.add_patch(matplotlib.patches.Circle(
                (0, 0), params.Rsep_in, fill=False, color="#a6e3a1", linewidth=1.0))

        if show_ecc:
            self.ax.plot([0, 0], [0, params.e], "o-", color="#f38ba8",
                         markersize=4, linewidth=1.2, label="Эксцентрик")
            self.ax.plot([-6, 6], [0, 0], "--", color="#f38ba8", linewidth=0.8)
            self.ax.plot([-3, 3], [params.e, params.e], "--", color="#f38ba8", linewidth=0.8)
            self.ax.add_patch(matplotlib.patches.Circle(
                (0, params.e), params.rd, fill=False, color="#f38ba8", linewidth=1.0))

        if show_balls:
            for k in range(params.zsh):
                self.ax.add_patch(matplotlib.patches.Circle(
                    (x_sh[k], y_sh[k]), params.rsh,
                    fill=True, facecolor="#fab38720", edgecolor="#fab387",
                    linewidth=0.8))
            self.ax.plot([], [], color="#fab387", linewidth=1.0, label="Шарики")

        if show_out:
            self.ax.add_patch(matplotlib.patches.Circle(
                (0, 0), params.D / 2, fill=False, color="#cba6f7",
                linewidth=1.0, linestyle="--", label="Внешний Ø"))

        leg = self.ax.legend(facecolor="#313244", edgecolor="#45475a",
                             labelcolor="#cdd6f4", fontsize=8, loc="upper right")

        lim = params.D / 2 * 1.1
        self.ax.set_xlim(-lim, lim)
        self.ax.set_ylim(-lim, lim)
        self.ax.set_title("ВПТК — Профиль редуктора", color="#cdd6f4", fontsize=10)
        self.draw()


import matplotlib.patches  # нужен для Circle


# ──────────────────────────────────────────────────────────────────────────────
# Панель параметров
# ──────────────────────────────────────────────────────────────────────────────

def _spin(value, min_val, max_val, decimals=0, step=1.0):
    if decimals == 0:
        w = QSpinBox()
        w.setRange(int(min_val), int(max_val))
        w.setValue(int(value))
    else:
        w = QDoubleSpinBox()
        w.setDecimals(decimals)
        w.setRange(min_val, max_val)
        w.setSingleStep(step)
        w.setValue(value)
    w.setMinimumWidth(90)
    return w


class ParamPanel(QScrollArea):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWidgetResizable(True)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setFrameShape(QFrame.NoFrame)
        self.setFixedWidth(270)

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setSpacing(10)
        layout.setContentsMargins(8, 8, 8, 8)

        # ── Входные параметры ──────────────────────────────────
        grp_in = QGroupBox("Входные параметры")
        form = QFormLayout(grp_in)
        form.setLabelAlignment(Qt.AlignRight)
        form.setSpacing(6)

        self.spin_i    = _spin(17,  2,    200)
        self.spin_dsh  = _spin(6.0, 1.0,  50.0, 2, 0.5)
        self.spin_Rout = _spin(38.0, 5.0, 500.0, 2, 1.0)
        self.spin_D    = _spin(90.0, 10.0, 600.0, 2, 1.0)
        self.spin_res  = _spin(600,  100, 4000)

        self.spin_dsh.setSuffix(" мм")
        self.spin_Rout.setSuffix(" мм")
        self.spin_D.setSuffix(" мм")

        form.addRow("Передат. число i:", self.spin_i)
        form.addRow("Диам. шарика dsh:", self.spin_dsh)
        form.addRow("Rout (впадины):",   self.spin_Rout)
        form.addRow("Внешний диам. D:",  self.spin_D)
        form.addRow("Разрешение:",       self.spin_res)
        layout.addWidget(grp_in)

        # ── Расчётные параметры ────────────────────────────────
        grp_calc = QGroupBox("Расчётные параметры")
        calc_layout = QFormLayout(grp_calc)
        calc_layout.setLabelAlignment(Qt.AlignRight)
        calc_layout.setSpacing(4)

        mono = QFont("Monospace", 9)

        def calc_label():
            lbl = QLabel("—")
            lbl.setFont(mono)
            lbl.setAlignment(Qt.AlignRight)
            return lbl

        self.lbl_e       = calc_label()
        self.lbl_zg      = calc_label()
        self.lbl_zsh     = calc_label()
        self.lbl_Rin     = calc_label()
        self.lbl_rd      = calc_label()
        self.lbl_Rsep_m  = calc_label()
        self.lbl_hc      = calc_label()

        calc_layout.addRow("Эксцентриситет e:",  self.lbl_e)
        calc_layout.addRow("Число впадин zg:",   self.lbl_zg)
        calc_layout.addRow("Число шариков zsh:", self.lbl_zsh)
        calc_layout.addRow("Rin:",               self.lbl_Rin)
        calc_layout.addRow("Радиус экс. rd:",    self.lbl_rd)
        calc_layout.addRow("Rsep (дел.):",       self.lbl_Rsep_m)
        calc_layout.addRow("Толщина сеп. hc:",   self.lbl_hc)
        layout.addWidget(grp_calc)

        # ── Отображаемые слои ──────────────────────────────────
        grp_vis = QGroupBox("Отображение")
        vis_layout = QVBoxLayout(grp_vis)
        vis_layout.setSpacing(4)

        self.chk_base  = QCheckBox("Профиль жёсткого колеса")
        self.chk_sep   = QCheckBox("Сепаратор")
        self.chk_ecc   = QCheckBox("Эксцентрик")
        self.chk_balls = QCheckBox("Шарики")
        self.chk_out   = QCheckBox("Внешний диаметр")

        self.chk_base.setChecked(True)
        self.chk_sep.setChecked(True)
        self.chk_ecc.setChecked(True)
        self.chk_balls.setChecked(True)
        self.chk_out.setChecked(True)

        for chk in (self.chk_base, self.chk_sep, self.chk_ecc,
                    self.chk_balls, self.chk_out):
            vis_layout.addWidget(chk)
        layout.addWidget(grp_vis)

        # ── DXF ───────────────────────────────────────────────
        grp_dxf = QGroupBox("Экспорт DXF")
        dxf_layout = QVBoxLayout(grp_dxf)

        self.edit_file = QLineEdit("vptk_output.dxf")
        dxf_layout.addWidget(QLabel("Имя файла:"))
        dxf_layout.addWidget(self.edit_file)

        self.btn_save = QPushButton("💾  Сохранить DXF")
        self.btn_save.setMinimumHeight(34)
        dxf_layout.addWidget(self.btn_save)
        layout.addWidget(grp_dxf)

        layout.addStretch()
        self.setWidget(container)


# ──────────────────────────────────────────────────────────────────────────────
# Главное окно
# ──────────────────────────────────────────────────────────────────────────────

STYLE = """
QMainWindow, QWidget {
    background-color: #1e1e2e;
    color: #cdd6f4;
}
QGroupBox {
    border: 1px solid #45475a;
    border-radius: 6px;
    margin-top: 10px;
    font-weight: bold;
    color: #89b4fa;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 8px;
    padding: 0 4px;
}
QSpinBox, QDoubleSpinBox, QLineEdit {
    background-color: #313244;
    border: 1px solid #45475a;
    border-radius: 4px;
    color: #cdd6f4;
    padding: 2px 6px;
}
QSpinBox::up-button, QDoubleSpinBox::up-button,
QSpinBox::down-button, QDoubleSpinBox::down-button {
    background-color: #45475a;
    border-radius: 2px;
}
QCheckBox { color: #cdd6f4; spacing: 6px; }
QCheckBox::indicator {
    width: 14px; height: 14px;
    border: 1px solid #585b70;
    border-radius: 3px;
    background: #313244;
}
QCheckBox::indicator:checked { background: #89b4fa; border-color: #89b4fa; }
QPushButton {
    background-color: #89b4fa;
    color: #1e1e2e;
    border: none;
    border-radius: 5px;
    padding: 5px 12px;
    font-weight: bold;
}
QPushButton:hover { background-color: #b4befe; }
QPushButton:pressed { background-color: #74c7ec; }
QPushButton:disabled { background-color: #45475a; color: #6c7086; }
QScrollBar:vertical {
    background: #1e1e2e; width: 8px; border-radius: 4px;
}
QScrollBar::handle:vertical { background: #45475a; border-radius: 4px; }
QLabel { color: #cdd6f4; }
QStatusBar { color: #a6adc8; background: #181825; }
"""

ERR_STYLE  = "color: #f38ba8; font-weight: bold;"
OK_STYLE   = "color: #a6e3a1;"


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ВПТК — Генератор профиля редуктора")
        self.resize(1080, 700)
        self.setStyleSheet(STYLE)

        # таймер для отложенного обновления при вводе
        self._update_timer = QTimer()
        self._update_timer.setSingleShot(True)
        self._update_timer.setInterval(400)
        self._update_timer.timeout.connect(self._refresh)

        # ── Центральный виджет ────────────────────────────────
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        # ── Панель параметров ─────────────────────────────────
        self.panel = ParamPanel()
        main_layout.addWidget(self.panel)

        # ── Правая колонка: canvas + статус ───────────────────
        right = QVBoxLayout()
        right.setSpacing(6)

        self.canvas = GearCanvas()
        right.addWidget(self.canvas)

        self.lbl_status = QLabel("")
        self.lbl_status.setAlignment(Qt.AlignCenter)
        right.addWidget(self.lbl_status)

        main_layout.addLayout(right, stretch=1)

        # ── Статусная строка ──────────────────────────────────
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Готово")

        # ── Подключение сигналов ──────────────────────────────
        for w in (self.panel.spin_i, self.panel.spin_dsh,
                  self.panel.spin_Rout, self.panel.spin_D,
                  self.panel.spin_res):
            w.valueChanged.connect(self._schedule_refresh)

        for chk in (self.panel.chk_base, self.panel.chk_sep, self.panel.chk_ecc,
                    self.panel.chk_balls, self.panel.chk_out):
            chk.stateChanged.connect(self._schedule_refresh)

        self.panel.btn_save.clicked.connect(self._save_dxf)

        # Первоначальная отрисовка
        self._refresh()

    # ── helpers ───────────────────────────────────────────────

    def _schedule_refresh(self):
        self._update_timer.start()

    def _build_params(self) -> GearParams:
        return GearParams(
            i=self.panel.spin_i.value(),
            dsh=self.panel.spin_dsh.value(),
            Rout=self.panel.spin_Rout.value(),
            D=self.panel.spin_D.value(),
            resolution=self.panel.spin_res.value(),
        )

    def _update_calc_labels(self, p: GearParams):
        self.panel.lbl_e.setText(f"{p.e:.3f} мм")
        self.panel.lbl_zg.setText(str(p.zg))
        self.panel.lbl_zsh.setText(str(p.zsh))
        self.panel.lbl_Rin.setText(f"{p.Rin:.3f} мм")
        self.panel.lbl_rd.setText(f"{p.rd:.3f} мм")
        self.panel.lbl_Rsep_m.setText(f"{p.Rsep_m:.3f} мм")
        self.panel.lbl_hc.setText(f"{p.hc:.3f} мм")

    def _flags(self):
        return dict(
            show_base =self.panel.chk_base.isChecked(),
            show_sep  =self.panel.chk_sep.isChecked(),
            show_ecc  =self.panel.chk_ecc.isChecked(),
            show_balls=self.panel.chk_balls.isChecked(),
            show_out  =self.panel.chk_out.isChecked(),
        )

    # ── slots ─────────────────────────────────────────────────

    def _refresh(self):
        p = self._build_params()
        self._update_calc_labels(p)

        ok, msg = p.validate()
        if not ok:
            self.lbl_status.setText(f"⚠  {msg}")
            self.lbl_status.setStyleSheet(ERR_STYLE)
            self.panel.btn_save.setEnabled(False)
            self.status_bar.showMessage("Ошибка параметров — перерисовка невозможна")
            return

        self.lbl_status.setText("✓  Параметры корректны")
        self.lbl_status.setStyleSheet(OK_STYLE)
        self.panel.btn_save.setEnabled(True)
        self.status_bar.showMessage("Обновление...")

        try:
            self.canvas.draw_gear(p, **self._flags())
            self.status_bar.showMessage(
                f"i={p.i}  zg={p.zg}  zsh={p.zsh}  "
                f"e={p.e:.2f} мм  rd={p.rd:.2f} мм  Rin={p.Rin:.2f} мм"
            )
        except Exception as exc:
            self.status_bar.showMessage(f"Ошибка отрисовки: {exc}")

    def _save_dxf(self):
        p = self._build_params()
        ok, msg = p.validate()
        if not ok:
            self.status_bar.showMessage(f"Нельзя сохранить: {msg}")
            return

        path = self.panel.edit_file.text().strip() or "vptk_output.dxf"
        if not path.lower().endswith(".dxf"):
            path += ".dxf"

        try:
            save_dxf(p, path, **self._flags())
            self.status_bar.showMessage(f"✓  Сохранено: {path}")
        except Exception as exc:
            self.status_bar.showMessage(f"Ошибка сохранения: {exc}")


# ──────────────────────────────────────────────────────────────────────────────

def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
