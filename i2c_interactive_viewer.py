#!/usr/bin/env python3
"""
I2C Interactive Waveform Viewer (pyqtgraph)
============================================
Replaces i2c_full_analysis.m with clearer annotations and oscilloscope-like interactivity.

Features:
  - Mouse wheel zoom, right-drag pan, left-drag box zoom (built into pyqtgraph ViewBox)
  - Two-panel layout: overview (upper) + main decode view (lower), linked X axes
  - START/STOP/RESTART markers with colored labels
  - Per-byte brackets: green for address, purple for data
  - Per-bit value labels on SDA waveform, R/W bit in orange
  - ACK/NACK with green/red highlighting
  - SCL rising-edge sampling point dots on SDA
  - Keyboard: A = auto-range, R = reset, 1/2/3 = preset zoom levels, H = home
  - Decode table printed to console
"""

import sys
import numpy as np
from pathlib import Path

import pyqtgraph as pg
import pyqtgraph.exporters
from PyQt5.QtWidgets import (QApplication, QMainWindow, QSplitter, QWidget,
                              QVBoxLayout, QLabel, QTextEdit, QPushButton,
                              QHBoxLayout, QDockWidget, QFrame)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QKeySequence

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from i2c_decode import load_and_decode_i2c

# ---- Colors ----
CLR_SCL    = QColor(26, 115, 204)     # blue
CLR_SDA    = QColor(230, 51, 64)      # red
CLR_ADDR   = QColor(0, 100, 0)        # dark green
CLR_DATA   = QColor(60, 40, 130)      # purple
CLR_RW     = QColor(255, 77, 0)       # orange
CLR_START  = QColor(0, 140, 0)        # start green
CLR_STOP   = QColor(200, 30, 30)      # stop red
CLR_RESTART= QColor(100, 60, 10)      # restart brown
CLR_ACK_OK = QColor(0, 120, 0)        # ACK green
CLR_NACK   = QColor(220, 0, 0)        # NACK red
CLR_THR    = QColor(100, 100, 100)    # threshold grey
CLR_SAMPLE = QColor(0, 0, 0)          # black sampling dots

# ---- I2C timing thresholds ----
VDD  = 1.8
V30  = VDD * 0.30
V70  = VDD * 0.70
DEBOUNCE_US = 1e-6


def find_crossing(v, tvec, thr, direction, start_idx, search_win):
    si = max(0, start_idx - search_win)
    ei = min(len(v), start_idx + search_win + 1)
    vs = v[si:ei]; ts = tvec[si:ei]
    mask = (vs[:-1] <= thr) & (vs[1:] > thr) if direction == 'rising' else (vs[:-1] >= thr) & (vs[1:] < thr)
    ci = np.where(mask)[0]
    if len(ci) == 0:
        return float('nan'), -1
    ci = ci[0]
    t1, t2 = ts[ci], ts[ci+1]
    v1, v2 = vs[ci], vs[ci+1]
    t_cross = t1 + (thr - v1) * (t2 - t1) / (v2 - v1)
    return t_cross, si + ci



class I2CInteractiveViewer(QMainWindow):
    def __init__(self, D):
        super().__init__()
        self.D = D
        self.setWindowTitle('I2C Interactive Waveform Decoder')
        self.resize(1600, 950)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(4, 4, 4, 4)

        # ---- Control bar ----
        ctrl_bar = QHBoxLayout()
        info_lbl = QLabel(f"File: 996D-CWWB-MAIN1.csv | {len(D['t_us'])/1e6:.2f}M pts | "
                          f"{len(D['txn_list'])} segments, {sum(t['num_bytes'] for t in D['txn_list'])} bytes | "
                          f"th={D['th']:.2f}V | SCL=blue SDA=red | Wheel=zoom R-drag=pan L-drag=boxzoom")
        info_lbl.setStyleSheet("color:#555; font-size:9pt;")
        ctrl_bar.addWidget(info_lbl)
        ctrl_bar.addStretch()
        for label, key in [('Auto', 'A'), ('Reset', 'R'), ('Home', 'H')]:
            btn = QPushButton(f'{label} [{key}]')
            btn.setMaximumWidth(80)
            ctrl_bar.addWidget(btn)
        layout.addLayout(ctrl_bar)

        # ---- Splitter: overview + main ----
        splitter = QSplitter(Qt.Vertical)

        # Overview panel
        self.overview = pg.PlotWidget()
        self.overview.setLabel('left', 'V')
        self.overview.setLabel('bottom', 'Time', units='us')
        self.overview.showGrid(x=True, y=True, alpha=0.3)
        self.overview.getPlotItem().setTitle(
            f'Full Capture ({D["txn_list"][0]["bytes"].size if D["txn_list"] else 0} segments decoded)')
        splitter.addWidget(self.overview)

        # Main decode panel
        self.main_plot = pg.PlotWidget()
        self.main_plot.setLabel('left', 'Voltage', units='V')
        self.main_plot.setLabel('bottom', 'Time', units='us')
        self.main_plot.showGrid(x=True, y=True, alpha=0.3)
        splitter.addWidget(self.main_plot)

        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 5)
        layout.addWidget(splitter)

        # ---- Draw overview ----
        self._draw_overview()

        # ---- Draw main decode view ----
        self._draw_main_decode()

        # ---- Link X axes: overview drives main ----
        self.main_plot.setXLink(self.overview)

        # ---- Set initial view range ----
        pad = 10
        ax1, ax2 = D['active_x1'], D['active_x2']
        self.overview.setXRange(max(0, ax1 - pad), min(D['t_us'][-1], ax2 + pad))
        self.overview.setYRange(-0.3, 2.5)
        self.main_plot.setYRange(-0.4, 2.6)

    def _draw_overview(self):
        D = self.D
        p = self.overview.getPlotItem()

        p.plot(D['t_us'], D['scl_raw'], pen=pg.mkPen(CLR_SCL, width=0.4), name='SCL')
        p.plot(D['t_us'], D['sda_raw'], pen=pg.mkPen(CLR_SDA, width=0.4), name='SDA')
        p.addLine(y=D['th'], pen=pg.mkPen(CLR_THR, width=0.8, style=Qt.DashLine))

        # Active region shading
        y_min = float(np.min(np.minimum(D['scl_raw'], D['sda_raw'])))
        y_max = float(np.max(np.maximum(D['scl_raw'], D['sda_raw'])))
        for c1, c2 in D['act_chunks']:
            x1, x2 = D['t_us'][c1], D['t_us'][c2]
            region = pg.LinearRegionItem(values=(x1, x2), orientation='vertical',
                                         brush=pg.mkBrush(255, 255, 120, 30), movable=False)
            p.addItem(region)

        # Segment markers
        for txn in D['txn_list']:
            st = txn['start_us']
            p.addLine(x=st, pen=pg.mkPen(CLR_START if not txn['is_restart'] else CLR_RESTART,
                                          width=1, style=Qt.DashLine))
            if not txn['is_restart']:
                p.addLine(x=txn['stop_us'], pen=pg.mkPen(CLR_STOP, width=1, style=Qt.DashLine))

    def _draw_main_decode(self):
        D = self.D
        p = self.main_plot.getPlotItem()
        t_us = D['t_us']
        idl = D['idle_v']
        th = D['th']

        p.plot(t_us, D['scl_raw'], pen=pg.mkPen(CLR_SCL, width=0.5), name='SCL')
        p.plot(t_us, D['sda_raw'], pen=pg.mkPen(CLR_SDA, width=0.7), name='SDA')
        p.addLine(y=th, pen=pg.mkPen(CLR_THR, width=1, style=Qt.DashLine))

        y_sda_hi = idl * 1.05
        y_top = idl * 1.18

        for txn in D['txn_list']:
            # ----- START marker -----
            st = txn['start_us']
            if txn['is_restart']:
                p.addLine(x=st, pen=pg.mkPen(CLR_RESTART, width=1.5, style=Qt.DashLine))
                ti = pg.TextItem('Sr\nRESTART', color=CLR_RESTART.darker(120),
                                 anchor=(0.5, 1), fill=pg.mkBrush(255, 245, 230, 220),
                                 border=pg.mkPen(CLR_RESTART, width=1))
                ti.setPos(st, y_top)
            else:
                p.addLine(x=st, pen=pg.mkPen(CLR_START, width=1.5, style=Qt.DashLine))
                ti = pg.TextItem('START', color=CLR_START.darker(120),
                                 anchor=(0.5, 1), fill=pg.mkBrush(230, 255, 230, 220),
                                 border=pg.mkPen(CLR_START, width=1))
                ti.setPos(st, y_top)
            ti.setFont(QFont('Arial', 7, QFont.Bold))
            p.addItem(ti)

            # ----- STOP marker -----
            if not txn['is_restart']:
                sp = txn['stop_us']
                p.addLine(x=sp, pen=pg.mkPen(CLR_STOP, width=1.5, style=Qt.DashLine))
                ti = pg.TextItem('STOP', color=CLR_STOP.darker(120),
                                 anchor=(0.5, 1), fill=pg.mkBrush(255, 230, 230, 220),
                                 border=pg.mkPen(CLR_STOP, width=1))
                ti.setPos(sp, y_top)
                ti.setFont(QFont('Arial', 7, QFont.Bold))
                p.addItem(ti)

            # ----- Per-byte annotations -----
            for b in range(txn['num_bytes']):
                bp = b * 9
                v = txn['bytes'][b]
                ack = txn['acks'][b]
                t_lo = txn['bit_times'][bp]      # MSB
                t_hi = txn['bit_times'][bp + 7]  # LSB
                t_mid = (t_lo + t_hi) / 2
                t_ack = txn['bit_times'][bp + 8]
                y_ack = txn['sda_at_rise'][bp + 8]

                if b == 0:
                    # --- Address byte ---
                    clr = CLR_ADDR
                    bg = pg.mkBrush(230, 250, 230, 210)
                    brd = pg.mkPen(CLR_ADDR, width=1.2)
                    rws = 'W' if txn.get('rw', 0) == 0 else 'R'
                    label_text = f'ADDR  0x{v:02X}\n7b=0x{txn["addr7"]:02X} {rws}'
                else:
                    # --- Data byte ---
                    clr = CLR_DATA
                    bg = pg.mkBrush(235, 235, 255, 210)
                    brd = pg.mkPen(CLR_DATA, width=1.2)
                    ch = ''
                    if 32 <= v <= 126:
                        ch = f" '{chr(v)}'"
                    label_text = f'0x{v:02X}{ch}'

                # Bracket lines
                y_br = y_sda_hi + 0.03 + b * 0.01
                h = 0.06
                p.plot([t_lo, t_hi], [y_br, y_br], pen=pg.mkPen(clr, width=1.5))
                p.plot([t_lo, t_lo], [y_br - h, y_br], pen=pg.mkPen(clr, width=1.5))
                p.plot([t_hi, t_hi], [y_br - h, y_br], pen=pg.mkPen(clr, width=1.5))

                # Byte label
                ti = pg.TextItem(label_text, color=clr.darker(120),
                                 anchor=(0.5, 0), fill=bg, border=brd)
                ti.setPos(t_mid, y_br + 0.02)
                ti.setFont(QFont('Arial', 7, QFont.Bold))
                p.addItem(ti)

                # --- Per-bit value labels on SDA ---
                for k in range(8):
                    idx = bp + k
                    tk = txn['bit_times'][idx]
                    yk = txn['sda_at_rise'][idx]
                    bv = txn['bit_vals'][idx]

                    if k == 7 and b == 0:
                        # R/W bit
                        clr_bit = CLR_RW
                        bg_bit = pg.mkBrush(255, 240, 220, 200)
                        label_bit = rws
                        fnt = QFont('Arial', 7, QFont.Bold)
                    else:
                        clr_bit = clr.darker(120)
                        bg_bit = None
                        label_bit = str(bv)
                        fnt = QFont('Arial', 6)

                    if bv == 1:
                        ty = yk + 0.18
                        ancr = (0.5, 0)
                    else:
                        ty = yk - 0.15
                        ancr = (0.5, 1)

                    ti = pg.TextItem(label_bit, color=clr_bit, anchor=ancr, fill=bg_bit)
                    ti.setPos(tk, ty)
                    ti.setFont(fnt)
                    p.addItem(ti)

                # --- ACK/NACK annotation ---
                if ack == 0:
                    ack_label = 'ACK'
                    ack_color = CLR_ACK_OK
                    ack_bg = pg.mkBrush(230, 255, 230, 200)
                else:
                    ack_label = 'NACK'
                    ack_color = CLR_NACK
                    ack_bg = pg.mkBrush(255, 220, 220, 200)

                if y_ack > th:
                    ty_ack = y_ack + 0.18
                    ancr_ack = (0.5, 0)
                else:
                    ty_ack = y_ack - 0.15
                    ancr_ack = (0.5, 1)

                ti = pg.TextItem(ack_label, color=ack_color, anchor=ancr_ack, fill=ack_bg)
                ti.setPos(t_ack, ty_ack)
                ti.setFont(QFont('Arial', 7, QFont.Bold))
                p.addItem(ti)

        # ---- SCL sampling points on SDA ----
        for txn in D['txn_list']:
            sc_t = txn['scl_rise_times']
            sc_y = txn['sda_at_scl_rise']
            scatter = pg.ScatterPlotItem(x=sc_t, y=sc_y, symbol='o', size=3,
                                          pen=None, brush=CLR_SAMPLE)
            p.addItem(scatter)

        # ---- Build title ----
        title_parts = ['I2C Bus Decode  |  ']
        for txn in D['txn_list']:
            if txn['is_restart']:
                title_parts.append('RESTART  ')
            rws = 'W' if txn.get('rw', 0) == 0 else 'R'
            if txn['num_bytes'] >= 1:
                title_parts.append(f'[0x{txn["bytes"][0]:02X}={rws}] ')
            for j in range(1, txn['num_bytes']):
                title_parts.append(f'0x{txn["bytes"][j]:02X} ')
            title_parts.append('| ')
        p.setTitle(''.join(title_parts))

    def keyPressEvent(self, event):
        key = event.key()
        D = self.D
        if key == Qt.Key_A:
            pad = 10
            self.overview.setXRange(max(0, D['active_x1'] - pad),
                                     min(D['t_us'][-1], D['active_x2'] + pad))
        elif key == Qt.Key_R or key == Qt.Key_H:
            self.overview.autoRange()
        elif key == Qt.Key_1:
            rng = self.overview.getViewBox().viewRange()
            center = sum(rng[0]) / 2
            self.overview.setXRange(center - 50, center + 50)
        elif key == Qt.Key_2:
            rng = self.overview.getViewBox().viewRange()
            center = sum(rng[0]) / 2
            self.overview.setXRange(center - 200, center + 200)
        elif key == Qt.Key_3:
            rng = self.overview.getViewBox().viewRange()
            center = sum(rng[0]) / 2
            self.overview.setXRange(center - 500, center + 500)
        else:
            super().keyPressEvent(event)


def main():
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOptions(antialias=True)

    app = QApplication(sys.argv)

    D = load_and_decode_i2c()

    viewer = I2CInteractiveViewer(D)
    viewer.show()

    print('Keyboard shortcuts:')
    print('  Wheel      = Zoom X')
    print('  Right-drag = Pan')
    print('  Left-drag  = Box zoom (RectMode)')
    print('  A          = Auto-range to active I2C region')
    print('  R / H      = Reset / Home (full view)')
    print('  1          = ±50μs zoom')
    print('  2          = ±200μs zoom')
    print('  3          = ±500μs zoom')
    print()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
