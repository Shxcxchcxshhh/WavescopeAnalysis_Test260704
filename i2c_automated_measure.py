#!/usr/bin/env python3
"""
I2C Automated Measurement Script (Python/pyqtgraph)
=====================================================
Per guide: 1.8V level, 30%(0.54V) / 70%(1.26V) timing reference.
Distinguishes Master->Slave (S) vs Slave->Master (M) per direction.
Outputs measurement values + annotated screenshots using pyqtgraph.
Does NOT judge Pass/Fail; only reports measured values.
"""

import os
import sys
import numpy as np
from pathlib import Path

import pyqtgraph as pg
import pyqtgraph.exporters
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QColor

app = QApplication(sys.argv)

SCRIPT_DIR = Path(__file__).resolve().parent
os.chdir(str(SCRIPT_DIR))

# =============================================================================
# CONFIGURATION
# =============================================================================
FILENAME = '996D-CWWB-MAIN1.csv'
OUT_DIR  = Path('measurements')
VDD      = 1.8
V30      = VDD * 0.30
V70      = VDD * 0.70
DEBOUNCE_US = 1e-6

dir_S     = OUT_DIR / 'Master_to_Slave'
dir_M     = OUT_DIR / 'Slave_to_Master'
dir_other = OUT_DIR / 'Other'
dir_S.mkdir(parents=True, exist_ok=True)
dir_M.mkdir(parents=True, exist_ok=True)
dir_other.mkdir(parents=True, exist_ok=True)

SCL_COLOR = QColor(26, 115, 204)    # [0.10 0.45 0.80]
SDA_COLOR = QColor(230, 51, 64)     # [0.90 0.20 0.25]

print('=' * 56)
print('    I2C Automated Measurement  (1.8V / 30%-70%)')
print('=' * 56)

# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

def find_crossing(v, tvec, thr, direction, start_idx, search_win):
    """Find first threshold crossing in search window around start_idx by linear interpolation.
    v, tvec: 1D numpy arrays. thr: threshold. direction: 'rising'|'falling'
    Returns (t_cross, idx)  or (NaN, NaN) if not found."""
    si = max(0, start_idx - search_win)
    ei = min(len(v), start_idx + search_win + 1)
    vs = v[si:ei]; ts = tvec[si:ei]
    if direction == 'rising':
        cross_mask = (vs[:-1] <= thr) & (vs[1:] > thr)
    else:
        cross_mask = (vs[:-1] >= thr) & (vs[1:] < thr)
    ci = np.where(cross_mask)[0]
    if len(ci) == 0:
        return float('nan'), -1
    ci = ci[0]
    t1, t2 = ts[ci], ts[ci+1]
    v1, v2 = vs[ci], vs[ci+1]
    t_cross = t1 + (thr - v1) * (t2 - t1) / (v2 - v1)
    return t_cross, si + ci

def find_crossing_after(v, tvec, thr, direction, t_start, search_win):
    """Find first threshold crossing strictly AFTER t_start within search_win."""
    idx_start = np.searchsorted(tvec, t_start, side='left')
    idx_start = max(0, min(idx_start, len(v)-1))
    idx_end = min(len(v), idx_start + search_win + 1)
    vs = v[idx_start:idx_end]; ts = tvec[idx_start:idx_end]
    if direction == 'rising':
        cross_mask = (vs[:-1] <= thr) & (vs[1:] > thr)
    else:
        cross_mask = (vs[:-1] >= thr) & (vs[1:] < thr)
    ci = np.where(cross_mask)[0]
    if len(ci) == 0:
        return float('nan'), -1
    ci = ci[0]
    t1, t2 = ts[ci], ts[ci+1]
    v1, v2 = vs[ci], vs[ci+1]
    t_cross = t1 + (thr - v1) * (t2 - t1) / (v2 - v1)
    return t_cross, idx_start + ci

def nan2str(val, fmt='.3f'):
    """Format a float value, return 'N/A' for NaN."""
    if val is None or (isinstance(val, float) and np.isnan(val)):
        return 'N/A'
    if isinstance(val, float) and np.isnan(val):
        return 'N/A'
    return f'{val:{fmt}}'

# =============================================================================
# 1-4. LOAD, IDENTIFY, CONVERT, DECODE  (via i2c_decode state machine)
# =============================================================================
from i2c_decode import load_and_decode_i2c
D = load_and_decode_i2c()

t_all   = D['t_all'];  t_us   = D['t_us'];  dt    = D['dt']
scl_raw = D['scl_raw']; sda_raw = D['sda_raw']
scl_d   = D['scl_d'];  sda_d  = D['sda_d']
scl_rise = D['scl_rise']; scl_fall = D['scl_fall']
START_idx = D['START_idx']; STOP_idx = D['STOP_idx']
events    = D['events']
txn_list  = D['txn_list']
seg_idx_S = D['seg_idx_S']; seg_idx_M = D['seg_idx_M']
N = len(t_all); n_txn = len(txn_list)

print(f'      Segments: {n_txn} (WRITE={len(seg_idx_S)}, READ={len(seg_idx_M)})')
print('\n--- DECODE ---')
for seg_i, txn in enumerate(txn_list):
    if len(txn['bytes']) == 0:
        continue
    tag = ' [RESTART]' if txn['is_restart'] else ''
    dir_str = 'READ' if txn.get('rw', 0) == 1 else 'WRITE'
    addr = txn['bytes'][0]
    parts = [f'Seg{seg_i+1} {txn["start_us"]:.1f}→{txn["stop_us"]:.1f}us {tag} addr=0x{addr:02X}(7b:0x{txn.get("addr7",0):02X},{dir_str})']
    for j in range(1, txn['num_bytes']):
        parts.append(f'0x{txn["bytes"][j]:02X}')
    print('  ' + ' '.join(parts))


# =============================================================================
# 5. PARTITION BUS REGIONS BY DIRECTION
# =============================================================================
print('[5/12] Partitioning bus regions by direction ...')

wr_start = -1; wr_stop = -1; rd_start = -1; rd_stop = -1
if seg_idx_S and seg_idx_M:
    wr_start = txn_list[seg_idx_S[0]]['start_idx']
    wr_stop  = txn_list[seg_idx_S[-1]]['stop_idx']
    rd_start = txn_list[seg_idx_M[0]]['start_idx']
    rd_stop  = txn_list[seg_idx_M[-1]]['stop_idx']
elif seg_idx_S:
    wr_start = txn_list[seg_idx_S[0]]['start_idx']
    wr_stop  = txn_list[seg_idx_S[-1]]['stop_idx']
elif seg_idx_M:
    rd_start = txn_list[seg_idx_M[0]]['start_idx']
    rd_stop  = txn_list[seg_idx_M[-1]]['stop_idx']

def idx2us_str(idx):
    return f'{t_us[idx]:.1f}' if idx >= 0 else 'N/A'

print(f'      WRITE region: {idx2us_str(wr_start)}→{idx2us_str(wr_stop)} us')
print(f'      READ  region: {idx2us_str(rd_start)}→{idx2us_str(rd_stop)} us')

# =============================================================================
# 6. DEFINE SCALARS FOR EDGE SELECTION
# =============================================================================
print('[6/12] Preparing edge selection ...')

scl_r_in_wr = np.array([], dtype=int)
scl_f_in_wr = np.array([], dtype=int)
sda_r_idx   = np.array([], dtype=int)
sda_f_idx   = np.array([], dtype=int)

if wr_start >= 0:
    scl_r_in_wr = scl_rise[(scl_rise >= wr_start) & (scl_rise <= wr_stop)]
    scl_f_in_wr = scl_fall[(scl_fall >= wr_start) & (scl_fall <= wr_stop)]
    sda_trans_idx = np.where(np.diff(sda_d[wr_start:wr_stop+1]) != 0)[0] + wr_start + 1
    sda_r_idx = sda_trans_idx[sda_d[sda_trans_idx] == 0]  # digital 0→1 is rising
    sda_f_idx = sda_trans_idx[sda_d[sda_trans_idx] == 1]  # digital 1→0 is falling

EDGE_WIN = int(round(500e-9 / dt))
print(f'      WRITE edges: SCLrise={len(scl_r_in_wr)} SCLfall={len(scl_f_in_wr)} SDArise={len(sda_r_idx)} SDAfall={len(sda_f_idx)}')

# =============================================================================
# 7. DC LEVEL MEASUREMENTS
# =============================================================================
print('[7/12] DC level measurements ...')

if wr_start >= 0:
    scl_S_hi = np.max(scl_raw[wr_start:wr_stop+1])
    scl_S_lo = np.min(scl_raw[wr_start:wr_stop+1])
    sda_S_hi = np.max(sda_raw[wr_start:wr_stop+1])
    sda_S_lo = np.min(sda_raw[wr_start:wr_stop+1])
else:
    scl_S_hi = scl_S_lo = sda_S_hi = sda_S_lo = float('nan')

if rd_start >= 0:
    sda_M_hi = np.max(sda_raw[rd_start:rd_stop+1])
    sda_M_lo = np.min(sda_raw[rd_start:rd_stop+1])
else:
    sda_M_hi = sda_M_lo = float('nan')

print(f'      SCL(S): HI={scl_S_hi:.3f}V  LO={scl_S_lo:.3f}V')
print(f'      SDA(S): HI={sda_S_hi:.3f}V  LO={sda_S_lo:.3f}V')
print(f'      SDA(M): HI={sda_M_hi:.3f}V  LO={sda_M_lo:.3f}V')

# Initialize cursor-tracking variables
cc_start_sda30 = float('nan'); cc_start_scl30 = float('nan')
cc_rst_scl30 = float('nan');   cc_rst_sda30 = float('nan')
cc_stop_scl30 = float('nan');  cc_stop_sda30 = float('nan')
cc_scl_r70_S = []; cc_scl_f70_S = []
cc_sda_v30_S = []
cc_start_idx = -1; cc_rst_idx = -1
cc_scl_r70_M = []; cc_scl_f70_M = []
cc_sda_v30_M = []

# =============================================================================
# 8. EDGE DETAIL MEASUREMENTS (Signal Quality)
# =============================================================================
print('[8/12] Edge detail measurements ...')

scl_rise_t = float('nan'); scl_fall_t = float('nan')
sda_rise_t = float('nan'); sda_fall_t = float('nan')
overshoot_scl = float('nan'); overshoot_sda = float('nan')
undershoot_scl = float('nan'); undershoot_sda = float('nan')

if wr_start >= 0:
    # SCL rise time
    if len(scl_r_in_wr) > 0:
        sel = scl_r_in_wr[len(scl_r_in_wr)//2]
        t_r30, _ = find_crossing(scl_raw, t_all, V30, 'rising', sel, EDGE_WIN)
        t_r70, _ = find_crossing(scl_raw, t_all, V70, 'rising', sel, EDGE_WIN)
        if not np.isnan(t_r30) and not np.isnan(t_r70):
            scl_rise_t = t_r70 - t_r30
        lo_win = max(0, sel-EDGE_WIN)
        hi_win = min(N, sel+EDGE_WIN+1)
        os_val = np.max(scl_raw[lo_win:hi_win]) - VDD
        overshoot_scl = os_val if os_val > 0.01 else 0.0

    # SCL fall time
    if len(scl_r_in_wr) > 0:
        sel_f = scl_r_in_wr[len(scl_r_in_wr)//2]
        t_sr70_rise, idx_rise70 = find_crossing(scl_raw, t_all, V70, 'rising', sel_f, EDGE_WIN)
        if not np.isnan(t_sr70_rise):
            WIDE_WIN = int(round(20e-6/dt))
            t_f70, _ = find_crossing(scl_raw, t_all, V70, 'falling', sel_f, WIDE_WIN)
            if not np.isnan(t_f70):
                t_f30, _ = find_crossing_after(scl_raw, t_all, V30, 'falling', t_f70, int(round(2e-6/dt)))
                if not np.isnan(t_f30):
                    scl_fall_t = t_f30 - t_f70
        if np.isnan(scl_fall_t):
            undershoot_scl = 0.0
        else:
            lo_win = max(0, sel_f-EDGE_WIN)
            hi_win = min(N, sel_f+EDGE_WIN+1)
            us_val = np.min(scl_raw[lo_win:hi_win])
            undershoot_scl = us_val if us_val < -0.01 else 0.0

    SDA_EDGE_WIN = int(round(2e-6/dt))  # wider window for SDA

    # SDA rise time: chained: find V30 rising, then V70 rising after
    if len(sda_r_idx) > 0:
        sel_sda_r = sda_r_idx[len(sda_r_idx)//2]
        t_sda_r30, _ = find_crossing(sda_raw, t_all, V30, 'rising', sel_sda_r, SDA_EDGE_WIN)
        if not np.isnan(t_sda_r30):
            t_sda_r70, _ = find_crossing_after(sda_raw, t_all, V70, 'rising', t_sda_r30, int(round(500e-9/dt)))
            if not np.isnan(t_sda_r70):
                sda_rise_t = t_sda_r70 - t_sda_r30
        lo_win = max(0, sel_sda_r-EDGE_WIN)
        hi_win = min(N, sel_sda_r+EDGE_WIN+1)
        os_val = np.max(sda_raw[lo_win:hi_win]) - VDD
        overshoot_sda = os_val if os_val > 0.01 else 0.0

    # SDA fall time: chained: find V70 falling, then V30 falling after
    if len(sda_f_idx) > 0:
        sel_sda_f = sda_f_idx[len(sda_f_idx)//2]
        t_sda_f70, _ = find_crossing(sda_raw, t_all, V70, 'falling', sel_sda_f, SDA_EDGE_WIN)
        if not np.isnan(t_sda_f70):
            t_sda_f30, _ = find_crossing_after(sda_raw, t_all, V30, 'falling', t_sda_f70, int(round(500e-9/dt)))
            if not np.isnan(t_sda_f30):
                sda_fall_t = t_sda_f30 - t_sda_f70
        lo_win = max(0, sel_sda_f-EDGE_WIN)
        hi_win = min(N, sel_sda_f+EDGE_WIN+1)
        us_val = np.min(sda_raw[lo_win:hi_win])
        undershoot_sda = us_val if us_val < -0.01 else 0.0

    print(f'      SCL rise: {scl_rise_t*1e9:.1f}ns  overshoot: {overshoot_scl*1e3:.0f}mV')
    print(f'      SCL fall: {scl_fall_t*1e9:.1f}ns  undershoot: {undershoot_scl*1e3:.0f}mV')
    print(f'      SDA rise: {sda_rise_t*1e9:.1f}ns  overshoot: {overshoot_sda*1e3:.0f}mV')
    print(f'      SDA fall: {sda_fall_t*1e9:.1f}ns  undershoot: {undershoot_sda*1e3:.0f}mV')

    # Combined edge detail plot — 2x2 subplots
    win_e = pg.GraphicsLayoutWidget()
    ZOOM_NS = 0.5  # ±500ns zoom

    # --- SCL Rising ---
    p_e1 = win_e.addPlot(row=0, col=0)
    p_e1.setLabel('bottom', 'Time', units='us')
    p_e1.setLabel('left', 'Voltage', units='V')
    if len(scl_r_in_wr) > 0 and not np.isnan(scl_rise_t):
        sel_plot = scl_r_in_wr[len(scl_r_in_wr)//2]
        t_r30_v, _ = find_crossing(scl_raw, t_all, V30, 'rising', sel_plot, EDGE_WIN)
        t_r70_v, _ = find_crossing(scl_raw, t_all, V70, 'rising', sel_plot, EDGE_WIN)
        p_e1.plot(t_us, scl_raw, pen=pg.mkPen(SCL_COLOR, width=1.2))
        p_e1.setXRange(t_us[sel_plot] - ZOOM_NS, t_us[sel_plot] + ZOOM_NS)
        p_e1.addLine(y=V30, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine))
        p_e1.addLine(y=V70, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine))
        if not np.isnan(t_r30_v):
            vline = pg.InfiniteLine(pos=t_r30_v*1e6, angle=90, pen=pg.mkPen('g', width=1, style=pg.QtCore.Qt.DashLine))
            p_e1.addItem(vline)
        if not np.isnan(t_r70_v):
            vline = pg.InfiniteLine(pos=t_r70_v*1e6, angle=90, pen=pg.mkPen('r', width=1, style=pg.QtCore.Qt.DashLine))
            p_e1.addItem(vline)
        if overshoot_scl > 0.01:
            p_e1.addLine(y=VDD + overshoot_scl, pen=pg.mkPen('r', style=pg.QtCore.Qt.DashLine))
    p_e1.setTitle(f'SCL Rising  |  {scl_rise_t*1e9:.1f}ns (30%->70%)')
    p_e1.showGrid(x=True, y=True)

    # --- SCL Falling ---
    p_e2 = win_e.addPlot(row=0, col=1)
    p_e2.setLabel('bottom', 'Time', units='us')
    p_e2.setLabel('left', 'Voltage', units='V')
    if not np.isnan(scl_fall_t):
        sel_fplot = scl_r_in_wr[len(scl_r_in_wr)//2]
        W_WIN = int(round(10e-6/dt))
        t_f70_v, _ = find_crossing(scl_raw, t_all, V70, 'falling', sel_fplot, W_WIN)
        if not np.isnan(t_f70_v):
            t_f30_v, _ = find_crossing_after(scl_raw, t_all, V30, 'falling', t_f70_v, int(round(2e-6/dt)))
        else:
            t_f30_v = float('nan')
        p_e2.plot(t_us, scl_raw, pen=pg.mkPen(SCL_COLOR, width=1.2))
        t_cf = t_f70_v if not np.isnan(t_f70_v) else t_us[sel_fplot]
        p_e2.setXRange(t_cf*1e6 - ZOOM_NS, t_cf*1e6 + ZOOM_NS)
        p_e2.addLine(y=V30, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine))
        p_e2.addLine(y=V70, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine))
        if not np.isnan(t_f70_v):
            vline = pg.InfiniteLine(pos=t_f70_v*1e6, angle=90, pen=pg.mkPen('g', width=1, style=pg.QtCore.Qt.DashLine))
            p_e2.addItem(vline)
        if not np.isnan(t_f30_v):
            vline = pg.InfiniteLine(pos=t_f30_v*1e6, angle=90, pen=pg.mkPen('r', width=1, style=pg.QtCore.Qt.DashLine))
            p_e2.addItem(vline)
        if undershoot_scl < -0.01:
            p_e2.addLine(y=undershoot_scl, pen=pg.mkPen('r', style=pg.QtCore.Qt.DashLine))
    p_e2.setTitle(f'SCL Falling  |  {scl_fall_t*1e9:.1f}ns (70%->30%)')
    p_e2.showGrid(x=True, y=True)

    # --- SDA Rising ---
    p_e3 = win_e.addPlot(row=1, col=0)
    p_e3.setLabel('bottom', 'Time', units='us')
    p_e3.setLabel('left', 'Voltage', units='V')
    if len(sda_r_idx) > 0 and not np.isnan(sda_rise_t):
        sel_sr = sda_r_idx[len(sda_r_idx)//2]
        t_sr30_v, _ = find_crossing(sda_raw, t_all, V30, 'rising', sel_sr, SDA_EDGE_WIN)
        if not np.isnan(t_sr30_v):
            t_sr70_v, _ = find_crossing_after(sda_raw, t_all, V70, 'rising', t_sr30_v, int(round(500e-9/dt)))
        else:
            t_sr70_v = float('nan')
        p_e3.plot(t_us, sda_raw, pen=pg.mkPen(SDA_COLOR, width=1.2))
        p_e3.setXRange(t_us[sel_sr] - ZOOM_NS, t_us[sel_sr] + ZOOM_NS)
        p_e3.addLine(y=V30, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine))
        p_e3.addLine(y=V70, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine))
        if not np.isnan(t_sr30_v):
            vline = pg.InfiniteLine(pos=t_sr30_v*1e6, angle=90, pen=pg.mkPen('g', width=1, style=pg.QtCore.Qt.DashLine))
            p_e3.addItem(vline)
        if not np.isnan(t_sr70_v):
            vline = pg.InfiniteLine(pos=t_sr70_v*1e6, angle=90, pen=pg.mkPen('r', width=1, style=pg.QtCore.Qt.DashLine))
            p_e3.addItem(vline)
        if overshoot_sda > 0.01:
            p_e3.addLine(y=VDD + overshoot_sda, pen=pg.mkPen('r', style=pg.QtCore.Qt.DashLine))
    p_e3.setTitle(f'SDA Rising  |  {sda_rise_t*1e9:.1f}ns (30%->70%)')
    p_e3.showGrid(x=True, y=True)

    # --- SDA Falling ---
    p_e4 = win_e.addPlot(row=1, col=1)
    p_e4.setLabel('bottom', 'Time', units='us')
    p_e4.setLabel('left', 'Voltage', units='V')
    if len(sda_f_idx) > 0 and not np.isnan(sda_fall_t):
        sel_sf = sda_f_idx[len(sda_f_idx)//2]
        t_sf70_v, _ = find_crossing(sda_raw, t_all, V70, 'falling', sel_sf, SDA_EDGE_WIN)
        if not np.isnan(t_sf70_v):
            t_sf30_v, _ = find_crossing_after(sda_raw, t_all, V30, 'falling', t_sf70_v, int(round(500e-9/dt)))
        else:
            t_sf30_v = float('nan')
        p_e4.plot(t_us, sda_raw, pen=pg.mkPen(SDA_COLOR, width=1.2))
        p_e4.setXRange(t_us[sel_sf] - ZOOM_NS, t_us[sel_sf] + ZOOM_NS)
        p_e4.addLine(y=V30, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine))
        p_e4.addLine(y=V70, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine))
        if not np.isnan(t_sf70_v):
            vline = pg.InfiniteLine(pos=t_sf70_v*1e6, angle=90, pen=pg.mkPen('g', width=1, style=pg.QtCore.Qt.DashLine))
            p_e4.addItem(vline)
        if not np.isnan(t_sf30_v):
            vline = pg.InfiniteLine(pos=t_sf30_v*1e6, angle=90, pen=pg.mkPen('r', width=1, style=pg.QtCore.Qt.DashLine))
            p_e4.addItem(vline)
        if undershoot_sda < -0.01:
            p_e4.addLine(y=undershoot_sda, pen=pg.mkPen('r', style=pg.QtCore.Qt.DashLine))
    p_e4.setTitle(f'SDA Falling  |  {sda_fall_t*1e9:.1f}ns (70%->30%)')
    p_e4.showGrid(x=True, y=True)

    exporter = pg.exporters.ImageExporter(win_e.scene())
    exporter.parameters()['width'] = 1800
    exporter.export(str(dir_S / 'I2C_信号质量(S)_信号质量(S).png'))
    print('      -> Saved 信号质量(S) edge detail plot')

# =============================================================================
# 9. AC TIMING — Master->Slave
# =============================================================================
print('[9/12] AC timing measurements (Master->Slave) ...')

results_S = []

# Build decoded transaction SCL edges for S direction
if len(seg_idx_S) > 0:
    scl_rise_S_txn = txn_list[seg_idx_S[0]]['seg_rise']
    scl_rise_times_S = t_all[scl_rise_S_txn]
    scl_fall_S_paired = np.zeros(len(scl_rise_S_txn), dtype=int)
    for k in range(len(scl_rise_S_txn) - 1):
        candidates = scl_fall[(scl_fall > scl_rise_S_txn[k]) & (scl_fall < scl_rise_S_txn[k+1])]
        if len(candidates) > 0:
            scl_fall_S_paired[k] = candidates[0]
else:
    scl_rise_S_txn = np.array([], dtype=int)
    scl_fall_S_paired = np.array([], dtype=int)
    scl_rise_times_S = np.array([])

# 9a. SCL high/low time
scl_hi_T = float('nan'); scl_lo_T = float('nan')
if len(scl_rise_S_txn) >= 3:
    HALF_WIN = int(round(8e-6 / dt))
    hi_vals = []; lo_vals = []
    for k in range(len(scl_rise_S_txn) - 1):
        ri = scl_rise_S_txn[k]
        t_r70, idx_r70 = find_crossing(scl_raw, t_all, V70, 'rising', ri, HALF_WIN)
        if np.isnan(t_r70): continue
        t_f70, _ = find_crossing_after(scl_raw, t_all, V70, 'falling', t_r70, HALF_WIN)
        if np.isnan(t_f70): continue
        hi_vals.append(t_f70 - t_r70)
        t_f30, idx_f30 = find_crossing_after(scl_raw, t_all, V30, 'falling', t_f70, HALF_WIN)
        if np.isnan(t_f30): continue
        t_nr30, _ = find_crossing_after(scl_raw, t_all, V30, 'rising', t_f30, HALF_WIN)
        if not np.isnan(t_nr30):
            lo_vals.append(t_nr30 - t_f30)
    if hi_vals: scl_hi_T = np.mean(hi_vals)
    if lo_vals: scl_lo_T = np.mean(lo_vals)

print(f'      SCL high time:  {scl_hi_T*1e6:.3f} us')
print(f'      SCL low  time:  {scl_lo_T*1e6:.3f} us')

results_S.append(['I2C_时钟高电平时间(S)_时钟高电平时间(S)', f'{scl_hi_T*1e6:.2f}us'])
results_S.append(['I2C_时钟低电平时间(S)_时钟低电平时间(S)', f'{scl_lo_T*1e6:.2f}us'])

# 9b. START setup/hold
t_HD_STA = float('nan'); t_SU_STA = float('nan')
t_sda_start = float('nan'); t_scl_fall30 = float('nan')
t_scl_rise30 = float('nan'); t_sda_rst30 = float('nan')
restart_start = -1

if wr_start >= 0 and len(events) > 0:
    first_start = events[0][0]
    t_start_time = t_all[first_start]
    t_sda_start, _ = find_crossing_after(sda_raw, t_all, V30, 'falling', t_start_time - 1e-7, int(round(5e-6/dt)))
    if not np.isnan(t_sda_start):
        t_scl_fall30, _ = find_crossing(scl_raw, t_all, V30, 'falling', first_start, int(round(10e-6/dt)))
        if not np.isnan(t_scl_fall30):
            t_HD_STA = t_scl_fall30 - t_sda_start

    # t_SU;STA at RESTART
    for txn in txn_list:
        if len(txn['bytes']) > 0 and txn['is_restart']:
            restart_start = txn['start_idx']
            break
    if restart_start >= 0:
        t_rst_time = t_all[restart_start]
        last_scl_candidates = scl_rise[scl_rise < restart_start]
        if len(last_scl_candidates) > 0:
            last_scl_rise_idx = last_scl_candidates[-1]
            t_scl_rise_time = t_all[last_scl_rise_idx]
            t_scl_rise30, _ = find_crossing_after(scl_raw, t_all, V30, 'rising', t_scl_rise_time - 5e-7, int(round(5e-6/dt)))
            t_sda_rst30, _ = find_crossing_after(sda_raw, t_all, V30, 'falling', t_rst_time - 1e-7, int(round(5e-6/dt)))
            if not np.isnan(t_scl_rise30) and not np.isnan(t_sda_rst30):
                t_SU_STA = t_sda_rst30 - t_scl_rise30

    # --- t_HD;STA plot ---
    if not np.isnan(t_HD_STA):
        win_hd = pg.GraphicsLayoutWidget()
        phd = win_hd.addPlot(row=0, col=0)
        phd.setLabel('bottom', 'Time', units='us')
        phd.setLabel('left', 'Voltage', units='V')
        t_hd_ctr = (t_sda_start + t_scl_fall30) / 2 * 1e6
        ZOOM_HD = 5
        phd.plot(t_us, scl_raw, pen=pg.mkPen(SCL_COLOR, width=0.8), name='SCL')
        phd.plot(t_us, sda_raw, pen=pg.mkPen(SDA_COLOR, width=0.8), name='SDA')
        phd.addLine(y=V30, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine, width=1))
        phd.addLine(y=V70, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine, width=1))
        phd.setXRange(t_hd_ctr - ZOOM_HD, t_hd_ctr + ZOOM_HD)
        phd.addLine(x=t_sda_start*1e6, pen=pg.mkPen('r', width=1.5, style=pg.QtCore.Qt.DashDotDotLine))
        phd.addLine(x=t_scl_fall30*1e6, pen=pg.mkPen('b', width=1.5, style=pg.QtCore.Qt.DashDotDotLine))
        phd.addLegend()
        phd.setTitle('START Hold Time (t_HD;STA)')
        phd.showGrid(x=True, y=True)
        exporter = pg.exporters.ImageExporter(win_hd.scene())
        exporter.parameters()['width'] = 1400
        exporter.export(str(dir_S / 'I2C_起始信号保持时间(S)_起始信号保持时间(S).png'))

    # --- t_SU;STA plot (RESTART) ---
    if not np.isnan(t_SU_STA) and restart_start >= 0 and not np.isnan(t_scl_rise30) and not np.isnan(t_sda_rst30):
        win_su = pg.GraphicsLayoutWidget()
        psu = win_su.addPlot(row=0, col=0)
        psu.setLabel('bottom', 'Time', units='us')
        psu.setLabel('left', 'Voltage', units='V')
        t_su_ctr = (t_scl_rise30 + t_sda_rst30) / 2 * 1e6
        ZOOM_SU = 5
        psu.plot(t_us, scl_raw, pen=pg.mkPen(SCL_COLOR, width=0.8), name='SCL')
        psu.plot(t_us, sda_raw, pen=pg.mkPen(SDA_COLOR, width=0.8), name='SDA')
        psu.addLine(y=V30, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine, width=1))
        psu.addLine(y=V70, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine, width=1))
        psu.setXRange(t_su_ctr - ZOOM_SU, t_su_ctr + ZOOM_SU)
        psu.addLine(x=t_scl_rise30*1e6, pen=pg.mkPen('g', width=1.5, style=pg.QtCore.Qt.DashDotDotLine))
        psu.addLine(x=t_sda_rst30*1e6, pen=pg.mkPen('m', width=1.5, style=pg.QtCore.Qt.DashDotDotLine))
        psu.addLegend()
        psu.setTitle('Repeated START Setup Time (t_SU;STA)')
        psu.showGrid(x=True, y=True)
        exporter = pg.exporters.ImageExporter(win_su.scene())
        exporter.parameters()['width'] = 1400
        exporter.export(str(dir_S / 'I2C_起始信号建立时间(S)_起始信号建立时间(S).png'))

    cc_start_sda30 = t_sda_start; cc_start_scl30 = t_scl_fall30
    cc_rst_scl30 = t_scl_rise30; cc_rst_sda30 = t_sda_rst30
    cc_start_idx = first_start; cc_rst_idx = restart_start

print(f'      t_HD;STA:   {t_HD_STA*1e9:.1f} ns')
print(f'      t_SU;STA:   {t_SU_STA*1e9:.1f} ns')

results_S.append(['I2C_起始信号建立时间(S)_起始信号建立时间(S)', f'{t_SU_STA*1e6:.2f}us'])
results_S.append(['I2C_起始信号保持时间(S)_起始信号保持时间(S)', f'{t_HD_STA*1e6:.2f}us'])

# 9c. Data setup/hold (S)
t_SU_DAT = float('nan'); t_HD_DAT = float('nan')
if len(scl_rise_S_txn) >= 2:
    HALF_WIN_DS = int(round(8e-6/dt))
    scl_r70_times = []
    scl_f70_times = []
    for k in range(len(scl_rise_S_txn)):
        tr70, _ = find_crossing(scl_raw, t_all, V70, 'rising', scl_rise_S_txn[k], HALF_WIN_DS)
        scl_r70_times.append(tr70)
        if k < len(scl_fall_S_paired) and scl_fall_S_paired[k] > 0:
            tf70, _ = find_crossing(scl_raw, t_all, V70, 'falling', scl_fall_S_paired[k], HALF_WIN_DS)
            scl_f70_times.append(tf70)
        else:
            scl_f70_times.append(float('nan'))

    t_active_start = t_all[scl_rise_S_txn[0]]
    t_active_stop = t_all[scl_rise_S_txn[-1]]
    active_idx_start = max(0, int(np.searchsorted(t_all, t_active_start - 1e-6, side='left')))
    active_idx_stop = min(N, int(np.searchsorted(t_all, t_active_stop + 1e-6, side='right')))

    sda_tr_all = np.where(np.diff(sda_d) != 0)[0] + 1
    sda_tr_active = sda_tr_all[(sda_tr_all >= active_idx_start) & (sda_tr_all <= active_idx_stop)]
    sda_tr_cross = []
    for j in range(len(sda_tr_active)):
        ti = sda_tr_active[j]
        direction = 'falling' if sda_d[ti] == 1 else 'rising'
        tc, _ = find_crossing(sda_raw, t_all, V30, direction, ti, int(round(2e-6/dt)))
        if not np.isnan(tc):
            sda_tr_cross.append(tc)

    t_SU_DAT_vals = []; t_HD_DAT_vals = []
    SCL_LO_REF = 5.3e-6
    for k in range(len(sda_tr_cross)):
        t_sda30 = sda_tr_cross[k]
        # Find previous SCL V70 fall and next SCL V70 rise
        scl_f70_arr = np.array([v for v in scl_f70_times if not np.isnan(v)])
        scl_r70_arr = np.array([v for v in scl_r70_times if not np.isnan(v)])
        prev_fall_list = scl_f70_arr[scl_f70_arr < t_sda30]
        next_rise_list = scl_r70_arr[scl_r70_arr > t_sda30]
        if len(prev_fall_list) > 0 and len(next_rise_list) > 0:
            prev_fall = prev_fall_list[-1]
            next_rise = next_rise_list[0]
            t_hd_v = t_sda30 - prev_fall
            t_su_v = next_rise - t_sda30
            if 0 < t_hd_v < SCL_LO_REF * 2 and 0 < t_su_v < SCL_LO_REF * 2:
                t_HD_DAT_vals.append(t_hd_v)
                t_SU_DAT_vals.append(t_su_v)
    if t_SU_DAT_vals: t_SU_DAT = np.median(t_SU_DAT_vals)
    if t_HD_DAT_vals: t_HD_DAT = np.median(t_HD_DAT_vals)

    # Data setup/hold plot (S)
    if len(scl_rise_S_txn) >= 4 and len(sda_tr_cross) >= 2:
        sel_k = min(1, len(sda_tr_cross) - 1)  # 0-indexed: 2nd element
        t_sda_sel = sda_tr_cross[sel_k]
        pf_list = scl_f70_arr[scl_f70_arr < t_sda_sel]
        nr_list = scl_r70_arr[scl_r70_arr > t_sda_sel]
        if len(pf_list) > 0 and len(nr_list) > 0:
            t_sf70_p = pf_list[-1]
            t_sr70_p = nr_list[0]
            win_dsh = pg.GraphicsLayoutWidget()
            pdsh = win_dsh.addPlot(row=0, col=0)
            pdsh.setLabel('bottom', 'Time', units='us')
            pdsh.setLabel('left', 'Voltage', units='V')
            t_dsh_ctr = (t_sf70_p + t_sr70_p) / 2 * 1e6
            ZOOM_DSH = 6
            pdsh.plot(t_us, scl_raw, pen=pg.mkPen(SCL_COLOR, width=0.8), name='SCL')
            pdsh.plot(t_us, sda_raw, pen=pg.mkPen(SDA_COLOR, width=0.8), name='SDA')
            pdsh.addLine(y=V30, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine, width=1))
            pdsh.addLine(y=V70, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine, width=1))
            pdsh.setXRange(t_dsh_ctr - ZOOM_DSH, t_dsh_ctr + ZOOM_DSH)
            pdsh.addLine(x=t_sda_sel*1e6, pen=pg.mkPen('r', width=1.5, style=pg.QtCore.Qt.DashDotDotLine))
            pdsh.addLine(x=t_sr70_p*1e6, pen=pg.mkPen('g', width=1.5, style=pg.QtCore.Qt.DashDotDotLine))
            pdsh.addLine(x=t_sf70_p*1e6, pen=pg.mkPen('b', width=1.5, style=pg.QtCore.Qt.DashDotDotLine))
            pdsh.addLegend()
            pdsh.setTitle('Data Setup & Hold Time (Master->Slave)')
            pdsh.showGrid(x=True, y=True)
            exporter = pg.exporters.ImageExporter(win_dsh.scene())
            exporter.parameters()['width'] = 1400
            exporter.export(str(dir_S / 'I2C_数据信号建立时间(S)_数据信号建立时间(S).png'))
            exporter.export(str(dir_S / 'I2C_数据信号保持时间(S)_数据信号保持时间(S).png'))
    cc_scl_r70_S = scl_r70_times; cc_scl_f70_S = scl_f70_times
    cc_sda_v30_S = sda_tr_cross

print(f'      t_SU;DAT(S): {t_SU_DAT*1e9:.1f} ns')
print(f'      t_HD;DAT(S): {t_HD_DAT*1e9:.1f} ns')

results_S.append(['I2C_数据信号建立时间(S)_数据信号建立时间(S)', f'{t_SU_DAT*1e6:.2f}us'])
results_S.append(['I2C_数据信号保持时间(S)_数据信号保持时间(S)', f'{t_HD_DAT*1e6:.2f}us'])

# 9d. STOP setup time
t_SU_STO = float('nan')
t_scl_stop30 = float('nan'); t_sda_stop30 = float('nan'); last_stop = -1
if len(STOP_idx) > 0 and len(scl_rise) > 0:
    last_stop = STOP_idx[-1]
    t_stop_time = t_all[last_stop]
    last_scl_candidates = scl_rise[scl_rise < last_stop]
    if len(last_scl_candidates) > 0:
        last_scl_r = last_scl_candidates[-1]
        t_scl_r_time = t_all[last_scl_r]
        t_scl_stop30, _ = find_crossing_after(scl_raw, t_all, V30, 'rising', t_scl_r_time - 5e-7, int(round(5e-6/dt)))
        t_sda_stop30, _ = find_crossing_after(sda_raw, t_all, V30, 'rising', t_stop_time - 1e-7, int(round(5e-6/dt)))
        if not np.isnan(t_scl_stop30) and not np.isnan(t_sda_stop30):
            t_SU_STO = t_sda_stop30 - t_scl_stop30

    win_sto = pg.GraphicsLayoutWidget()
    psto = win_sto.addPlot(row=0, col=0)
    psto.setLabel('bottom', 'Time', units='us')
    psto.setLabel('left', 'Voltage', units='V')
    t_sto_ctr = (t_scl_stop30 + t_sda_stop30) / 2 * 1e6
    ZOOM_STO = 5
    psto.plot(t_us, scl_raw, pen=pg.mkPen(SCL_COLOR, width=0.8), name='SCL')
    psto.plot(t_us, sda_raw, pen=pg.mkPen(SDA_COLOR, width=0.8), name='SDA')
    psto.addLine(y=V30, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine, width=1))
    psto.addLine(y=V70, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine, width=1))
    psto.setXRange(t_sto_ctr - ZOOM_STO, t_sto_ctr + ZOOM_STO)
    if not np.isnan(t_scl_stop30):
        psto.addLine(x=t_scl_stop30*1e6, pen=pg.mkPen('b', width=1.5, style=pg.QtCore.Qt.DashDotDotLine))
    if not np.isnan(t_sda_stop30):
        psto.addLine(x=t_sda_stop30*1e6, pen=pg.mkPen('r', width=1.5, style=pg.QtCore.Qt.DashDotDotLine))
    psto.addLegend()
    psto.setTitle('STOP Condition Timing')
    psto.showGrid(x=True, y=True)
    exporter = pg.exporters.ImageExporter(win_sto.scene())
    exporter.parameters()['width'] = 1400
    exporter.export(str(dir_S / 'I2C_结束信号建立时间(S)_结束信号建立时间(S).png'))
    cc_stop_scl30 = t_scl_stop30; cc_stop_sda30 = t_sda_stop30

print(f'      t_SU;STO:   {t_SU_STO*1e9:.1f} ns')
results_S.append(['I2C_结束信号建立时间(S)_结束信号建立时间(S)', f'{t_SU_STO*1e6:.2f}us'])

# 9e. Idle time
t_BUF = float('nan')
if len(START_idx) >= 2:
    prev_stop_candidates = STOP_idx[STOP_idx < START_idx[1]]
    if len(prev_stop_candidates) > 0:
        prev_stop = prev_stop_candidates[-1]
        t_stop30_b, _ = find_crossing(sda_raw, t_all, V30, 'rising', prev_stop, 1000)
        t_next_start30_b, _ = find_crossing(sda_raw, t_all, V30, 'falling', START_idx[1], 1000)
        if not np.isnan(t_stop30_b) and not np.isnan(t_next_start30_b):
            t_BUF = t_next_start30_b - t_stop30_b
print(f'      t_BUF:      {t_BUF*1e6:.1f} us')

results_S.append(['I2C_空闲时间(S)_空闲时间(S)', f'{t_BUF*1e6:.2f}us' if not np.isnan(t_BUF) else '0.00us'])

# 9f. Individual edge plots (S) — reusing results from section 8
ZOOM_EDGE = 0.5

def plot_single_edge(t_us_arr, t_all_arr, signal, edge_idx, meas_val, sig_name, edge_type):
    """Create a single-edge zoomed plot and return the GraphicsLayoutWidget."""
    SEARCH_WIN = int(round(2e-6 / (t_all_arr[1] - t_all_arr[0])))
    if edge_type == 'rise':
        t_v30, _ = find_crossing(signal, t_all_arr, V30, 'rising', edge_idx, SEARCH_WIN)
        t_v70, _ = find_crossing(signal, t_all_arr, V70, 'rising', edge_idx, SEARCH_WIN)
    else:
        t_v70, _ = find_crossing(signal, t_all_arr, V70, 'falling', edge_idx, SEARCH_WIN)
        t_v30, _ = find_crossing(signal, t_all_arr, V30, 'falling', edge_idx, SEARCH_WIN)
    win = pg.GraphicsLayoutWidget()
    p = win.addPlot(row=0, col=0)
    p.setLabel('bottom', 'Time', units='us')
    p.setLabel('left', 'Voltage', units='V')
    t_center = t_us_arr[edge_idx]
    if sig_name == 'SDA':
        p.plot(t_us_arr, signal, pen=pg.mkPen(SDA_COLOR, width=0.8))
    else:
        p.plot(t_us_arr, signal, pen=pg.mkPen(SCL_COLOR, width=0.8))
    p.setXRange(t_center - ZOOM_EDGE, t_center + ZOOM_EDGE)
    p.addLine(y=V30, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine))
    p.addLine(y=V70, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine))
    if not np.isnan(t_v30):
        p.addLine(x=t_v30*1e6, pen=pg.mkPen('g', style=pg.QtCore.Qt.DashLine))
    if not np.isnan(t_v70):
        p.addLine(x=t_v70*1e6, pen=pg.mkPen('r', style=pg.QtCore.Qt.DashLine))
    if edge_type == 'rise':
        p.setTitle(f'{sig_name} Rising Edge  |  Rise Time (30%->70%) = {meas_val*1e9:.1f}ns')
    else:
        p.setTitle(f'{sig_name} Falling Edge  |  Fall Time (70%->30%) = {meas_val*1e9:.1f}ns')
    p.showGrid(x=True, y=True)
    return win

def export_edge(win, fname):
    exporter = pg.exporters.ImageExporter(win.scene())
    exporter.parameters()['width'] = 1200
    exporter.export(str(fname))

if not np.isnan(scl_rise_t) and len(scl_rise_S_txn) > 0:
    sel_rise_plot = scl_rise_S_txn[len(scl_rise_S_txn)//2]
    win_cr = plot_single_edge(t_us, t_all, scl_raw, sel_rise_plot, scl_rise_t, 'SCL', 'rise')
    export_edge(win_cr, dir_S / 'I2C_时钟上升时间(S)_时钟上升时间(S).png')

if not np.isnan(scl_fall_t) and len(scl_rise_S_txn) > 0:
    mid_k = len(scl_rise_S_txn)//2
    if mid_k < len(scl_fall_S_paired) and scl_fall_S_paired[mid_k] > 0:
        sel_f_plot = scl_fall_S_paired[mid_k]
    else:
        sel_f_plot = scl_rise_S_txn[mid_k]
    win_cf = plot_single_edge(t_us, t_all, scl_raw, sel_f_plot, scl_fall_t, 'SCL', 'fall')
    export_edge(win_cf, dir_S / 'I2C_时钟下降时间(S)_时钟下降时间(S).png')

if not np.isnan(sda_rise_t) and len(sda_r_idx) > 0:
    sel_sr = sda_r_idx[len(sda_r_idx)//2]
    win_dr = plot_single_edge(t_us, t_all, sda_raw, sel_sr, sda_rise_t, 'SDA', 'rise')
    export_edge(win_dr, dir_S / 'I2C_数据上升时间(S)_数据上升时间(S).png')

if not np.isnan(sda_fall_t) and len(sda_f_idx) > 0:
    sel_sf = sda_f_idx[len(sda_f_idx)//2]
    win_df = plot_single_edge(t_us, t_all, sda_raw, sel_sf, sda_fall_t, 'SDA', 'fall')
    export_edge(win_df, dir_S / 'I2C_数据下降时间(S)_数据下降时间(S).png')

results_S.append(['I2C_时钟上升时间(S)_时钟上升时间(S)', f'{scl_rise_t*1e9:.1f}ns'])
results_S.append(['I2C_时钟下降时间(S)_时钟下降时间(S)', f'{scl_fall_t*1e9:.1f}ns'])
results_S.append(['I2C_数据上升时间(S)_数据上升时间(S)', f'{sda_rise_t*1e9:.1f}ns'])
results_S.append(['I2C_数据下降时间(S)_数据下降时间(S)', f'{sda_fall_t*1e9:.1f}ns'])
results_S.append(['I2C_SCL(S)_高电平电压', f'{scl_S_hi:.3f}V'])
results_S.append(['I2C_SCL(S)_低电平电压', f'{scl_S_lo:.3f}V'])
results_S.append(['I2C_SDA(S)_高电平电压', f'{sda_S_hi:.3f}V'])
results_S.append(['I2C_SDA(S)_低电平电压', f'{sda_S_lo:.3f}V'])

print('      -> Master->Slave measurements complete.')

# =============================================================================
# 10. AC TIMING — Slave->Master (M)
# =============================================================================
print('[10/12] AC timing measurements (Slave->Master) ...')

results_M = []
sda_rise_t_M = float('nan'); sda_fall_t_M = float('nan')
t_SU_DAT_M = float('nan'); t_HD_DAT_M = float('nan')
overshoot_sda_M = float('nan'); undershoot_sda_M = float('nan')

if rd_start >= 0 and len(seg_idx_M) > 0:
    sda_tr_all = np.where(np.diff(sda_d) != 0)[0] + 1
    sda_tr_rd = sda_tr_all[(sda_tr_all >= rd_start) & (sda_tr_all <= rd_stop)]
    EDGE_WIN_M = int(round(500e-9 / dt))

    # SDA rise time (M)
    sda_r_rd = sda_tr_rd[sda_d[sda_tr_rd] == 0]
    if len(sda_r_rd) > 0:
        sel_mr = sda_r_rd[len(sda_r_rd)//2]
        t_mr30, _ = find_crossing(sda_raw, t_all, V30, 'rising', sel_mr, SDA_EDGE_WIN)
        if not np.isnan(t_mr30):
            t_mr70, _ = find_crossing_after(sda_raw, t_all, V70, 'rising', t_mr30, int(round(500e-9/dt)))
            if not np.isnan(t_mr70):
                sda_rise_t_M = t_mr70 - t_mr30
        lo_win = max(0, sel_mr-EDGE_WIN_M); hi_win = min(N, sel_mr+EDGE_WIN_M+1)
        os_val = np.max(sda_raw[lo_win:hi_win]) - VDD
        overshoot_sda_M = os_val if os_val > 0.01 else 0.0

    # SDA fall time (M)
    sda_f_rd = sda_tr_rd[sda_d[sda_tr_rd] == 1]
    sel_mf = -1
    if len(sda_f_rd) > 0:
        sel_mf = sda_f_rd[len(sda_f_rd)//2]
        t_mf70, _ = find_crossing(sda_raw, t_all, V70, 'falling', sel_mf, SDA_EDGE_WIN)
        if not np.isnan(t_mf70):
            t_mf30, _ = find_crossing_after(sda_raw, t_all, V30, 'falling', t_mf70, int(round(500e-9/dt)))
            if not np.isnan(t_mf30):
                sda_fall_t_M = t_mf30 - t_mf70
        lo_win = max(0, sel_mf-EDGE_WIN_M); hi_win = min(N, sel_mf+EDGE_WIN_M+1)
        us_val = np.min(sda_raw[lo_win:hi_win])
        undershoot_sda_M = us_val if us_val < -0.01 else 0.0

    print(f'      SDA(M) rise: {sda_rise_t_M*1e9:.1f}ns  overshoot: {overshoot_sda_M*1e3:.0f}mV')
    print(f'      SDA(M) fall: {sda_fall_t_M*1e9:.1f}ns  undershoot: {undershoot_sda_M*1e3:.0f}mV')

    # Edge detail plot (M) — 1x2 subplots
    win_em = pg.GraphicsLayoutWidget()
    # SDA rising (M)
    p_em1 = win_em.addPlot(row=0, col=0)
    p_em1.setLabel('bottom', 'Time', units='us')
    p_em1.setLabel('left', 'Voltage', units='V')
    if len(sda_r_rd) > 0 and not np.isnan(sda_rise_t_M):
        p_em1.plot(t_us, sda_raw, pen=pg.mkPen(SDA_COLOR, width=1.2))
        p_em1.setXRange(t_us[sel_mr] - ZOOM_NS, t_us[sel_mr] + ZOOM_NS)
        p_em1.addLine(y=V30, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine))
        p_em1.addLine(y=V70, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine))
        if not np.isnan(t_mr30):
            p_em1.addLine(x=t_mr30*1e6, pen=pg.mkPen('g', style=pg.QtCore.Qt.DashLine))
        if not np.isnan(t_mr70):
            p_em1.addLine(x=t_mr70*1e6, pen=pg.mkPen('r', style=pg.QtCore.Qt.DashLine))
        if overshoot_sda_M > 0.01:
            p_em1.addLine(y=VDD + overshoot_sda_M, pen=pg.mkPen('r', style=pg.QtCore.Qt.DashLine))
    p_em1.setTitle(f'SDA(M) Rising  |  {sda_rise_t_M*1e9:.1f}ns (30%->70%)')
    p_em1.showGrid(x=True, y=True)

    # SDA falling (M)
    p_em2 = win_em.addPlot(row=0, col=1)
    p_em2.setLabel('bottom', 'Time', units='us')
    p_em2.setLabel('left', 'Voltage', units='V')
    if len(sda_f_rd) > 0 and not np.isnan(sda_fall_t_M):
        p_em2.plot(t_us, sda_raw, pen=pg.mkPen(SDA_COLOR, width=1.2))
        p_em2.setXRange(t_us[sel_mf] - ZOOM_NS, t_us[sel_mf] + ZOOM_NS)
        p_em2.addLine(y=V30, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine))
        p_em2.addLine(y=V70, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine))
        if not np.isnan(t_mf70):
            p_em2.addLine(x=t_mf70*1e6, pen=pg.mkPen('g', style=pg.QtCore.Qt.DashLine))
        if not np.isnan(t_mf30):
            p_em2.addLine(x=t_mf30*1e6, pen=pg.mkPen('r', style=pg.QtCore.Qt.DashLine))
        if undershoot_sda_M < -0.01:
            p_em2.addLine(y=undershoot_sda_M, pen=pg.mkPen('r', style=pg.QtCore.Qt.DashLine))
    p_em2.setTitle(f'SDA(M) Falling  |  {sda_fall_t_M*1e9:.1f}ns (70%->30%)')
    p_em2.showGrid(x=True, y=True)

    exporter = pg.exporters.ImageExporter(win_em.scene())
    exporter.parameters()['width'] = 1600
    exporter.export(str(dir_M / 'I2C_数据上升时间(M)_数据上升时间(M).png'))
    exporter.export(str(dir_M / 'I2C_数据下降时间(M)_数据下降时间(M).png'))

    # Data setup/hold (M)
    scl_rise_M_txn = txn_list[seg_idx_M[0]]['seg_rise']
    if len(scl_rise_M_txn) >= 2:
        HALF_WIN_DS_M = int(round(8e-6/dt))
        scl_fall_M_paired = np.zeros(len(scl_rise_M_txn), dtype=int)
        for k in range(len(scl_rise_M_txn) - 1):
            candidates = scl_fall[(scl_fall > scl_rise_M_txn[k]) & (scl_fall < scl_rise_M_txn[k+1])]
            if len(candidates) > 0:
                scl_fall_M_paired[k] = candidates[0]

        scl_r70_times_m = []; scl_f70_times_m = []
        for k in range(len(scl_rise_M_txn)):
            tr70, _ = find_crossing(scl_raw, t_all, V70, 'rising', scl_rise_M_txn[k], HALF_WIN_DS_M)
            scl_r70_times_m.append(tr70)
            if k < len(scl_fall_M_paired) and scl_fall_M_paired[k] > 0:
                tf70, _ = find_crossing(scl_raw, t_all, V70, 'falling', scl_fall_M_paired[k], HALF_WIN_DS_M)
                scl_f70_times_m.append(tf70)
            else:
                scl_f70_times_m.append(float('nan'))

        t_active_m1 = t_all[scl_rise_M_txn[0]]
        t_active_m2 = t_all[scl_rise_M_txn[-1]]
        am1 = max(0, int(np.searchsorted(t_all, t_active_m1 - 1e-6, side='left')))
        am2 = min(N, int(np.searchsorted(t_all, t_active_m2 + 1e-6, side='right')))
        sda_tr_all = np.where(np.diff(sda_d) != 0)[0] + 1
        sda_tr_active_m = sda_tr_all[(sda_tr_all >= am1) & (sda_tr_all <= am2)]
        sda_tr_cross_M = []
        for j in range(len(sda_tr_active_m)):
            ti = sda_tr_active_m[j]
            direction = 'falling' if sda_d[ti] == 1 else 'rising'
            tc, _ = find_crossing(sda_raw, t_all, V30, direction, ti, int(round(2e-6/dt)))
            if not np.isnan(tc):
                sda_tr_cross_M.append(tc)

        t_SU_DAT_M_vals = []; t_HD_DAT_M_vals = []
        scl_f70_arr_m = np.array([v for v in scl_f70_times_m if not np.isnan(v)])
        scl_r70_arr_m = np.array([v for v in scl_r70_times_m if not np.isnan(v)])
        for k in range(len(sda_tr_cross_M)):
            t_sda30 = sda_tr_cross_M[k]
            pf_list = scl_f70_arr_m[scl_f70_arr_m < t_sda30]
            nr_list = scl_r70_arr_m[scl_r70_arr_m > t_sda30]
            if len(pf_list) > 0 and len(nr_list) > 0:
                prev_fall = pf_list[-1]
                next_rise = nr_list[0]
                t_hd_v = t_sda30 - prev_fall
                t_su_v = next_rise - t_sda30
                if 0 < t_hd_v < SCL_LO_REF * 2 and 0 < t_su_v < SCL_LO_REF * 2:
                    t_HD_DAT_M_vals.append(t_hd_v)
                    t_SU_DAT_M_vals.append(t_su_v)
        if t_SU_DAT_M_vals: t_SU_DAT_M = np.median(t_SU_DAT_M_vals)
        if t_HD_DAT_M_vals: t_HD_DAT_M = np.median(t_HD_DAT_M_vals)

        print(f'      t_SU;DAT(M): {t_SU_DAT_M*1e9:.1f} ns')
        print(f'      t_HD;DAT(M): {t_HD_DAT_M*1e9:.1f} ns')

        # Plot data setup/hold (M)
        if len(scl_rise_M_txn) >= 4 and len(sda_tr_cross_M) >= 2:
            sel_k = min(1, len(sda_tr_cross_M) - 1)
            t_sda_sel = sda_tr_cross_M[sel_k]
            pf_list = scl_f70_arr_m[scl_f70_arr_m < t_sda_sel]
            nr_list = scl_r70_arr_m[scl_r70_arr_m > t_sda_sel]
            if len(pf_list) > 0 and len(nr_list) > 0:
                t_sf70_p = pf_list[-1]
                t_sr70_p = nr_list[0]
                win_dshm = pg.GraphicsLayoutWidget()
                pdshm = win_dshm.addPlot(row=0, col=0)
                pdshm.setLabel('bottom', 'Time', units='us')
                pdshm.setLabel('left', 'Voltage', units='V')
                t_dshm_ctr = (t_sf70_p + t_sr70_p) / 2 * 1e6
                ZOOM_DSH_M = 6
                pdshm.plot(t_us, scl_raw, pen=pg.mkPen(SCL_COLOR, width=0.8), name='SCL')
                pdshm.plot(t_us, sda_raw, pen=pg.mkPen(SDA_COLOR, width=0.8), name='SDA')
                pdshm.addLine(y=V30, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine, width=1))
                pdshm.addLine(y=V70, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine, width=1))
                pdshm.setXRange(t_dshm_ctr - ZOOM_DSH_M, t_dshm_ctr + ZOOM_DSH_M)
                pdshm.addLine(x=t_sda_sel*1e6, pen=pg.mkPen('r', width=1.5, style=pg.QtCore.Qt.DashDotDotLine))
                pdshm.addLine(x=t_sr70_p*1e6, pen=pg.mkPen('g', width=1.5, style=pg.QtCore.Qt.DashDotDotLine))
                pdshm.addLine(x=t_sf70_p*1e6, pen=pg.mkPen('b', width=1.5, style=pg.QtCore.Qt.DashDotDotLine))
                pdshm.addLegend()
                pdshm.setTitle('Data Setup & Hold Time (Slave->Master)')
                pdshm.showGrid(x=True, y=True)
                exporter = pg.exporters.ImageExporter(win_dshm.scene())
                exporter.parameters()['width'] = 1400
                exporter.export(str(dir_M / 'I2C_数据建立时间(M)_数据建立时间(M).png'))
                exporter.export(str(dir_M / 'I2C_数据保持时间(M)_数据保持时间(M).png'))
        cc_scl_r70_M = scl_r70_times_m; cc_scl_f70_M = scl_f70_times_m
        cc_sda_v30_M = sda_tr_cross_M

    results_M.append(['I2C_数据上升时间(M)_数据上升时间(M)', f'{sda_rise_t_M*1e9:.1f}ns'])
    results_M.append(['I2C_数据下降时间(M)_数据下降时间(M)', f'{sda_fall_t_M*1e9:.1f}ns'])
    results_M.append(['I2C_数据建立时间(M)_数据建立时间(M)', f'{t_SU_DAT_M*1e6:.2f}us'])
    results_M.append(['I2C_数据保持时间(M)_数据保持时间(M)', f'{t_HD_DAT_M*1e6:.2f}us'])
    results_M.append(['I2C_SDA(M)_高电平电压', f'{sda_M_hi:.3f}V'])
    results_M.append(['I2C_SDA(M)_低电平电压', f'{sda_M_lo:.3f}V'])

print('      -> Slave->Master measurements complete.')

# =============================================================================
# 11. DEADLOCK DETECTION
# =============================================================================
print('[11/12] Deadlock detection ...')
deadlock_found = False

sda_low_runs = []
in_low = False
low_start = 0
for i in range(N):
    if sda_d[i] == 1 and not in_low:
        in_low = True; low_start = i
    elif sda_d[i] == 0 and in_low:
        in_low = False
        dur = t_all[i] - t_all[low_start]
        if dur > 1e-6:
            sda_low_runs.append([low_start, i, dur])
if in_low:
    sda_low_runs.append([low_start, N-1, t_all[N-1]-t_all[low_start]])

for r in sda_low_runs:
    scl_pulses = scl_rise[(scl_rise >= r[0]) & (scl_rise <= r[1])]
    if len(scl_pulses) >= 9:
        deadlock_found = True
        print(f'      DEADLOCK: {len(scl_pulses)} SCL pulses during SDA low ({t_us[r[0]]:.1f}->{t_us[r[1]]:.1f} us)')
        win_dl = pg.GraphicsLayoutWidget()
        pdl = win_dl.addPlot(row=0, col=0)
        pdl.setLabel('bottom', 'Time', units='us')
        pdl.setLabel('left', 'Voltage', units='V')
        pdl.plot(t_us, scl_raw, pen=pg.mkPen(SCL_COLOR, width=0.8), name='SCL')
        pdl.plot(t_us, sda_raw, pen=pg.mkPen(SDA_COLOR, width=0.8), name='SDA')
        pdl.setXRange(t_us[r[0]] - 2, t_us[r[1]] + 2)
        pdl.addLine(y=V30, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine))
        pdl.addLine(y=V70, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine))
        pdl.addLegend()
        pdl.setTitle(f'I2C Deadlock: {len(scl_pulses)} SCL pulses during SDA stuck LOW')
        pdl.showGrid(x=True, y=True)
        exporter = pg.exporters.ImageExporter(win_dl.scene())
        exporter.parameters()['width'] = 1400
        exporter.export(str(dir_other / 'I2C_死锁_9CLK波形.png'))
        break

if not deadlock_found:
    print('      No deadlock condition found.')

# =============================================================================
# 12. RESULTS SUMMARY & CSV EXPORT
# =============================================================================
print('[12/12] Exporting results ...')

print()
print('=' * 60)
print('                MEASUREMENT RESULTS SUMMARY')
print('+' + '-'*58 + '+')
print(f'  Level: 1.8V   Thresholds: 30%={V30:.2f}V  70%={V70:.2f}V')

if results_S:
    print('+ --------------------------------------------------------+')
    print('  Master->Slave (S)')
    print('+ --------------------------------------------------------+')
    for k in range(len(results_S)):
        print(f'  {results_S[k][0]:50s}  {results_S[k][1]:>10s}')

if results_M:
    print('+ --------------------------------------------------------+')
    print('  Slave->Master (M)')
    print('+ --------------------------------------------------------+')
    for k in range(len(results_M)):
        print(f'  {results_M[k][0]:50s}  {results_M[k][1]:>10s}')

print('+ --------------------------------------------------------+')
print(f'  Deadlock: {"YES" if deadlock_found else "No":>10s}')
if not np.isnan(scl_hi_T) and not np.isnan(scl_lo_T) and (scl_hi_T + scl_lo_T) > 0:
    freq_khz = 1/(scl_hi_T + scl_lo_T)/1e3
    print(f'  SCL freq: {freq_khz:.1f} kHz (std mode)')
else:
    print('  SCL freq: N/A')
print('=' * 60)

# CSV export (UTF-8 BOM via encoding='utf-8-sig')
all_results = results_S + results_M
csv_path = OUT_DIR / 'measurements.csv'
with open(csv_path, 'w', encoding='utf-8-sig') as f:
    f.write('指标名称,实测值\n')
    for row in all_results:
        f.write(f'{row[0]},{row[1]}\n')
print(f'\n-> CSV: {csv_path}')

# ---- Comprehensive overview plot ----
print('\n-> Generating overview plot with all measurement cursors ...')
WR_T0 = t_us[wr_start] if wr_start >= 0 else 0
WR_T1 = t_us[wr_stop]  if wr_stop >= 0  else 0
RD_T0 = t_us[rd_start] if rd_start >= 0 else 0
RD_T1 = t_us[rd_stop]  if rd_stop >= 0  else 0

win_ov = pg.GraphicsLayoutWidget()
# SCL subplot
ax1 = win_ov.addPlot(row=0, col=0)
ax1.setLabel('bottom', 'Time', units='us')
ax1.setLabel('left', 'SCL', units='V')
ax1.plot(t_us, scl_raw, pen=pg.mkPen(SCL_COLOR, width=0.4))
ax1.addLine(y=V30, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine, width=1.5))
ax1.addLine(y=V70, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine, width=1.5))
if not np.isnan(scl_S_hi):
    ax1.addLine(y=scl_S_hi, pen=pg.mkPen('r', style=pg.QtCore.Qt.DashLine))
    ax1.addLine(y=scl_S_lo, pen=pg.mkPen('b', style=pg.QtCore.Qt.DashLine))

# WRITE/READ shading
if wr_start >= 0:
    lr = pg.LinearRegionItem(values=(WR_T0, WR_T1), orientation='vertical',
                             brush=pg.mkBrush(174, 217, 255, 45), movable=False)
    ax1.addItem(lr)
if rd_start >= 0:
    lr = pg.LinearRegionItem(values=(RD_T0, RD_T1), orientation='vertical',
                             brush=pg.mkBrush(255, 191, 191, 45), movable=False)
    ax1.addItem(lr)

# SCL V70 crossing markers (S)
scl_r70_S_arr = np.array([v for v in cc_scl_r70_S if not np.isnan(v)])
if len(scl_r70_S_arr) > 0:
    scatter = pg.ScatterPlotItem(x=scl_r70_S_arr*1e6, y=np.full(len(scl_r70_S_arr), V70),
                                  symbol='t', size=5, pen='g', brush='g')
    ax1.addItem(scatter)
scl_f70_S_arr = np.array([v for v in cc_scl_f70_S if not np.isnan(v)])
if len(scl_f70_S_arr) > 0:
    scatter = pg.ScatterPlotItem(x=scl_f70_S_arr*1e6, y=np.full(len(scl_f70_S_arr), V70),
                                  symbol='t1', size=5, pen='b', brush='b')
    ax1.addItem(scatter)
# SCL V70 crossing markers (M)
scl_r70_M_arr = np.array([v for v in cc_scl_r70_M if not np.isnan(v)])
if len(scl_r70_M_arr) > 0:
    scatter = pg.ScatterPlotItem(x=scl_r70_M_arr*1e6, y=np.full(len(scl_r70_M_arr), V70),
                                  symbol='t', size=4, pen='r', brush='r')
    ax1.addItem(scatter)
scl_f70_M_arr = np.array([v for v in cc_scl_f70_M if not np.isnan(v)])
if len(scl_f70_M_arr) > 0:
    scatter = pg.ScatterPlotItem(x=scl_f70_M_arr*1e6, y=np.full(len(scl_f70_M_arr), V70),
                                  symbol='t1', size=4, pen='r', brush='r')
    ax1.addItem(scatter)

# Event markers
if cc_start_idx >= 0:
    ax1.addLine(x=t_us[cc_start_idx], pen=pg.mkPen('m', width=1.5, style=pg.QtCore.Qt.DashLine))
if cc_rst_idx >= 0:
    ax1.addLine(x=t_us[cc_rst_idx], pen=pg.mkPen('m', width=1.5, style=pg.QtCore.Qt.DashLine))
if last_stop >= 0:
    ax1.addLine(x=t_us[last_stop], pen=pg.mkPen('m', width=1.5, style=pg.QtCore.Qt.DashLine))

# V30 cursor lines
if not np.isnan(cc_start_scl30):
    ax1.addLine(x=cc_start_scl30*1e6, pen=pg.mkPen('c', style=pg.QtCore.Qt.DashDotDotLine))
if not np.isnan(cc_rst_scl30):
    ax1.addLine(x=cc_rst_scl30*1e6, pen=pg.mkPen('c', style=pg.QtCore.Qt.DashDotDotLine))
if not np.isnan(cc_stop_scl30):
    ax1.addLine(x=cc_stop_scl30*1e6, pen=pg.mkPen('c', style=pg.QtCore.Qt.DashDotDotLine))

freq_label = f'{1/(scl_hi_T + scl_lo_T)/1e3:.1f}kHz' if (not np.isnan(scl_hi_T) and not np.isnan(scl_lo_T)) else 'N/A'
ax1.setTitle(f'SCL  |  Freq={freq_label}  HI={nan2str(scl_S_hi)}V/LO={nan2str(scl_S_lo)}V')
ax1.showGrid(x=True, y=True)
ax1.setYRange(-0.3, 2.5)

# SDA subplot
ax2 = win_ov.addPlot(row=1, col=0)
ax2.setLabel('bottom', 'Time', units='us')
ax2.setLabel('left', 'SDA', units='V')
ax2.plot(t_us, sda_raw, pen=pg.mkPen(SDA_COLOR, width=0.4))
ax2.addLine(y=V30, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine, width=1.5))
ax2.addLine(y=V70, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine, width=1.5))
if not np.isnan(sda_S_hi):
    ax2.addLine(y=sda_S_hi, pen=pg.mkPen('r', style=pg.QtCore.Qt.DashLine))
    ax2.addLine(y=sda_S_lo, pen=pg.mkPen('b', style=pg.QtCore.Qt.DashLine))

if wr_start >= 0:
    lr = pg.LinearRegionItem(values=(WR_T0, WR_T1), orientation='vertical',
                             brush=pg.mkBrush(174, 217, 255, 45), movable=False)
    ax2.addItem(lr)
if rd_start >= 0:
    lr = pg.LinearRegionItem(values=(RD_T0, RD_T1), orientation='vertical',
                             brush=pg.mkBrush(255, 191, 191, 45), movable=False)
    ax2.addItem(lr)

# SDA V30 transition markers
y_sda30 = V30 + 0.06
if len(cc_sda_v30_S) > 0:
    sda_v30_arr = np.array([v for v in cc_sda_v30_S if not np.isnan(v)])
    if len(sda_v30_arr) > 0:
        scatter = pg.ScatterPlotItem(x=sda_v30_arr*1e6, y=np.full(len(sda_v30_arr), y_sda30),
                                      symbol='s', size=5, pen='g', brush='g')
        ax2.addItem(scatter)
if len(cc_sda_v30_M) > 0:
    sda_v30_m_arr = np.array([v for v in cc_sda_v30_M if not np.isnan(v)])
    if len(sda_v30_m_arr) > 0:
        scatter = pg.ScatterPlotItem(x=sda_v30_m_arr*1e6, y=np.full(len(sda_v30_m_arr), V30),
                                      symbol='s', size=4, pen='r', brush='r')
        ax2.addItem(scatter)

if cc_start_idx >= 0:
    ax2.addLine(x=t_us[cc_start_idx], pen=pg.mkPen('m', width=1.5, style=pg.QtCore.Qt.DashLine))
if cc_rst_idx >= 0:
    ax2.addLine(x=t_us[cc_rst_idx], pen=pg.mkPen('m', width=1.5, style=pg.QtCore.Qt.DashLine))
if last_stop >= 0:
    ax2.addLine(x=t_us[last_stop], pen=pg.mkPen('m', width=1.5, style=pg.QtCore.Qt.DashLine))
if not np.isnan(cc_start_sda30):
    ax2.addLine(x=cc_start_sda30*1e6, pen=pg.mkPen('r', style=pg.QtCore.Qt.DashDotDotLine))
if not np.isnan(cc_rst_sda30):
    ax2.addLine(x=cc_rst_sda30*1e6, pen=pg.mkPen('r', style=pg.QtCore.Qt.DashDotDotLine))
if not np.isnan(cc_stop_sda30):
    ax2.addLine(x=cc_stop_sda30*1e6, pen=pg.mkPen('r', style=pg.QtCore.Qt.DashDotDotLine))

# Transaction labels
for txn in txn_list:
    if len(txn['bytes']) > 0:
        rw_label = 'R' if txn.get('rw', 0) == 1 else 'W'
        lbl = f'0x{txn["bytes"][0]:02X}({rw_label})=0x{txn["bytes"][1]:02X}' if txn['num_bytes'] >= 2 else f'0x{txn["bytes"][0]:02X}'
        prefix = 'RESTART ' if txn['is_restart'] else 'START '
        txt_item = pg.TextItem(text=prefix + lbl, color=(0, 77, 179) if not txn['is_restart'] else (179, 26, 26),
                                anchor=(1, 0.5), angle=90)
        txt_item.setPos(t_us[txn['start_idx']], V30 - 0.15)
        ax2.addItem(txt_item)

ax2.setTitle(f'SDA  |  S:HI={nan2str(sda_S_hi)}V/LO={nan2str(sda_S_lo)}V  M:HI={nan2str(sda_M_hi)}V/LO={nan2str(sda_M_lo)}V')
ax2.setYRange(-0.3, 2.5)
ax2.showGrid(x=True, y=True)

# Link X axes
ax2.setXLink(ax1)

exporter = pg.exporters.ImageExporter(win_ov.scene())
exporter.parameters()['width'] = 1800
exporter.export(str(dir_other / 'I2C_DC_电平汇总.png'))
print('      -> Saved overview plot: I2C_DC_电平汇总.png')

# List generated files
print('\nGenerated files:')
for png_file in sorted(OUT_DIR.rglob('*.png')):
    print(f'  {png_file}')
print()
print('=== DONE ===')
