#!/usr/bin/env python3
"""
I2C Decode Engine — based on libsigrokdecode decoders/i2c/pd.py state machine.
================================================================================
Key patterns adopted from libsigrokdecode:
  - State machine: want_start / collect_address / collect_byte / ACK phases
  - Bit-level tracking: each bit stored as (value, ss_sample, es_sample)
  - 10-bit address detection: 0b11110xxx pattern → 2-byte address phase
  - R/W bit separated from 7-bit address data annotation position
  - Inter-bit width calculation for last bit's end position
  - RESTART detected mid-transfer during byte collection

Polarity convention (matches MATLAB):
  sda_d/scl_d = 1  →  voltage BELOW threshold (line pulled LOW)
  sda_d/scl_d = 0  →  voltage ABOVE threshold (line HIGH / idle)
  I2C logic bit = 1 − sda_d  (invert to get I2C protocol convention)
"""

import numpy as np

DEBOUNCE_US = 1e-6


class I2CDecoder:
    """State-machine based I2C protocol decoder.

    Core states flow: WANT_START → WANT_ADDR_OR_DATA → WANT_ACK → WANT_ADDR_OR_DATA → ...
    Transitions: START opens transfer, RESTART re-opens mid-transfer, STOP closes.

    Usage:
        dec = I2CDecoder()
        txn_list = dec.decode(scl_d, sda_d, scl_rise, t_all, sda_raw)
    """

    # State machine constants
    S_IDLE          = 0   # Waiting for START
    S_GET_ADDR      = 1   # Collecting address byte bits
    S_GET_DATA      = 2   # Collecting data byte bits
    S_GET_ACK       = 3   # Collecting ACK/NACK bit

    def __init__(self):
        self.reset()

    def reset(self):
        self.is_repeat_start = False
        self.pdu_start_idx = None
        self.pdu_bits = 0
        self.data_bits = []      # [(value, ss, es), ...] MSB first
        self.bitwidth = 0        # inter-bit width (in samples)
        self.state = self.S_IDLE
        self.is_write = None
        self.rem_addr_bytes = None
        self.slave_addr_7 = None
        self.slave_addr_10 = None

    # ------------------------------------------------------------------
    # State machine query helpers (matching libsigrokdecode semantics)
    # ------------------------------------------------------------------
    def _wants_start(self):
        return self.state == self.S_IDLE

    def _collects_address(self):
        return self.rem_addr_bytes is None or self.rem_addr_bytes != 0

    def _collects_byte(self):
        return len(self.data_bits) < 8

    def _is_address_phase(self):
        # Address phase: we either haven't determined address length yet (None)
        # or we still have remaining address bytes to collect (>0).
        # Once rem_addr_bytes hits 0, we're in data phase.
        return self.rem_addr_bytes is None or self.rem_addr_bytes != 0

    # ------------------------------------------------------------------
    # Bit accumulation
    # ------------------------------------------------------------------
    def _receive_bit(self, value, ss, es=None):
        """Accumulate a bit. Sets es of previous bit = ss of current bit.
        On 8th bit: estimates last bit's es from bitwidth, returns (byte_val, ss, es, bit9_)."""
        self.pdu_bits += 1
        if self.data_bits:
            self.data_bits[-1] = (self.data_bits[-1][0], self.data_bits[-1][1], ss)
        self.data_bits.append((value, ss, es if es is not None else ss + 1))

        if len(self.data_bits) < 8:
            return None

        # Estimate last bit's end from second-to-last bit's width
        if len(self.data_bits) >= 3:
            self.bitwidth = self.data_bits[-2][2] - self.data_bits[-3][2]
        else:
            self.bitwidth = self.data_bits[-1][1] - self.data_bits[0][1]
        if self.bitwidth <= 0:
            self.bitwidth = 1
        self.data_bits[-1] = (self.data_bits[-1][0], self.data_bits[-1][1],
                               self.data_bits[-1][1] + max(self.bitwidth, 1))

        # Pack MSB-first to byte value
        val = 0
        for i in range(8):
            val = val | (self.data_bits[i][0] << (7 - i))
        ss_byte = self.data_bits[0][1]
        es_byte = self.data_bits[-1][2]
        return val, ss_byte, es_byte

    def _receive_ack(self, value, ss, es=None):
        """Handle the 9th bit (ACK/NACK)."""
        # Ack bit gets its own implied span using bitwidth
        if es is None and self.bitwidth > 0:
            es = ss + self.bitwidth
        return value == 0, ss, es if es is not None else ss + 1  # True = ACK

    def _clear_byte(self):
        self.data_bits.clear()

    # ------------------------------------------------------------------
    # Address byte processing
    # ------------------------------------------------------------------
    def _process_address(self, addr_byte, ss_byte, es_byte):
        """Process first (or second) address byte. Returns (is_sevenbit, addr7, addr10, has_rw_bit, rw_bit_pos info)."""
        has_rw_bit = False
        rw_ss = rw_es = None
        addr_display = addr_byte

        if self.rem_addr_bytes is None:
            # First address byte — check 7-bit vs 10-bit
            if (addr_byte & 0xF8) == 0xF0:  # 0b11110xxx = 10-bit address prefix
                self.rem_addr_bytes = 2
                self.slave_addr_7 = None
                self.slave_addr_10 = (addr_byte & 0x06) << 7
                is_seven = False
            else:
                self.rem_addr_bytes = 1
                self.slave_addr_7 = addr_byte >> 1
                self.slave_addr_10 = None
                self.is_write = (addr_byte & 1) == 0  # 0=Write, 1=Read
                has_rw_bit = True
                # R/W bit occupies the last bit of this byte
                rw_ss = self.data_bits[-1][1]
                rw_es = self.data_bits[-1][2]
                # Shrink address byte annotation to exclude R/W bit
                es_byte = self.data_bits[-2][2]
                addr_display = addr_byte >> 1  # shifted format
                is_seven = True
        else:
            # Second address byte for 10-bit addressing
            if self.slave_addr_10 is not None:
                self.slave_addr_10 |= addr_byte
            is_seven = False

        return {
            'is_sevenbit': self.slave_addr_7 is not None,
            'addr7': self.slave_addr_7,
            'addr10': self.slave_addr_10,
            'is_write': self.is_write,
            'has_rw_bit': has_rw_bit,
            'rw_ss': rw_ss,
            'rw_es': rw_es,
            'addr_display': addr_display,
            'addr_es': es_byte,
        }

    # ------------------------------------------------------------------
    # Main decode entry point
    # ------------------------------------------------------------------
    def decode(self, scl_d, sda_d, scl_rise, t_all, sda_raw):
        """Run state-machine decode on pre-processed digital signals.

        Builds a merged event timeline from START/STOP conditions + SCL rising edges,
        then walks it chronologically with a state machine. Uses self.state directly
        (mirroring libsigrokdecode's instance-based state tracking) to keep
        _is_address_phase() etc. in sync.
        """
        self.reset()
        transactions = []

        START_raw = np.where(np.diff(sda_d) > 0.5)[0] + 1
        STOP_raw  = np.where(np.diff(sda_d) < -0.5)[0] + 1
        START_valid = [si for si in START_raw if si - 1 >= 0 and scl_d[si - 1] == 0]
        STOP_valid  = [si for si in STOP_raw  if si - 1 >= 0 and scl_d[si - 1] == 0]

        merged = []
        for si in START_valid:
            merged.append((si, 'S'))
        for si in STOP_valid:
            merged.append((si, 'P'))
        for si in scl_rise:
            merged.append((si, 'R'))
        merged.sort(key=lambda x: x[0])

        current_scl_rises = []
        is_repeat_start = False

        i = 0
        while i < len(merged):
            idx, etype = merged[i]

            if etype == 'S':
                if self.state == self.S_IDLE:
                    self._begin_transaction(idx)
                    self.state = self.S_GET_ADDR
                    is_repeat_start = False
                    current_scl_rises = []
                elif self.state in (self.S_GET_ADDR, self.S_GET_DATA, self.S_GET_ACK):
                    txn = self._finalize_transaction(t_all, sda_raw, current_scl_rises)
                    if txn:
                        txn['is_restart'] = is_repeat_start
                        transactions.append(txn)
                    self._begin_transaction(idx)
                    self.state = self.S_GET_ADDR
                    is_repeat_start = True
                    current_scl_rises = []
                i += 1

            elif etype == 'P':
                if self.state in (self.S_GET_ADDR, self.S_GET_DATA, self.S_GET_ACK):
                    txn = self._finalize_transaction(t_all, sda_raw, current_scl_rises)
                    if txn:
                        txn['is_restart'] = is_repeat_start
                        transactions.append(txn)
                    self.state = self.S_IDLE
                    is_repeat_start = False
                    current_scl_rises = []
                i += 1

            elif etype == 'R':
                if self.state == self.S_IDLE:
                    i += 1
                    continue

                sda_i2c = int(1 - sda_d[idx])
                current_scl_rises.append(idx)

                if self.state in (self.S_GET_ADDR, self.S_GET_DATA):
                    if len(self.data_bits) < 8:
                        self._receive_bit(sda_i2c, idx)
                        if len(self.data_bits) == 8:
                            self.state = self.S_GET_ACK
                elif self.state == self.S_GET_ACK:
                    is_ack = (sda_i2c == 0)
                    ack_ss = idx
                    ack_es = idx + (max(self.bitwidth, 1) if self.bitwidth > 0 else 1)

                    if len(self.data_bits) == 8:
                        byte_val, ss_byte, es_byte = self._pack_current_byte()
                        addr_info = None
                        if self._is_address_phase():
                            addr_info = self._process_address(byte_val, ss_byte, es_byte)
                        self._store_byte(is_ack, ack_ss, ack_es, addr_info)
                        if self._is_address_phase() and self.rem_addr_bytes is not None:
                            self.rem_addr_bytes -= 1
                        if self._is_address_phase():
                            if self.rem_addr_bytes is None or self.rem_addr_bytes <= 0:
                                self.state = self.S_GET_DATA
                            else:
                                self.state = self.S_GET_ADDR
                        else:
                            self.state = self.S_GET_DATA

                    self._clear_byte()
                i += 1

        if self.state != self.S_IDLE:
            txn = self._finalize_transaction(t_all, sda_raw, current_scl_rises)
            if txn:
                txn['is_restart'] = is_repeat_start
                transactions.append(txn)

        for seg_i, txn in enumerate(transactions):
            txn['seg_i'] = seg_i + 1

        return transactions

    def _begin_transaction(self, start_idx):
        """Initialize state for a new transaction."""
        self.pdu_start_idx = start_idx
        self.pdu_bits = 0
        self.is_write = None
        self.slave_addr_7 = None
        self.slave_addr_10 = None
        self.rem_addr_bytes = None
        self._clear_byte()
        self.bitwidth = 0

    def _pack_current_byte(self):
        val = 0
        for i in range(8):
            val = val | (self.data_bits[i][0] << (7 - i))
        ss_byte = self.data_bits[0][1]
        es_byte = self.data_bits[-1][2]
        return val, ss_byte, es_byte

    def _store_byte(self, is_ack, ack_ss, ack_es, addr_info):
        """Store a completed byte (8 data bits + ACK) into the internal byte buffer."""
        if not hasattr(self, '_byte_buffer'):
            self._byte_buffer = []
            self._scl_rise_buffer = []
            self._current_scl_rises = []
        byte_val, ss_byte, es_byte = self._pack_current_byte()
        byte_entry = {
            'val': byte_val,
            'ss': ss_byte,
            'es': es_byte,
            'is_ack': is_ack,
            'ack_ss': ack_ss,
            'ack_es': ack_es,
            'addr_info': addr_info,
            'data_bits': self.data_bits[:],
        }
        self._byte_buffer.append(byte_entry)

    def _finalize_transaction(self, t_all, sda_raw, scl_rise_indices):
        """Build a transaction dict from accumulated bytes."""
        byte_buf = getattr(self, '_byte_buffer', [])
        if not byte_buf:
            return None

        nbytes = len(byte_buf)
        byte_vals = np.zeros(nbytes, dtype=int)
        byte_acks = np.zeros(nbytes, dtype=int)
        byte_times_us = np.zeros(nbytes)

        total_bits = nbytes * 9
        all_bit_times = np.zeros(total_bits)
        all_sda_at_rise = np.zeros(total_bits)
        all_bit_vals = np.zeros(total_bits, dtype=int)
        bit_labels = []

        addr7 = None; addr10 = None; rw = None

        for b, entry in enumerate(byte_buf):
            byte_vals[b] = entry['val']
            byte_acks[b] = 1 if not entry['is_ack'] else 0
            byte_times_us[b] = t_all[entry['ss']] * 1e6

            # 8 data bits
            for k in range(8):
                idx = b * 9 + k
                val, ss, es = entry['data_bits'][k]
                all_bit_times[idx] = t_all[ss] * 1e6
                all_sda_at_rise[idx] = sda_raw[ss]
                all_bit_vals[idx] = val

                if b == 0 and entry['addr_info'] and entry['addr_info'].get('has_rw_bit'):
                    if k < 7:
                        bit_labels.append(f'A{6-k}')
                    else:
                        bit_labels.append('R/W')
                else:
                    if b == 0:
                        if k < 7:
                            bit_labels.append(f'A{6-k}')
                        else:
                            bit_labels.append('R/W')
                    else:
                        bit_labels.append(f'D{7-k}')

            # ACK bit
            ack_idx = b * 9 + 8
            all_bit_times[ack_idx] = t_all[entry['ack_ss']] * 1e6
            all_sda_at_rise[ack_idx] = sda_raw[entry['ack_ss']]
            all_bit_vals[ack_idx] = int(not entry['is_ack'])
            bit_labels.append('ACK')

            # Address info (first byte only)
            if b == 0 and entry['addr_info']:
                info = entry['addr_info']
                addr7 = info.get('addr7')
                addr10 = info.get('addr10')
                rw = 0 if (info.get('is_write') is True) else 1

        start_idx = self.pdu_start_idx
        if scl_rise_indices:
            stop_idx = scl_rise_indices[-1]
        else:
            stop_idx = start_idx

        # Collect all SCL rise indices that contributed to this txn
        seg_rise = np.array(scl_rise_indices, dtype=int) if scl_rise_indices else np.array([], dtype=int)

        txn = {
            'start_us': t_all[start_idx] * 1e6,
            'stop_us': t_all[stop_idx] * 1e6,
            'start_idx': start_idx,
            'stop_idx': stop_idx,
            'is_restart': self.is_repeat_start,
            'num_bytes': nbytes,
            'bytes': byte_vals,
            'times_us': byte_times_us,
            'acks': byte_acks,
            'bit_times': all_bit_times,
            'sda_at_rise': all_sda_at_rise,
            'bit_vals': all_bit_vals,
            'bit_labels': bit_labels,
            'scl_rise_times': t_all[seg_rise] * 1e6 if len(seg_rise) > 0 else np.array([]),
            'sda_at_scl_rise': sda_raw[seg_rise] if len(seg_rise) > 0 else np.array([]),
            'seg_rise': seg_rise,
        }
        if addr7 is not None:
            txn['addr7'] = addr7
        if addr10 is not None:
            txn['addr10'] = addr10
        if rw is not None:
            txn['rw'] = rw

        self._byte_buffer = []
        return txn


# ------------------------------------------------------------------
# Convenience function: full load + decode pipeline
# ------------------------------------------------------------------
def load_and_decode_i2c(filepath=None):
    """Load CSV, auto-identify channels, decode all I2C bytes.
    Returns dict with all data needed for plotting + txn_list.
    """
    import sys
    if filepath is None:
        from pathlib import Path
        filepath = Path(__file__).resolve().parent / '996D-CWWB-MAIN1.csv'

    print(f'Loading {filepath} ...'); sys.stdout.flush()
    with open(str(filepath), 'r', encoding='gbk', errors='replace') as f:
        data = np.genfromtxt(f, delimiter=',', skip_header=22)
    t_all = data[:, 0]
    ch1 = data[:, 1]
    ch2 = data[:, 2]
    N = len(t_all)
    dt = np.mean(np.diff(t_all))
    t_us = t_all * 1e6
    print(f'  {N} samples @ {1/dt/1e9:.1f} GHz, span={(t_all[-1]-t_all[0])*1e3:.1f} ms')

    # Auto-identify channels
    idle_v = np.max(np.concatenate([ch1[:100], ch2[:100]]))
    th = idle_v * 0.5
    scores = np.zeros(2)
    for flip in range(2):
        x_sda_c, x_scl_c = (ch1, ch2) if flip == 0 else (ch2, ch1)
        xd = (x_sda_c < th).astype(float)
        xc = (x_scl_c < th).astype(float)
        xc_rise_raw = np.where(np.diff(xc) < -0.5)[0] + 1
        if len(xc_rise_raw) == 0:
            scl_cnt = 0
        else:
            clean = [xc_rise_raw[0]]
            for k in range(1, len(xc_rise_raw)):
                if t_all[xc_rise_raw[k]] - t_all[clean[-1]] > DEBOUNCE_US:
                    clean.append(xc_rise_raw[k])
            scl_cnt = len(clean)
        sda_noise = np.sum(np.abs(np.diff(xd)) > 0.5)
        scores[flip] = scl_cnt - sda_noise * 0.05
    if scores[0] >= scores[1]:
        sda_raw, scl_raw = ch1, ch2
    else:
        sda_raw, scl_raw = ch2, ch1

    print(f'  CH1=SDA  CH2=SCL  (idle={idle_v:.2f}V, th={th:.2f}V)')

    # Digital conversion
    sda_d = (sda_raw < th).astype(float)
    scl_d = (scl_raw < th).astype(float)

    # SCL rising edges (debounced)
    scl_rise_raw = np.where(np.diff(scl_d) < -0.5)[0] + 1
    scl_rise = []
    if len(scl_rise_raw) > 0:
        scl_rise.append(scl_rise_raw[0])
        for i in range(1, len(scl_rise_raw)):
            if t_all[scl_rise_raw[i]] - t_all[scl_rise[-1]] > DEBOUNCE_US:
                scl_rise.append(scl_rise_raw[i])
    scl_rise = np.array(scl_rise, dtype=int)

    # SCL falling edges (debounced)
    scl_fall_raw = np.where(np.diff(scl_d) > 0.5)[0] + 1
    scl_fall = []
    if len(scl_fall_raw) > 0:
        scl_fall.append(scl_fall_raw[0])
        for i in range(1, len(scl_fall_raw)):
            if t_all[scl_fall_raw[i]] - t_all[scl_fall[-1]] > DEBOUNCE_US:
                scl_fall.append(scl_fall_raw[i])
    scl_fall = np.array(scl_fall, dtype=int)

    # Decode using state machine
    print(f'Decoding with state-machine I2C decoder ...'); sys.stdout.flush()
    dec = I2CDecoder()
    txn_list = dec.decode(scl_d, sda_d, scl_rise, t_all, sda_raw)

    # Print decode table
    print()
    print('=' * 70)
    print('                    I2C DECODE RESULTS (sigrok state machine)')
    print('-' * 70)
    for txn in txn_list:
        tag = ' [RESTART]' if txn['is_restart'] else ''
        rws = 'R' if txn.get('rw', 0) == 1 else 'W'
        parts = [f"Seg{txn['seg_i']} {txn['start_us']:.1f}->{txn['stop_us']:.1f}us {tag}"]
        for j in range(txn['num_bytes']):
            v = txn['bytes'][j]
            ack = txn['acks'][j]
            ack_s = 'ACK' if ack == 0 else 'NACK'
            if j == 0:
                addr7_str = f'{txn["addr7"]:02X}' if txn.get('addr7') is not None else '??'
                parts.append(f'Addr=0x{v:02X}(7b:0x{addr7_str} {rws} {ack_s})')
            else:
                ch = ''
                if 32 <= v <= 126:
                    ch = f" '{chr(v)}'"
                parts.append(f'0x{v:02X}{ch} {ack_s}')
        print('  ' + ' | '.join(parts))
    print('=' * 70)
    print(f'  Total: {len(txn_list)} segments, {sum(t["num_bytes"] for t in txn_list)} bytes')
    print(); sys.stdout.flush()

    # Build active region for overview
    act = (scl_d == 1) | (sda_d == 1)
    act_idx = np.where(act)[0]
    act_chunks = []
    if len(act_idx) > 0:
        chunk_start = act_idx[0]
        for i in range(1, len(act_idx)):
            if act_idx[i] - act_idx[i-1] > 1000:
                act_chunks.append((chunk_start, act_idx[i-1]))
                chunk_start = act_idx[i]
        act_chunks.append((chunk_start, act_idx[-1]))

    active_x1 = t_all[act_idx[0]] * 1e6 if len(act_idx) > 0 else 0
    active_x2 = t_all[act_idx[-1]] * 1e6 if len(act_idx) > 0 else 0

    # Recompute START/STOP/events for external use
    START_raw = np.where(np.diff(sda_d) > 0.5)[0] + 1
    STOP_raw  = np.where(np.diff(sda_d) < -0.5)[0] + 1
    START_idx = np.array([si for si in START_raw if si - 1 >= 0 and scl_d[si - 1] == 0], dtype=int)
    STOP_idx = np.array([si for si in STOP_raw if si - 1 >= 0 and scl_d[si - 1] == 0], dtype=int)
    events = []
    for si in START_idx:
        events.append([int(si), 1])
    for si in STOP_idx:
        events.append([int(si), 2])
    events.sort(key=lambda x: x[0])

    seg_idx_S = [i for i, txn in enumerate(txn_list) if txn.get('rw') == 0]
    seg_idx_M = [i for i, txn in enumerate(txn_list) if txn.get('rw') == 1]

    return {
        't_us': t_us, 't_all': t_all,
        'scl_raw': scl_raw, 'sda_raw': sda_raw,
        'scl_d': scl_d, 'sda_d': sda_d,
        'scl_rise': scl_rise, 'scl_fall': scl_fall,
        'th': th, 'idle_v': idle_v,
        'txn_list': txn_list,
        'act_chunks': act_chunks,
        'active_x1': active_x1, 'active_x2': active_x2,
        'dt': dt,
        'START_idx': START_idx, 'STOP_idx': STOP_idx,
        'events': events,
        'seg_idx_S': seg_idx_S, 'seg_idx_M': seg_idx_M,
    }


if __name__ == '__main__':
    import sys
    D = load_and_decode_i2c()
    for txn in D['txn_list']:
        print(f"\nSeg {txn['seg_i']}: {txn['start_us']:.1f}->{txn['stop_us']:.1f}us "
              f"RESTART={txn['is_restart']} bytes={txn['num_bytes']}")
        for j in range(txn['num_bytes']):
            print(f"  Byte {j}: 0x{txn['bytes'][j]:02X} ACK={txn['acks'][j]==0}")
            for k in range(9):
                idx = j*9 + k
                print(f"    bit[{k}]: val={txn['bit_vals'][idx]} "
                      f"label={txn['bit_labels'][idx]} "
                      f"t={txn['bit_times'][idx]:.3f}us")
