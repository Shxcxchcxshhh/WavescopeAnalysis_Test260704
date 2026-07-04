%% I2C Waveform Analysis & Decoder
% ZDS3024 Plus Oscilloscope, 2GHz sampling, 2-channel I2C capture
% Auto-detects SCL/SDA, decodes all bytes, annotates directly on waveform.
%
% Polarity convention:
%   sda_d/cl_d = 1  →  voltage below threshold (line pulled LOW)
%   sda_d/cl_d = 0  →  voltage above threshold (line HIGH / idle)
%   I2C logic bit  = 1 − sda_d  (invert to get I2C convention)
% =========================================================================
clear; close all; clc;

%% ========================================================================
% 1. LOAD DATA
% ========================================================================
fprintf('=== I2C Waveform Decoder ===\n\n');
fprintf('[1] Loading 996D-CWWB-MAIN1.csv ...\n');

filename = '996D-CWWB-MAIN1.csv';
opts = detectImportOptions(filename, 'NumHeaderLines', 22);
raw = readmatrix(filename, opts);
t_all = raw(:,1);                          % seconds
ch1   = raw(:,2);                          % oscilloscope CH1 voltage
ch2   = raw(:,3);                          % oscilloscope CH2 voltage

N  = length(t_all);
dt = mean(diff(t_all));
fprintf('    %d samples @ %.1f GHz,  span = %.1f ms\n', N, 1/dt/1e9, (t_all(end)-t_all(1))*1e3);

%% ========================================================================
% 2. AUTO-IDENTIFY CHANNELS (SCL vs SDA)
% ========================================================================
fprintf('[2] Auto-identifying SCL / SDA ...\n');

% Threshold = 50% of idle (both channels idle at ~1.81V)
idle_v = max([ch1(1:100); ch2(1:100)]);
th = idle_v * 0.5;

% For both assignments, compute: # SCL edges  &  # SDA noise transitions
% Correct assignment: more clock edges, fewer data glitches
for flip = 0:1
    if flip == 0
        x_sda = ch1;  x_scl = ch2;  label = 'CH1=SDA  CH2=SCL';
    else
        x_sda = ch2;  x_scl = ch1;  label = 'CH1=SCL  CH2=SDA';
    end

    xd = double(x_sda < th);
    xc = double(x_scl < th);

    % Debounced SCL rising edges  (scl_d 1→0 = voltage LOW→HIGH)
    xc_rise_raw = find(diff(xc) < -0.5) + 1;
    if isempty(xc_rise_raw)
        scl_cnt = 0;
    else
        clean = xc_rise_raw(1);
        for k = 2:length(xc_rise_raw)
            if t_all(xc_rise_raw(k)) - t_all(clean(end)) > 1e-6
                clean = [clean; xc_rise_raw(k)];
            end
        end
        scl_cnt = length(clean);
    end

    sda_noise = length(find(abs(diff(xd)) > 0.5));

    fprintf('    %s  →  SCL edges=%d, SDA transitions=%d\n', label, scl_cnt, sda_noise);
    scores(flip+1) = scl_cnt - sda_noise * 0.05;
end

if scores(1) >= scores(2)
    sda_raw = ch1;  scl_raw = ch2;  ch_label = 'CH1=SDA  CH2=SCL';
else
    sda_raw = ch2;  scl_raw = ch1;  ch_label = 'CH1=SCL  CH2=SDA';
end
fprintf('    Selected: %s\n\n', ch_label);

%% ========================================================================
% 3. DIGITAL CONVERSION & EDGE DETECTION
% ========================================================================
fprintf('[3] Converting to digital & finding edges ...\n');

% sda_d/scl_d:  1 = voltage LOW (pulled down),  0 = voltage HIGH (idle)
sda_d = double(sda_raw < th);
scl_d = double(scl_raw < th);

% ---- SCL rising edge:  scl_d goes 1→0  (voltage goes LOW→HIGH)  ----
scl_rise_raw = find(diff(scl_d) < -0.5) + 1;
if ~isempty(scl_rise_raw)
    scl_rise = scl_rise_raw(1);
    for i = 2:length(scl_rise_raw)
        if t_all(scl_rise_raw(i)) - t_all(scl_rise(end)) > 1e-6
            scl_rise = [scl_rise; scl_rise_raw(i)];
        end
    end
else
    scl_rise = [];
end

% ---- SCL falling edge:  scl_d goes 0→1  (voltage goes HIGH→LOW)  ----
scl_fall_raw = find(diff(scl_d) > 0.5) + 1;
if ~isempty(scl_fall_raw)
    scl_fall = scl_fall_raw(1);
    for i = 2:length(scl_fall_raw)
        if t_all(scl_fall_raw(i)) - t_all(scl_fall(end)) > 1e-6
            scl_fall = [scl_fall; scl_fall_raw(i)];
        end
    end
else
    scl_fall = [];
end

fprintf('    SCL edges:  %d rising, %d falling  (debounced)\n', ...
        length(scl_rise), length(scl_fall));

% ---- START condition: SDA voltage HIGH→LOW  while SCL HIGH ----
%   = sda_d goes 0→1  while scl_d == 0
st_raw = find(diff(sda_d) > 0.5) + 1;
% ---- STOP condition:  SDA voltage LOW→HIGH  while SCL HIGH ----
%   = sda_d goes 1→0  while scl_d == 0
sp_raw = find(diff(sda_d) < -0.5) + 1;

START_idx = [];
for i = 1:length(st_raw)
    si = st_raw(i);
    if si > 3 && scl_d(si-1) == 0
        START_idx = [START_idx; si];
    end
end

STOP_idx = [];
for i = 1:length(sp_raw)
    si = sp_raw(i);
    if si > 3 && scl_d(si-1) == 0
        STOP_idx = [STOP_idx; si];
    end
end

fprintf('    START conditions: %d   STOP conditions: %d\n', ...
        length(START_idx), length(STOP_idx));

%% ========================================================================
% 4. TRANSACTION PAIRING & I2C DECODE
% ========================================================================
fprintf('[4] Pairing transactions & decoding I2C ...\n');

% Pair START → STOP (non-overlapping)
% Build event timeline: [idx, type] where type=1 for START, 2 for STOP
events = [];
for i = 1:length(START_idx)
    events = [events; START_idx(i), 1];
end
for i = 1:length(STOP_idx)
    events = [events; STOP_idx(i), 2];
end
events = sortrows(events);

% Walk events: a START opens a segment, a STOP closes it.
% A START while a segment is already open = REPEATED START (RESTART).
current_start = 0;
boundaries = [];  % [start_idx, end_idx, is_restart]

for e = 1:size(events, 1)
    idx = events(e, 1);
    typ = events(e, 2);
    if typ == 1  % START
        if current_start > 0
            % RESTART: close current segment, open new one
            boundaries(end+1, :) = [current_start, idx, 0];  % 0 → ends at RESTART
        end
        current_start = idx;
    else  % STOP
        if current_start > 0
            boundaries(end+1, :) = [current_start, idx, 0];
            current_start = 0;
        end
    end
end

% Sort by start time
boundaries = sortrows(boundaries);

% ---- Decode each segment ----
txn_data = struct('start_us', {}, 'stop_us', {}, 'is_restart', {}, ...
                  'num_bytes', {}, 'bytes', {}, 'times_us', {}, ...
                  'addr7', {}, 'rw', {}, 'acks', {}, ...
                  'bit_times', {}, 'bit_vals', {}, 'bit_labels', {}, ...
                  'scl_rise_times', {}, 'sda_at_rise', {});

for seg = 1:size(boundaries, 1)
    si = boundaries(seg, 1);
    ei = boundaries(seg, 2);
    % A segment is a RESTART if its START is not the first START event
    is_restart = (si ~= events(1,1));

    seg_rise = scl_rise(scl_rise >= si & scl_rise <= ei);
    if length(seg_rise) < 9, continue; end

    i2c_bits = 1 - sda_d(seg_rise);
    nbytes   = floor(length(i2c_bits) / 9);

    byte_vals  = [];
    byte_times = [];    % time of MSB (bit7) of each byte
    byte_acks  = [];
    bit_times  = [];    % time of every individual bit
    bit_vals   = [];    % value of every bit
    bit_labels = {};    % role label per bit

    for b = 1:nbytes
        bp  = (b-1)*9 + 1;
        bb  = i2c_bits(bp:bp+7);
        ack = i2c_bits(bp+8);
        val = 0;
        for k = 1:8
            val = val + bb(k) * 2^(8-k);
        end
        byte_vals  = [byte_vals;  val];
        byte_times = [byte_times; t_all(seg_rise(bp))*1e6];
        byte_acks  = [byte_acks;  ack];

        % Per-bit info
        for k = 1:8
            bit_times = [bit_times; t_all(seg_rise(bp + k - 1))*1e6];
            bit_vals  = [bit_vals;  bb(k)];
            if b == 1
                if k <= 7
                    bit_labels{end+1} = sprintf('A%d', 7-k);  % A6..A0
                else
                    bit_labels{end+1} = 'R/W';
                end
            else
                bit_labels{end+1} = sprintf('D%d', 8-k);  % D7..D0
            end
        end
        % ACK bit
        bit_times = [bit_times; t_all(seg_rise(bp + 8))*1e6];
        bit_vals  = [bit_vals;  ack];
        bit_labels{end+1} = 'ACK';
    end

    txn_data(seg).start_us      = t_all(si)*1e6;
    txn_data(seg).stop_us       = t_all(ei)*1e6;
    txn_data(seg).is_restart    = is_restart;
    txn_data(seg).num_bytes     = nbytes;
    txn_data(seg).bytes         = byte_vals;
    txn_data(seg).times_us      = byte_times;
    txn_data(seg).acks          = byte_acks;
    txn_data(seg).bit_times     = bit_times;
    txn_data(seg).bit_vals      = bit_vals;
    txn_data(seg).bit_labels    = bit_labels;
    txn_data(seg).scl_rise_times = t_all(seg_rise)*1e6;
    txn_data(seg).sda_at_rise   = sda_raw(seg_rise);

    if nbytes >= 1
        txn_data(seg).addr7 = bitshift(byte_vals(1), -1);
        txn_data(seg).rw    = bitand(byte_vals(1), 1);
    end
end

% ---- Print decode table ----
fprintf('\n');
fprintf(' ╔══════════════════════════════════════════════════════════╗\n');
fprintf(' ║              I2C DECODE RESULTS                          ║\n');
fprintf(' ╠══════╦══════════╦══════════╦════════╦════════════════════╣\n');
fprintf(' ║ Seg  ║ START us ║  STOP us ║  Bytes ║ Detail             ║\n');
fprintf(' ╠══════╬══════════╬══════════╬════════╬════════════════════╣\n');
for seg = 1:length(txn_data)
    if isempty(txn_data(seg).bytes), continue; end
    d = txn_data(seg);
    tag = ''; if d.is_restart, tag = ' [RESTART]'; end
    fprintf(' ║  %2d  ║ %8.2f ║ %8.2f ║   %2d   ║', ...
            seg, d.start_us, d.stop_us, d.num_bytes);
    for j = 1:d.num_bytes
        if j == 1
            rws = 'W'; if d.rw == 1, rws = 'R'; end
            acs = 'ACK'; if d.acks(1) == 1, acs = 'NACK'; end
            fprintf(' Addr=0x%02X(7b:0x%02X %s %s)', d.bytes(1), d.addr7, rws, acs);
        else
            fprintf(' 0x%02X', d.bytes(j));
        end
    end
    fprintf('%s\n', tag);
end
fprintf(' ╚══════╩══════════╩══════════╩════════╩════════════════════╝\n');
fprintf('\n');

for seg = 1:length(txn_data)
    d = txn_data(seg);
    if isempty(d.bytes), continue; end
    fprintf('Segment %d  [%.2f → %.2f us]  %d bytes\n', seg, d.start_us, d.stop_us, d.num_bytes);
    for j = 1:d.num_bytes
        v = d.bytes(j);
        ack = d.acks(j);
        if j == 1
            fprintf('  [Addr]  0x%02X  (%s)  7-bit=0x%02X  %s  %s\n', ...
                    v, dec2bin(v,8), d.addr7, ...
                    iif(d.rw==0,'WRITE','READ '), ...
                    iif(ack==0,'ACK ✓','NACK ✗'));
        else
            ch = ''; if v >= 32 && v <= 126, ch = ['  ASCII=''' char(v) '''']; end
            fprintf('  [Data]  0x%02X  (%s)  %3d  %s%s\n', ...
                    v, dec2bin(v,8), v, iif(ack==0,'ACK ✓','NACK ✗'), ch);
        end
    end
    fprintf('\n');
end

%% ========================================================================
% 5. WAVEFORM PLOT WITH ANNOTATIONS
% ========================================================================
fprintf('[5] Plotting waveforms ...\n');

t_us = t_all * 1e6;

% ---- Figure: full-screen for easy pan/zoom ----
screen = get(0, 'ScreenSize');
fig = figure('Name', 'I2C Waveform Analysis', ...
             'Position', [20, 40, screen(3)-40, screen(4)-120], ...
             'NumberTitle', 'off');

%% ── Panel A: Full capture overview (15% height) ──
ax_full = subplot(8, 1, 1);
cla(ax_full);
plot(t_us, scl_raw, 'Color', [0.10 0.45 0.80], 'LineWidth', 0.3); hold on;
plot(t_us, sda_raw, 'Color', [0.90 0.20 0.25], 'LineWidth', 0.3);
yline(th, '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 0.8);

% Highlight active regions
act = (scl_d == 1) | (sda_d == 1);
if any(act)
    act_idx = find(act);
    act_chunks = [act_idx(1)];
    act_ends   = [];
    for i = 2:length(act_idx)
        if act_idx(i) - act_idx(i-1) > 1000
            act_chunks(end+1) = act_idx(i);
            act_ends(end+1)    = act_idx(i-1);
        end
    end
    act_ends(end+1) = act_idx(end);
    for i = 1:length(act_chunks)
        x1 = t_us(act_chunks(i));  x2 = t_us(act_ends(i));
        y1 = min(scl_raw);  y2 = max(scl_raw);
        patch([x1 x2 x2 x1], [y1 y1 y2 y2], [1 1 0.5], ...
              'FaceAlpha', 0.12, 'EdgeColor', 'none');
    end
end

xlabel('Time (\mus)'); ylabel('V');
title(sprintf('Full Capture  (%s)   %.0f GHz,  %.0f ms span', ...
      ch_label, 1/dt/1e9, (t_all(end)-t_all(1))*1e3));
legend({'SCL', 'SDA', 'Threshold'}, 'Location', 'northeast', 'FontSize', 8);
grid on;

%% ── Panel B: Main zoomed decode view (85% height) ──
ax_main = subplot(8, 1, 2:8);
cla(ax_main);

% === Plot analog signals ===
h_scl = plot(t_us, scl_raw, 'Color', [0.10 0.45 0.80], 'LineWidth', 0.5); hold on;
h_sda = plot(t_us, sda_raw, 'Color', [0.90 0.20 0.25], 'LineWidth', 0.7);
yline(th, '--', 'Color', [0.35 0.35 0.35], 'LineWidth', 1);

% === Auto-zoom to active I2C region ===
if any(act)
    pad = 10;  % 10 us padding
    x1 = max(t_us(1),   t_us(act_idx(1))  - pad);
    x2 = min(t_us(end), t_us(act_idx(end)) + pad);
    xlim([x1, x2]);
end

% === Annotate START / STOP conditions on SDA trace ===
y_top  = max(scl_raw) * 1.15;
y_sda_hi = max(sda_raw) * 1.08;
y_sda_lo = min(sda_raw) - 0.12;
y_box_hi = max(scl_raw) * 0.92;

for seg = 1:length(txn_data)
    d = txn_data(seg);
    if isempty(d.bytes), continue; end

    % START marker (green dashed line + label)
    st = d.start_us;
    xline(st, '--', 'Color', [0 0.55 0], 'LineWidth', 1);
    if d.is_restart
        text(st, y_top, 'Sr\n(RESTART)', 'FontSize', 7, 'FontWeight', 'bold', ...
             'Color', [0 0.45 0], 'HorizontalAlignment', 'center', ...
             'VerticalAlignment', 'top', 'BackgroundColor', [0.95 1 0.95 0.9], ...
             'EdgeColor', [0 0.5 0], 'Margin', 2);
    else
        text(st, y_top, 'START', 'FontSize', 7, 'FontWeight', 'bold', ...
             'Color', [0 0.45 0], 'HorizontalAlignment', 'center', ...
             'VerticalAlignment', 'top', 'BackgroundColor', [0.95 1 0.95 0.9], ...
             'EdgeColor', [0 0.5 0], 'Margin', 2);
    end

    % STOP marker (red dashed line + label)
    if ~d.is_restart
        sp = d.stop_us;
        xline(sp, '--', 'Color', [0.8 0.15 0.15], 'LineWidth', 1);
        text(sp, y_top, 'STOP', 'FontSize', 7, 'FontWeight', 'bold', ...
             'Color', [0.8 0.15 0.15], 'HorizontalAlignment', 'center', ...
             'VerticalAlignment', 'top', 'BackgroundColor', [1 0.95 0.95 0.9], ...
             'EdgeColor', [0.8 0 0], 'Margin', 2);
    end

    % ---- Per-byte annotations on SDA waveform ----
    for b = 1:d.num_bytes
        bp    = (b-1)*9 + 1;
        v     = d.bytes(b);
        ack   = d.acks(b);
        t_lo  = d.bit_times(bp);        % MSB time
        t_hi  = d.bit_times(bp + 7);    % LSB time
        t_ack = d.bit_times(bp + 8);    % ACK bit time
        y_lo  = d.sda_at_rise(bp);
        y_hi  = d.sda_at_rise(bp + 7);
        y_ack = d.sda_at_rise(bp + 8);

        if b == 1
            % ===== ADDRESS BYTE =====
            % Bracket spanning the address byte data bits (bits 0-7)
            y_bracket = y_sda_hi;
            line([t_lo t_hi], [y_bracket y_bracket], 'Color', [0 0.4 0], 'LineWidth', 1.5);
            line([t_lo t_lo], [y_bracket-0.05 y_bracket], 'Color', [0 0.4 0], 'LineWidth', 1.5);
            line([t_hi t_hi], [y_bracket-0.05 y_bracket], 'Color', [0 0.4 0], 'LineWidth', 1.5);

            % Address byte label at bracket center
            rw_str = 'W'; if d.rw == 1, rw_str = 'R'; end
            text((t_lo + t_hi)/2, y_bracket + 0.06, ...
                 {sprintf('ADDR  0x%02X', v), sprintf('7b=0x%02X %s', d.addr7, rw_str)}, ...
                 'FontSize', 8, 'FontWeight', 'bold', 'Color', [0 0.3 0], ...
                 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
                 'BackgroundColor', [0.9 1 0.9 0.85], ...
                 'EdgeColor', [0 0.5 0], 'Margin', 3);

            % ---- Mark each address bit on SDA ----
            for k = 1:8
                tk = d.bit_times(bp + k - 1);
                yk = d.sda_at_rise(bp + k - 1);
                lb = d.bit_labels(bp + k - 1);
                bv = d.bit_vals(bp + k - 1);
                tx = tk;
                ty = yk;
                if bv == 1
                    ty = yk + 0.14;
                    va = 'bottom';
                else
                    ty = yk - 0.12;
                    va = 'top';
                end
                if k == 8
                    % R/W bit: special highlight
                    text(tx, ty, rw_str, 'FontSize', 6, 'FontWeight', 'bold', ...
                         'Color', [1 0.3 0], 'BackgroundColor', [1 0.95 0.8 0.8], ...
                         'HorizontalAlignment', 'center', 'VerticalAlignment', va);
                else
                    text(tx, ty, num2str(bv), 'FontSize', 6, ...
                         'Color', [0.2 0.5 0.2], ...
                         'HorizontalAlignment', 'center', 'VerticalAlignment', va);
                end
            end

            % ---- ACK bit annotation ----
            acks = 'ACK'; ackc = [0 0.5 0];
            if ack == 1, acks = 'NACK'; ackc = [1 0 0]; end
            ty_ack = y_ack;
            if y_ack > th, ty_ack = y_ack + 0.14;
            else,          ty_ack = y_ack - 0.12; end
            text(t_ack, ty_ack, acks, 'FontSize', 7, 'FontWeight', 'bold', ...
                 'Color', ackc, 'HorizontalAlignment', 'center', ...
                 'VerticalAlignment', iif(y_ack > th, 'bottom', 'top'), ...
                 'BackgroundColor', iif(ack==0, [0.9 1 0.9 0.7], [1 0.9 0.9 0.7]));

        else
            % ===== DATA BYTE =====
            y_bracket = y_sda_hi;
            line([t_lo t_hi], [y_bracket y_bracket], 'Color', [0.3 0.2 0.6], 'LineWidth', 1.5);
            line([t_lo t_lo], [y_bracket-0.04 y_bracket], 'Color', [0.3 0.2 0.6], 'LineWidth', 1.5);
            line([t_hi t_hi], [y_bracket-0.04 y_bracket], 'Color', [0.3 0.2 0.6], 'LineWidth', 1.5);

            % Data byte label
            ch = '';
            if v >= 32 && v <= 126, ch = sprintf(' ''%c''', char(v)); end
            text((t_lo + t_hi)/2, y_bracket + 0.06, ...
                 sprintf('0x%02X%s', v, ch), ...
                 'FontSize', 8, 'FontWeight', 'bold', 'Color', [0.25 0 0.4], ...
                 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
                 'BackgroundColor', [0.9 0.9 1 0.85], ...
                 'EdgeColor', [0.3 0 0.6], 'Margin', 3);

            % Per-bit labels on SDA
            for k = 1:8
                tk = d.bit_times(bp + k - 1);
                yk = d.sda_at_rise(bp + k - 1);
                bv = d.bit_vals(bp + k - 1);
                if bv == 1
                    ty = yk + 0.14; va = 'bottom';
                else
                    ty = yk - 0.12; va = 'top';
                end
                text(tk, ty, num2str(bv), 'FontSize', 6, ...
                     'Color', [0.3 0.2 0.5], ...
                     'HorizontalAlignment', 'center', 'VerticalAlignment', va);
            end

            % ACK
            acks = 'ACK'; ackc = [0 0.5 0];
            if ack == 1, acks = 'NACK'; ackc = [1 0 0]; end
            ty_ack = y_ack;
            if y_ack > th, ty_ack = y_ack + 0.14;
            else,          ty_ack = y_ack - 0.12; end
            text(t_ack, ty_ack, acks, 'FontSize', 7, 'FontWeight', 'bold', ...
                 'Color', ackc, 'HorizontalAlignment', 'center', ...
                 'VerticalAlignment', iif(y_ack > th, 'bottom', 'top'), ...
                 'BackgroundColor', iif(ack==0, [0.9 1 0.9 0.7], [1 0.9 0.9 0.7]));
        end
    end
end

% === SCL rising edge sampling points ===
plot(t_us(scl_rise), sda_raw(scl_rise), 'k.', 'MarkerSize', 3);

% === Labels ===
xlabel('Time (\mus)', 'FontSize', 11);
ylabel('Voltage (V)', 'FontSize', 11);

% Build title string from decoded data
title_str = sprintf('I2C Bus  %s  |  Threshold = %.2f V  |  ', ch_label, th);
for seg = 1:length(txn_data)
    d = txn_data(seg);
    if isempty(d.bytes), continue; end
    if d.is_restart
        title_str = [title_str, 'RESTART→ '];
    end
    for j = 1:d.num_bytes
        if j == 1
            rws = 'W'; if d.rw == 1, rws = 'R'; end
            title_str = [title_str, sprintf('[ADDR:7b-0x%02X %s] ', d.addr7, rws)];
        else
            title_str = [title_str, sprintf('0x%02X ', d.bytes(j))];
        end
    end
    title_str = [title_str, '| '];
end
title(title_str, 'FontSize', 10, 'FontWeight', 'bold');

legend({'SCL (clock)', 'SDA (data)', 'Threshold (0.91V)'}, ...
       'Location', 'northeast', 'FontSize', 8);
grid on;

% Enable interactive zoom/pan tools
zoom on;
pan on;

%% ========================================================================
% 6. SUMMARY
% ========================================================================
fprintf('\n=== SUMMARY ===\n');
total_bytes = sum([txn_data.num_bytes]);
fprintf('  File:          %s\n', filename);
fprintf('  Sampling:      %.0f GHz  |  %.0f ms capture\n', 1/dt/1e9, (t_all(end)-t_all(1))*1e3);
fprintf('  Channels:      %s\n', ch_label);
fprintf('  Threshold:     %.2f V  (idle = %.2f V)\n', th, idle_v);
fprintf('  Segments:      %d\n', length(txn_data));
fprintf('  Total bytes:   %d\n', total_bytes);

for seg = 1:length(txn_data)
    d = txn_data(seg);
    if isempty(d.bytes), continue; end
    if d.is_restart
        fprintf('  Seg %d: RESTART @ %.1f us → ', seg, d.start_us);
    else
        fprintf('  Seg %d: START  @ %.1f us → STOP @ %.1f us  |  ', ...
                seg, d.start_us, d.stop_us);
    end
    fprintf('addr=0x%02X(7b:0x%02X,%s)', d.bytes(1), d.addr7, iif(d.rw==0,'W','R'));
    for j = 2:d.num_bytes
        fprintf('  0x%02X', d.bytes(j));
    end
    fprintf('\n');
end

fprintf('\nDone.  Use mouse wheel to zoom, drag to pan.\n');

%% ========================================================================
function result = iif(condition, trueVal, falseVal)
    if condition
        result = trueVal;
    else
        result = falseVal;
    end
end
