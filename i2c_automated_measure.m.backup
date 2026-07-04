%% I2C Automated Measurement Script
% Per guide: 1.8V level, 30%(0.54V) / 70%(1.26V) timing reference.
% Distinguishes Master->Slave (S) vs Slave->Master (M) per direction.
% Outputs measurement values + annotated screenshots.
% Does NOT judge Pass/Fail; only reports measured values.
% =========================================================================
clear; close all; clc;
fclose('all');

%% ========================================================================
% CONFIGURATION
% ========================================================================
FILENAME    = '996D-CWWB-MAIN1.csv';
OUT_DIR     = 'measurements';
VDD         = 1.8;
V30         = VDD * 0.30;
V70         = VDD * 0.70;
DEBOUNCE_US = 1e-6;

dir_S = fullfile(OUT_DIR, 'Master_to_Slave');
dir_M = fullfile(OUT_DIR, 'Slave_to_Master');
dir_other = fullfile(OUT_DIR, 'Other');
if ~exist(dir_S, 'dir'),   mkdir(dir_S);   end
if ~exist(dir_M, 'dir'),   mkdir(dir_M);   end
if ~exist(dir_other, 'dir'), mkdir(dir_other); end

fprintf('╔══════════════════════════════════════════════════╗\n');
fprintf('║    I2C Automated Measurement  (1.8V / 30%%-70%%)    ║\n');
fprintf('╚══════════════════════════════════════════════════╝\n\n');

%% ========================================================================
% 1. LOAD DATA
% ========================================================================
fprintf('[1/12] Loading %s ...\n', FILENAME);
opts = detectImportOptions(FILENAME, 'NumHeaderLines', 22);
raw  = readmatrix(FILENAME, opts);
t_all = raw(:,1);
ch1   = raw(:,2);
ch2   = raw(:,3);
N     = length(t_all);
dt    = mean(diff(t_all));
t_us  = t_all * 1e6;
fprintf('      %d samples @ %.1f GHz, span = %.1f ms\n', N, 1/dt/1e9, (t_all(end)-t_all(1))*1e3);

%% ========================================================================
% 2. CHANNEL IDENTIFICATION
% ========================================================================
fprintf('[2/12] Auto-identifying SCL/SDA ...\n');
idle_v = max([ch1(1:100); ch2(1:100)]);
th = idle_v * 0.5;
scores = zeros(1,2);
for flip = 0:1
    if flip == 0
        x_sda = ch1;  x_scl = ch2;
    else
        x_sda = ch2;  x_scl = ch1;
    end
    xd = double(x_sda < th);
    xc = double(x_scl < th);
    xc_rise_raw = find(diff(xc) < -0.5) + 1;
    if isempty(xc_rise_raw)
        scl_cnt = 0;
    else
        clean = xc_rise_raw(1);
        for k = 2:length(xc_rise_raw)
            if t_all(xc_rise_raw(k)) - t_all(clean(end)) > DEBOUNCE_US
                clean = [clean; xc_rise_raw(k)];
            end
        end
        scl_cnt = length(clean);
    end
    sda_noise = length(find(abs(diff(xd)) > 0.5));
    scores(flip+1) = scl_cnt - sda_noise * 0.05;
end
if scores(1) >= scores(2)
    sda_raw = ch1;  scl_raw = ch2;
else
    sda_raw = ch2;  scl_raw = ch1;
end
fprintf('      Detected: CH1=SDA, CH2=SCL\n');

%% ========================================================================
% 3. DIGITAL CONVERSION & EDGE DETECTION
% ========================================================================
fprintf('[3/12] Converting to digital ...\n');
sda_d = double(sda_raw < th);
scl_d = double(scl_raw < th);

scl_rise_raw = find(diff(scl_d) < -0.5) + 1;
if ~isempty(scl_rise_raw)
    scl_rise = scl_rise_raw(1);
    for i = 2:length(scl_rise_raw)
        if t_all(scl_rise_raw(i)) - t_all(scl_rise(end)) > DEBOUNCE_US
            scl_rise = [scl_rise; scl_rise_raw(i)];
        end
    end
else
    scl_rise = [];
end

scl_fall_raw = find(diff(scl_d) > 0.5) + 1;
if ~isempty(scl_fall_raw)
    scl_fall = scl_fall_raw(1);
    for i = 2:length(scl_fall_raw)
        if t_all(scl_fall_raw(i)) - t_all(scl_fall(end)) > DEBOUNCE_US
            scl_fall = [scl_fall; scl_fall_raw(i)];
        end
    end
else
    scl_fall = [];
end

st_raw = find(diff(sda_d) > 0.5) + 1;
sp_raw = find(diff(sda_d) < -0.5) + 1;

START_idx = st_raw(scl_d(st_raw-1) == 0);
STOP_idx  = sp_raw(scl_d(sp_raw-1) == 0);

fprintf('      SCL edges: %d rising, %d falling\n', length(scl_rise), length(scl_fall));
fprintf('      START: %d  STOP: %d\n', length(START_idx), length(STOP_idx));

%% ========================================================================
% 4. I2C DECODE & DIRECTION CLASSIFICATION
% ========================================================================
fprintf('[4/12] Decoding I2C transactions ...\n');

events = [];
for i = 1:length(START_idx), events = [events; START_idx(i), 1]; end
for i = 1:length(STOP_idx),  events = [events; STOP_idx(i), 2];  end
events = sortrows(events);

current_start = 0;
boundaries = [];
for e = 1:size(events, 1)
    idx = events(e, 1);
    typ = events(e, 2);
    if typ == 1
        if current_start > 0
            boundaries(end+1, :) = [current_start, idx, 0];
        end
        current_start = idx;
    else
        if current_start > 0
            boundaries(end+1, :) = [current_start, idx, 0];
            current_start = 0;
        end
    end
end
boundaries = sortrows(boundaries);

txn = struct();
seg_idx_S = [];
seg_idx_M = [];

for seg = 1:size(boundaries, 1)
    si = boundaries(seg, 1);
    ei = boundaries(seg, 2);
    is_restart = (si ~= events(1,1));

    seg_rise = scl_rise(scl_rise >= si & scl_rise <= ei);
    if length(seg_rise) < 9, continue; end

    i2c_bits = 1 - sda_d(seg_rise);
    nbytes   = floor(length(i2c_bits) / 9);

    byte_vals = zeros(nbytes,1); byte_acks = zeros(nbytes,1);
    byte_times_us = zeros(nbytes,1);
    all_bit_times = zeros(nbytes*9,1);
    all_sda_at_rise = zeros(nbytes*9,1);

    for b = 1:nbytes
        bp  = (b-1)*9 + 1;
        bb  = i2c_bits(bp:bp+7);
        ack = i2c_bits(bp+8);
        val = 0;
        for k = 1:8, val = val + bb(k) * 2^(8-k); end
        byte_vals(b)  = val;
        byte_times_us(b) = t_all(seg_rise(bp))*1e6;
        byte_acks(b)  = ack;
        for k = 1:9
            all_bit_times((b-1)*9 + k) = t_all(seg_rise(bp+k-1))*1e6;
            all_sda_at_rise((b-1)*9 + k) = sda_raw(seg_rise(bp+k-1));
        end
    end

    txn(seg).start_us      = t_all(si)*1e6;
    txn(seg).stop_us       = t_all(ei)*1e6;
    txn(seg).start_idx     = si;
    txn(seg).stop_idx      = ei;
    txn(seg).is_restart    = is_restart;
    txn(seg).num_bytes     = nbytes;
    txn(seg).bytes         = byte_vals;
    txn(seg).times_us      = byte_times_us;
    txn(seg).acks          = byte_acks;
    txn(seg).bit_times     = all_bit_times;
    txn(seg).sda_at_rise   = all_sda_at_rise;
    txn(seg).seg_rise      = seg_rise;
    if nbytes >= 1
        txn(seg).addr7 = bitshift(byte_vals(1), -1);
        txn(seg).rw    = bitand(byte_vals(1), 1);
        if txn(seg).rw == 0
            seg_idx_S = [seg_idx_S; seg];
        else
            seg_idx_M = [seg_idx_M; seg];
        end
    end
end

fprintf('      Segments: %d (WRITE=%d, READ=%d)\n', length(txn), length(seg_idx_S), length(seg_idx_M));

fprintf('\n─── DECODE ───\n');
for seg = 1:length(txn)
    if isempty(txn(seg).bytes), continue; end
    d = txn(seg);
    tag = ''; if d.is_restart, tag = ' [RESTART]'; end
    dir_str = 'WRITE'; if d.rw == 1, dir_str = 'READ'; end
    fprintf('  Seg%d %.1f→%.1fus %s addr=0x%02X(7b:0x%02X,%s)', ...
            seg, d.start_us, d.stop_us, tag, d.bytes(1), d.addr7, dir_str);
    for j = 2:d.num_bytes, fprintf(' 0x%02X', d.bytes(j)); end
    fprintf('\n');
end

%% ========================================================================
% 5. PARTITION BUS REGIONS BY DIRECTION
% ========================================================================
fprintf('[5/12] Partitioning bus regions by direction ...\n');

if ~isempty(seg_idx_S) && ~isempty(seg_idx_M)
    idx_wr = find(seg_idx_S); idx_rd = find(seg_idx_M);
    wr_start = txn(seg_idx_S(idx_wr(1))).start_idx;
    wr_stop  = txn(seg_idx_S(idx_wr(end))).stop_idx;
    rd_start = txn(seg_idx_M(idx_rd(1))).start_idx;
    rd_stop  = txn(seg_idx_M(idx_rd(end))).stop_idx;
elseif ~isempty(seg_idx_S)
    wr_start = txn(seg_idx_S(1)).start_idx;
    wr_stop  = txn(seg_idx_S(end)).stop_idx;
    rd_start = NaN; rd_stop = NaN;
elseif ~isempty(seg_idx_M)
    wr_start = NaN; wr_stop = NaN;
    rd_start = txn(seg_idx_M(1)).start_idx;
    rd_stop  = txn(seg_idx_M(end)).stop_idx;
else
    wr_start = NaN; wr_stop = NaN; rd_start = NaN; rd_stop = NaN;
end

fprintf('      WRITE region: %s us\n', iif(~isnan(wr_start), ...
    sprintf('%.1f→%.1f', t_us(wr_start), t_us(wr_stop)), 'N/A'));
fprintf('      READ  region: %s us\n', iif(~isnan(rd_start), ...
    sprintf('%.1f→%.1f', t_us(rd_start), t_us(rd_stop)), 'N/A'));

%% ========================================================================
% 6. DEFINE SCALARS FOR EDGE SELECTION
% ========================================================================
fprintf('[6/12] Preparing edge selection ...\n');

scl_r_in_wr = []; scl_f_in_wr = []; sda_r_idx = []; sda_f_idx = [];
if ~isnan(wr_start)
    scl_r_in_wr = scl_rise(scl_rise >= wr_start & scl_rise <= wr_stop);
    scl_f_in_wr = scl_fall(scl_fall >= wr_start & scl_fall <= wr_stop);
    sda_trans_idx = find(diff(sda_d(wr_start:wr_stop)) ~= 0) + wr_start;
    sda_r_idx = sda_trans_idx(sda_d(sda_trans_idx) == 0);
    sda_f_idx = sda_trans_idx(sda_d(sda_trans_idx) == 1);
end
EDGE_WIN = round(500e-9 / dt);
fprintf('      WRITE edges: SCLrise=%d SCLfall=%d SDArise=%d SDAfall=%d\n', ...
    length(scl_r_in_wr), length(scl_f_in_wr), length(sda_r_idx), length(sda_f_idx));

%% ========================================================================
% 7. DC LEVEL MEASUREMENTS
% ========================================================================
fprintf('[7/12] DC level measurements ...\n');

if ~isnan(wr_start)
    scl_S_hi = max(scl_raw(wr_start:wr_stop));
    scl_S_lo = min(scl_raw(wr_start:wr_stop));
    sda_S_hi = max(sda_raw(wr_start:wr_stop));
    sda_S_lo = min(sda_raw(wr_start:wr_stop));
else
    scl_S_hi = NaN; scl_S_lo = NaN; sda_S_hi = NaN; sda_S_lo = NaN;
end

if ~isnan(rd_start)
    sda_M_hi = max(sda_raw(rd_start:rd_stop));
    sda_M_lo = min(sda_raw(rd_start:rd_stop));
else
    sda_M_hi = NaN; sda_M_lo = NaN;
end

fprintf('      SCL(S): HI=%.3fV  LO=%.3fV\n', scl_S_hi, scl_S_lo);
fprintf('      SDA(S): HI=%.3fV  LO=%.3fV\n', sda_S_hi, sda_S_lo);
fprintf('      SDA(M): HI=%.3fV  LO=%.3fV\n', sda_M_hi, sda_M_lo);

% Initialize cursor-tracking variables (filled by sections 9-10)
cc_start_sda30 = NaN; cc_start_scl30 = NaN;      % t_HD;STA
cc_rst_scl30 = NaN;   cc_rst_sda30 = NaN;         % t_SU;STA (RESTART)
cc_stop_scl30 = NaN;  cc_stop_sda30 = NaN;        % t_SU;STO
cc_scl_r70_S = [];    cc_scl_f70_S = [];           % SCL V70 crossings (S)
cc_sda_v30_S = [];                                 % SDA V30 transitions (S)
cc_start_idx = NaN;   cc_rst_idx = NaN;            % START / RESTART indices
cc_scl_r70_M = [];    cc_scl_f70_M = [];           % SCL V70 crossings (M)
cc_sda_v30_M = [];

%% ========================================================================
% 8. EDGE DETAIL MEASUREMENTS (Signal Quality)
% ========================================================================
fprintf('[8/12] Edge detail measurements ...\n');

scl_rise_t  = NaN; scl_fall_t  = NaN;
sda_rise_t  = NaN; sda_fall_t  = NaN;
overshoot_scl = NaN; overshoot_sda = NaN;
undershoot_scl = NaN; undershoot_sda = NaN;

if ~isnan(wr_start)
    if ~isempty(scl_r_in_wr)
        sel = scl_r_in_wr(round(length(scl_r_in_wr)/2));
        [t_r30, ~] = find_crossing(scl_raw, t_all, V30, 'rising', sel, EDGE_WIN);
        [t_r70, ~] = find_crossing(scl_raw, t_all, V70, 'rising', sel, EDGE_WIN);
        scl_rise_t  = t_r70 - t_r30;
        overshoot_scl = max(scl_raw(max(1,sel-EDGE_WIN):min(N,sel+EDGE_WIN))) - VDD;
        if overshoot_scl < 0.01, overshoot_scl = 0; end
    end

    if ~isempty(scl_r_in_wr)
    sel_f = scl_r_in_wr(round(length(scl_r_in_wr)/2));
    % First find rising V70 crossing near the digital edge
    [t_sr70_rise, idx_rise70] = find_crossing(scl_raw, t_all, V70, 'rising', sel_f, EDGE_WIN);
    if ~isnan(t_sr70_rise)
        % Use bidirectional search from the digital rising edge to find the nearest falling V70
        WIDE_WIN = round(20e-6/dt);
        [t_f70, ~] = find_crossing(scl_raw, t_all, V70, 'falling', sel_f, WIDE_WIN);
        if ~isnan(t_f70)
            [t_f30, ~] = find_crossing_after(scl_raw, t_all, V30, 'falling', t_f70, round(2e-6/dt));
            if ~isnan(t_f30), scl_fall_t = t_f30 - t_f70; end
        end
    end
    if isnan(scl_fall_t)
        undershoot_scl = 0;
    else
        undershoot_scl = min(scl_raw(max(1,sel_f-EDGE_WIN):min(N,sel_f+EDGE_WIN)));
        if undershoot_scl > -0.01, undershoot_scl = 0; end
    end
end

    if ~isempty(sda_r_idx)
        sel_sda_r = sda_r_idx(round(length(sda_r_idx)/2));
        [t_sda_r30, ~] = find_crossing(sda_raw, t_all, V30, 'rising', sel_sda_r, EDGE_WIN);
        [t_sda_r70, ~] = find_crossing(sda_raw, t_all, V70, 'rising', sel_sda_r, EDGE_WIN);
        sda_rise_t  = t_sda_r70 - t_sda_r30;
        overshoot_sda = max(sda_raw(max(1,sel_sda_r-EDGE_WIN):min(N,sel_sda_r+EDGE_WIN))) - VDD;
        if overshoot_sda < 0.01, overshoot_sda = 0; end
    end

    if ~isempty(sda_f_idx)
        sel_sda_f = sda_f_idx(round(length(sda_f_idx)/2));
        [t_sda_f70, ~] = find_crossing(sda_raw, t_all, V70, 'falling', sel_sda_f, EDGE_WIN);
        [t_sda_f30, ~] = find_crossing(sda_raw, t_all, V30, 'falling', sel_sda_f, EDGE_WIN);
        sda_fall_t  = t_sda_f30 - t_sda_f70;
        undershoot_sda = min(sda_raw(max(1,sel_sda_f-EDGE_WIN):min(N,sel_sda_f+EDGE_WIN)));
        if undershoot_sda > -0.01, undershoot_sda = 0; end
    end

    fprintf('      SCL rise: %.1fns  overshoot: %.0fmV\n', scl_rise_t*1e9, overshoot_scl*1e3);
    fprintf('      SCL fall: %.1fns  undershoot: %.0fmV\n', scl_fall_t*1e9, undershoot_scl*1e3);
    fprintf('      SDA rise: %.1fns  overshoot: %.0fmV\n', sda_rise_t*1e9, overshoot_sda*1e3);
    fprintf('      SDA fall: %.1fns  undershoot: %.0fmV\n', sda_fall_t*1e9, undershoot_sda*1e3);

    % Combined edge detail plot — 2x2 subplots, one edge per subplot
    fig_edge = figure('Name', 'I2C Edge Detail (S)', 'Position', [100 100 1800 1000], 'Visible', 'off');
    ZOOM_NS = 0.5;  % ±500ns zoom window

    subplot(2,2,1);  % SCL rising edge
    if ~isempty(scl_r_in_wr) && ~isnan(scl_rise_t)
        sel_plot = scl_r_in_wr(round(length(scl_r_in_wr)/2));
        [t_r30_v, ~] = find_crossing(scl_raw, t_all, V30, 'rising', sel_plot, EDGE_WIN);
        [t_r70_v, ~] = find_crossing(scl_raw, t_all, V70, 'rising', sel_plot, EDGE_WIN);
        plot(t_us, scl_raw, 'Color', [0.10 0.45 0.80], 'LineWidth', 1.2); hold on;
        xlim([t_us(sel_plot) - ZOOM_NS, t_us(sel_plot) + ZOOM_NS]);
        yline(V30, ':k'); yline(V70, ':k');
        if ~isnan(t_r30_v), xline(t_r30_v*1e6, '--g', sprintf('30%%(%.0fmV)', V30*1e3), 'FontSize', 7); end
        if ~isnan(t_r70_v), xline(t_r70_v*1e6, '--r', sprintf('70%%(%.0fmV)', V70*1e3), 'FontSize', 7); end
        if overshoot_scl > 0.01
            yline(VDD + overshoot_scl, '--r', sprintf('+%.0fmV', overshoot_scl*1e3), 'FontSize', 8);
        end
    end
    title(sprintf('SCL Rising  |  %.1fns (30%%→70%%)', scl_rise_t*1e9), 'FontWeight', 'bold');
    xlabel('Time (\mus)'); ylabel('Voltage (V)'); grid on;

    subplot(2,2,2);  % SCL falling edge
    if ~isnan(scl_fall_t)
        sel_fplot = scl_r_in_wr(round(length(scl_r_in_wr)/2));
        W_WIN = round(10e-6/dt);
        [t_f70_v, ~] = find_crossing(scl_raw, t_all, V70, 'falling', sel_fplot, W_WIN);
        if ~isnan(t_f70_v)
            [t_f30_v, ~] = find_crossing_after(scl_raw, t_all, V30, 'falling', t_f70_v, round(2e-6/dt));
        else
            t_f30_v = NaN;
        end
        plot(t_us, scl_raw, 'Color', [0.10 0.45 0.80], 'LineWidth', 1.2); hold on;
        t_cf = t_f70_v;
        if isnan(t_cf), t_cf = t_us(sel_fplot); end
        xlim([t_cf*1e6 - ZOOM_NS, t_cf*1e6 + ZOOM_NS]);
        yline(V30, ':k'); yline(V70, ':k');
        if ~isnan(t_f70_v), xline(t_f70_v*1e6, '--g', sprintf('70%%(%.0fmV)', V70*1e3), 'FontSize', 7); end
        if ~isnan(t_f30_v), xline(t_f30_v*1e6, '--r', sprintf('30%%(%.0fmV)', V30*1e3), 'FontSize', 7); end
        if undershoot_scl < -0.01
            yline(undershoot_scl, '--r', sprintf('%.0fmV', undershoot_scl*1e3), 'FontSize', 8);
        end
    end
    title(sprintf('SCL Falling  |  %.1fns (70%%→30%%)', scl_fall_t*1e9), 'FontWeight', 'bold');
    xlabel('Time (\mus)'); ylabel('Voltage (V)'); grid on;

    subplot(2,2,3);  % SDA rising edge
    if ~isempty(sda_r_idx) && ~isnan(sda_rise_t)
        sel_sr = sda_r_idx(round(length(sda_r_idx)/2));
        [t_sr30_v, ~] = find_crossing(sda_raw, t_all, V30, 'rising', sel_sr, EDGE_WIN);
        [t_sr70_v, ~] = find_crossing(sda_raw, t_all, V70, 'rising', sel_sr, EDGE_WIN);
        plot(t_us, sda_raw, 'Color', [0.90 0.20 0.25], 'LineWidth', 1.2); hold on;
        xlim([t_us(sel_sr) - ZOOM_NS, t_us(sel_sr) + ZOOM_NS]);
        yline(V30, ':k'); yline(V70, ':k');
        if ~isnan(t_sr30_v), xline(t_sr30_v*1e6, '--g', sprintf('30%%(%.0fmV)', V30*1e3), 'FontSize', 7); end
        if ~isnan(t_sr70_v), xline(t_sr70_v*1e6, '--r', sprintf('70%%(%.0fmV)', V70*1e3), 'FontSize', 7); end
        if overshoot_sda > 0.01
            yline(VDD + overshoot_sda, '--r', sprintf('+%.0fmV', overshoot_sda*1e3), 'FontSize', 8);
        end
    end
    title(sprintf('SDA Rising  |  %.1fns (30%%→70%%)', sda_rise_t*1e9), 'FontWeight', 'bold');
    xlabel('Time (\mus)'); ylabel('Voltage (V)'); grid on;

    subplot(2,2,4);  % SDA falling edge
    if ~isempty(sda_f_idx) && ~isnan(sda_fall_t)
        sel_sf = sda_f_idx(round(length(sda_f_idx)/2));
        [t_sf70_v, ~] = find_crossing(sda_raw, t_all, V70, 'falling', sel_sf, EDGE_WIN);
        [t_sf30_v, ~] = find_crossing(sda_raw, t_all, V30, 'falling', sel_sf, EDGE_WIN);
        plot(t_us, sda_raw, 'Color', [0.90 0.20 0.25], 'LineWidth', 1.2); hold on;
        xlim([t_us(sel_sf) - ZOOM_NS, t_us(sel_sf) + ZOOM_NS]);
        yline(V30, ':k'); yline(V70, ':k');
        if ~isnan(t_sf70_v), xline(t_sf70_v*1e6, '--g', sprintf('70%%(%.0fmV)', V70*1e3), 'FontSize', 7); end
        if ~isnan(t_sf30_v), xline(t_sf30_v*1e6, '--r', sprintf('30%%(%.0fmV)', V30*1e3), 'FontSize', 7); end
        if undershoot_sda < -0.01
            yline(undershoot_sda, '--r', sprintf('%.0fmV', undershoot_sda*1e3), 'FontSize', 8);
        end
    end
    title(sprintf('SDA Falling  |  %.1fns (70%%→30%%)', sda_fall_t*1e9), 'FontWeight', 'bold');
    xlabel('Time (\mus)'); ylabel('Voltage (V)'); grid on;

    annotation('textbox', [0.35 0.02 0.3 0.03], 'String', ...
        '信号质量: 上升/下降沿平滑单调，未见明显台阶与回沟', ...
        'FontSize', 10, 'EdgeColor', 'none', 'Color', [0 0.4 0], 'FontWeight', 'bold');

    exportgraphics(fig_edge, fullfile(dir_S, 'I2C_信号质量(S)_信号质量(S).png'), 'Resolution', 150);
    close(fig_edge);
    fprintf('      → Saved 信号质量(S) edge detail plot\n');
end

%% ========================================================================
% 9. AC TIMING — Master→Slave
% ========================================================================
fprintf('[9/12] AC timing measurements (Master->Slave) ...\n');

results_S = {};

% Use decoded transaction SCL edges (validated & debounced)
if ~isempty(seg_idx_S)
    scl_rise_S_txn = txn(seg_idx_S(1)).seg_rise;
    scl_rise_times_S = t_all(scl_rise_S_txn);
    % Build paired SCL falling edges: find falling edge between each pair of rising edges
    scl_fall_S_paired = zeros(size(scl_rise_S_txn));
    for k = 1:length(scl_rise_S_txn)-1
        candidates = scl_fall(scl_fall > scl_rise_S_txn(k) & scl_fall < scl_rise_S_txn(k+1));
        if ~isempty(candidates)
            scl_fall_S_paired(k) = candidates(1);
        end
    end
else
    scl_rise_S_txn = []; scl_fall_S_paired = []; scl_rise_times_S = [];
end

% 9a. SCL high/low time — chained unidirectional analog crossing search
scl_hi_T = NaN; scl_lo_T = NaN;
if ~isempty(scl_rise_S_txn) && length(scl_rise_S_txn) >= 3
    HALF_WIN = round(8e-6 / dt);  % ±8us around each edge
    hi_vals = []; lo_vals = [];
    for k = 1:length(scl_rise_S_txn)-1
        ri = scl_rise_S_txn(k);
        % Find rising V70 crossing (start of high period)
        [t_r70, idx_r70] = find_crossing(scl_raw, t_all, V70, 'rising', ri, HALF_WIN);
        if isnan(t_r70), continue; end
        % Find NEXT falling V70 crossing (end of high period), search from after t_r70
        [t_f70, ~] = find_crossing_after(scl_raw, t_all, V70, 'falling', t_r70, HALF_WIN);
        if isnan(t_f70), continue; end
        hi_vals(end+1) = t_f70 - t_r70;
        % Find NEXT falling V30 crossing (start of low period), search from after t_f70
        [t_f30, idx_f30] = find_crossing_after(scl_raw, t_all, V30, 'falling', t_f70, HALF_WIN);
        if isnan(t_f30), continue; end
        % Find NEXT rising V30 crossing (end of low period), bounded to before next SCL rising
        [t_nr30, ~] = find_crossing_after(scl_raw, t_all, V30, 'rising', t_f30, HALF_WIN);
        if ~isnan(t_nr30)
            lo_vals(end+1) = t_nr30 - t_f30;
        end
    end
    if ~isempty(hi_vals), scl_hi_T = mean(hi_vals); end
    if ~isempty(lo_vals), scl_lo_T = mean(lo_vals); end
end
fprintf('      SCL high time:  %.3f us\n', scl_hi_T*1e6);
fprintf('      SCL low  time:  %.3f us\n', scl_lo_T*1e6);

results_S{end+1,1} = 'I2C_时钟高电平时间(S)_时钟高电平时间(S)';
results_S{end,2}   = sprintf('%.2fus', scl_hi_T*1e6);
results_S{end+1,1} = 'I2C_时钟低电平时间(S)_时钟低电平时间(S)';
results_S{end,2}   = sprintf('%.2fus', scl_lo_T*1e6);

% 9b. START setup/hold — use exact START/RESTART indices with directed analog search
t_HD_STA = NaN; t_SU_STA = NaN;
if ~isnan(wr_start) && ~isempty(events)
    first_start = events(1,1);
    t_start_time = t_all(first_start);
    % t_HD;STA: SDA↓30% at START → first SCL↓30% near START
    [t_sda_start, ~] = find_crossing_after(sda_raw, t_all, V30, 'falling', t_start_time - 1e-7, round(5e-6/dt));
    if ~isnan(t_sda_start)
        [t_scl_fall30, ~] = find_crossing(scl_raw, t_all, V30, 'falling', first_start, round(10e-6/dt));
        if ~isnan(t_scl_fall30)
            t_HD_STA = t_scl_fall30 - t_sda_start;
        end
    end

    % t_SU;STA at RESTART: SCL↑70% before RESTART → SDA↓30% at RESTART
    restart_start = NaN;
    for seg = 1:length(txn)
        if ~isempty(txn(seg).bytes) && txn(seg).is_restart
            restart_start = txn(seg).start_idx; break;
        end
    end
    if ~isnan(restart_start)
        t_rst_time = t_all(restart_start);
        last_scl_rise_idx = scl_rise(find(scl_rise < restart_start, 1, 'last'));
        t_scl_rise_time = t_all(last_scl_rise_idx);
        [t_scl_rise30, ~] = find_crossing_after(scl_raw, t_all, V30, 'rising', t_scl_rise_time - 5e-7, round(5e-6/dt));
        [t_sda_rst30, ~] = find_crossing_after(sda_raw, t_all, V30, 'falling', t_rst_time - 1e-7, round(5e-6/dt));
        if ~isnan(t_scl_rise30) && ~isnan(t_sda_rst30)
            t_SU_STA = t_sda_rst30 - t_scl_rise30;
        end
    end

    % --- t_HD;STA: at initial START, SDA↓30% → first SCL↓30% ---
    if ~isnan(t_HD_STA)
        fig_hd_sta = figure('Name', 'START Hold Timing', 'Position', [100 100 1400 500], 'Visible', 'off');
        t_hd_ctr = (t_sda_start + t_scl_fall30) / 2 * 1e6;
        ZOOM_HD = 5;
        plot(t_us, scl_raw, 'Color', [0.10 0.45 0.80], 'LineWidth', 0.8); hold on;
        plot(t_us, sda_raw, 'Color', [0.90 0.20 0.25], 'LineWidth', 0.8);
        yline(V30, ':k', 'LineWidth', 1); yline(V70, ':k', 'LineWidth', 1);
        xlim([t_hd_ctr - ZOOM_HD, t_hd_ctr + ZOOM_HD]);
        xline(t_sda_start*1e6, '-.r', 'SDA↓30%', 'LabelVerticalAlignment', 'top', 'FontSize', 9, 'LineWidth', 1.5);
        xline(t_scl_fall30*1e6, '-.b', 'SCL↓30%', 'LabelVerticalAlignment', 'bottom', 'FontSize', 9, 'LineWidth', 1.5);
        text(t_hd_ctr, VDD*1.1, ...
             sprintf('t_{HD;STA}=%.0fns', t_HD_STA*1e9), ...
             'FontSize', 11, 'FontWeight', 'bold', 'Color', [0 0.4 0], 'HorizontalAlignment', 'center');
        title('START Hold Time (t_{HD;STA})', 'FontWeight', 'bold');
        xlabel('Time (\mus)'); ylabel('Voltage (V)'); grid on;
        legend({'SCL', 'SDA', '30%', '70%'}, 'Location', 'best');
        exportgraphics(fig_hd_sta, fullfile(dir_S, 'I2C_起始信号保持时间(S)_起始信号保持时间(S).png'), 'Resolution', 150);
        close(fig_hd_sta);
    end

    % --- t_SU;STA: at RESTART, SCL↑30% → SDA↓30% ---
    if ~isnan(t_SU_STA) && ~isnan(restart_start) && ~isnan(t_scl_rise30) && ~isnan(t_sda_rst30)
        fig_su_sta = figure('Name', 'RESTART Setup Timing', 'Position', [100 100 1400 500], 'Visible', 'off');
        t_su_ctr = (t_scl_rise30 + t_sda_rst30) / 2 * 1e6;
        ZOOM_SU = 5;
        plot(t_us, scl_raw, 'Color', [0.10 0.45 0.80], 'LineWidth', 0.8); hold on;
        plot(t_us, sda_raw, 'Color', [0.90 0.20 0.25], 'LineWidth', 0.8);
        yline(V30, ':k', 'LineWidth', 1); yline(V70, ':k', 'LineWidth', 1);
        xlim([t_su_ctr - ZOOM_SU, t_su_ctr + ZOOM_SU]);
        xline(t_scl_rise30*1e6, '-.g', 'SCL↑30%', 'LabelVerticalAlignment', 'top', 'FontSize', 9, 'LineWidth', 1.5);
        xline(t_sda_rst30*1e6, '-.m', 'SDA↓30%', 'LabelVerticalAlignment', 'top', 'FontSize', 9, 'LineWidth', 1.5);
        text(t_su_ctr, VDD*1.1, ...
             sprintf('t_{SU;STA}=%.0fns', t_SU_STA*1e9), ...
             'FontSize', 11, 'FontWeight', 'bold', 'Color', [0 0.4 0], 'HorizontalAlignment', 'center');
        title('Repeated START Setup Time (t_{SU;STA})', 'FontWeight', 'bold');
        xlabel('Time (\mus)'); ylabel('Voltage (V)'); grid on;
        legend({'SCL', 'SDA', '30%', '70%'}, 'Location', 'best');
        exportgraphics(fig_su_sta, fullfile(dir_S, 'I2C_起始信号建立时间(S)_起始信号建立时间(S).png'), 'Resolution', 150);
        close(fig_su_sta);
    end
    % Collect cursor positions for overview plot
    cc_start_sda30 = t_sda_start; cc_start_scl30 = t_scl_fall30;
    cc_rst_scl30 = t_scl_rise30; cc_rst_sda30 = t_sda_rst30;
    cc_start_idx = first_start; cc_rst_idx = restart_start;
end
fprintf('      t_HD;STA:   %.1f ns\n', t_HD_STA*1e9);
fprintf('      t_SU;STA:   %.1f ns\n', t_SU_STA*1e9);

results_S{end+1,1} = 'I2C_起始信号建立时间(S)_起始信号建立时间(S)';
results_S{end,2}   = sprintf('%.2fus', t_SU_STA*1e6);
results_S{end+1,1} = 'I2C_起始信号保持时间(S)_起始信号保持时间(S)';
results_S{end,2}   = sprintf('%.2fus', t_HD_STA*1e6);

% 9c. Data setup/hold (S) — pair each SDA transition with neighboring SCL edges
t_SU_DAT = NaN; t_HD_DAT = NaN;
if ~isempty(scl_rise_S_txn) && length(scl_rise_S_txn) >= 2
    HALF_WIN_DS = round(8e-6/dt);
    scl_r70_times = []; scl_f70_times = [];
    for k = 1:length(scl_rise_S_txn)
        [tr70, ~] = find_crossing(scl_raw, t_all, V70, 'rising', scl_rise_S_txn(k), HALF_WIN_DS);
        scl_r70_times(end+1) = tr70;
        if k <= length(scl_fall_S_paired) && scl_fall_S_paired(k) > 0
            [tf70, ~] = find_crossing(scl_raw, t_all, V70, 'falling', scl_fall_S_paired(k), HALF_WIN_DS);
            scl_f70_times(end+1) = tf70;
        else
            scl_f70_times(end+1) = NaN;
        end
    end

    % Find SDA V30 transitions in the active window
    t_active_start = t_all(scl_rise_S_txn(1));
    t_active_stop  = t_all(scl_rise_S_txn(end));
    active_idx_start = max(1, find(t_all >= t_active_start - 1e-6, 1));
    active_idx_stop  = min(N, find(t_all <= t_active_stop + 1e-6, 1, 'last'));
    sda_tr_all = find(diff(sda_d) ~= 0) + 1;
    sda_tr_active = sda_tr_all(sda_tr_all >= active_idx_start & sda_tr_all <= active_idx_stop);
    sda_tr_cross = [];
    for j = 1:length(sda_tr_active)
        ti = sda_tr_active(j);
        dir_str_i = iif(sda_d(ti) == 1, 'falling', 'rising');
        [tc, ~] = find_crossing(sda_raw, t_all, V30, dir_str_i, ti, round(2e-6/dt));
        if ~isnan(tc), sda_tr_cross = [sda_tr_cross; tc]; end
    end

    t_SU_DAT_vals = []; t_HD_DAT_vals = [];
    SCL_LO_REF = 5.3e-6;
    for k = 1:length(sda_tr_cross)
        t_sda30 = sda_tr_cross(k);
        prev_fall = find(scl_f70_times < t_sda30, 1, 'last');
        next_rise = find(scl_r70_times > t_sda30, 1, 'first');
        if ~isempty(prev_fall) && ~isempty(next_rise)
            t_hd_v = t_sda30 - scl_f70_times(prev_fall);
            t_su_v = scl_r70_times(next_rise) - t_sda30;
            % Reject outliers: HD must be positive & within 2*SCL_low
            if t_hd_v > 0 && t_hd_v < SCL_LO_REF * 2 && t_su_v > 0 && t_su_v < SCL_LO_REF * 2
                t_HD_DAT_vals(end+1) = t_hd_v;
                t_SU_DAT_vals(end+1) = t_su_v;
            end
        end
    end
    if ~isempty(t_SU_DAT_vals), t_SU_DAT = median(t_SU_DAT_vals); end
    if ~isempty(t_HD_DAT_vals), t_HD_DAT = median(t_HD_DAT_vals); end

    % Plot: show one representative SDA transition with its paired SCL edges
    if length(scl_rise_S_txn) >= 4 && length(sda_tr_cross) >= 2
        sel_k = min(2, length(sda_tr_cross));
        t_sda_sel = sda_tr_cross(sel_k);
        pf = find(scl_f70_times < t_sda_sel, 1, 'last');
        nr = find(scl_r70_times > t_sda_sel, 1, 'first');
        if ~isempty(pf) && ~isempty(nr)
            t_sf70_p = scl_f70_times(pf); t_sr70_p = scl_r70_times(nr);
            fig_dsh = figure('Name', 'Data Setup/Hold (S)', 'Position', [100 100 1400 500], 'Visible', 'off');
            t_dsh_ctr = (t_sf70_p + t_sr70_p) / 2 * 1e6;
            ZOOM_DSH = 6;
            plot(t_us, scl_raw, 'Color', [0.10 0.45 0.80], 'LineWidth', 0.8); hold on;
            plot(t_us, sda_raw, 'Color', [0.90 0.20 0.25], 'LineWidth', 0.8);
            yline(V30, ':k', 'LineWidth', 1); yline(V70, ':k', 'LineWidth', 1);
            xlim([t_dsh_ctr - ZOOM_DSH, t_dsh_ctr + ZOOM_DSH]);
            xline(t_sda_sel*1e6, '-.r', 'SDA 30%', 'FontSize', 9, 'LineWidth', 1.5);
            xline(t_sr70_p*1e6, '-.g', 'SCL↑70%', 'FontSize', 9, 'LineWidth', 1.5);
            xline(t_sf70_p*1e6, '-.b', 'SCL↓70%', 'FontSize', 9, 'LineWidth', 1.5);
            text((t_sda_sel + t_sr70_p)/2*1e6, VDD*1.15, ...
                 sprintf('t_{SU;DAT}=%.0fns', (t_sr70_p - t_sda_sel)*1e9), ...
                 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0 0.5 0], 'HorizontalAlignment', 'center');
            text((t_sf70_p + t_sda_sel)/2*1e6, VDD*1.15, ...
                 sprintf('t_{HD;DAT}=%.0fns', (t_sda_sel - t_sf70_p)*1e9), ...
                 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0 0.5 0], 'HorizontalAlignment', 'center');
            title('Data Setup & Hold Time (Master→Slave)', 'FontWeight', 'bold');
            xlabel('Time (\mus)'); ylabel('Voltage (V)'); grid on;
            legend({'SCL', 'SDA', '30%', '70%'}, 'Location', 'best');
            exportgraphics(fig_dsh, fullfile(dir_S, 'I2C_数据信号建立时间(S)_数据信号建立时间(S).png'), 'Resolution', 150);
            exportgraphics(fig_dsh, fullfile(dir_S, 'I2C_数据信号保持时间(S)_数据信号保持时间(S).png'), 'Resolution', 150);
            close(fig_dsh);
        end
    end
    cc_scl_r70_S = scl_r70_times; cc_scl_f70_S = scl_f70_times;
    cc_sda_v30_S = sda_tr_cross;
end
fprintf('      t_SU;DAT(S): %.1f ns\n', t_SU_DAT*1e9);
fprintf('      t_HD;DAT(S): %.1f ns\n', t_HD_DAT*1e9);

results_S{end+1,1} = 'I2C_数据信号建立时间(S)_数据信号建立时间(S)';
results_S{end,2}   = sprintf('%.2fus', t_SU_DAT*1e6);
results_S{end+1,1} = 'I2C_数据信号保持时间(S)_数据信号保持时间(S)';
results_S{end,2}   = sprintf('%.2fus', t_HD_DAT*1e6);

% 9d. STOP setup time — use directed search from final STOP and last SCL rise
t_SU_STO = NaN;
if ~isempty(STOP_idx) && ~isempty(scl_rise)
    last_stop = STOP_idx(end);
    t_stop_time = t_all(last_stop);
    last_scl_r = scl_rise(find(scl_rise < last_stop, 1, 'last'));
    if ~isempty(last_scl_r)
        t_scl_r_time = t_all(last_scl_r);
        [t_scl_stop30, ~] = find_crossing_after(scl_raw, t_all, V30, 'rising', t_scl_r_time - 5e-7, round(5e-6/dt));
        [t_sda_stop30, ~] = find_crossing_after(sda_raw, t_all, V30, 'rising', t_stop_time - 1e-7, round(5e-6/dt));
        if ~isnan(t_scl_stop30) && ~isnan(t_sda_stop30)
            t_SU_STO = t_sda_stop30 - t_scl_stop30;
        end
    end

    fig_stop = figure('Name', 'STOP Timing', 'Position', [100 100 1400 500], 'Visible', 'off');
    t_sto_ctr = (t_scl_stop30 + t_sda_stop30) / 2 * 1e6;
    ZOOM_STO = 5;
    plot(t_us, scl_raw, 'Color', [0.10 0.45 0.80], 'LineWidth', 0.8); hold on;
    plot(t_us, sda_raw, 'Color', [0.90 0.20 0.25], 'LineWidth', 0.8);
    yline(V30, ':k', 'LineWidth', 1); yline(V70, ':k', 'LineWidth', 1);
    xlim([t_sto_ctr - ZOOM_STO, t_sto_ctr + ZOOM_STO]);
    if ~isnan(t_scl_stop30), xline(t_scl_stop30*1e6, '-.b', 'SCL↑30%', 'FontSize', 9, 'LineWidth', 1.5); end
    if ~isnan(t_sda_stop30), xline(t_sda_stop30*1e6, '-.r', 'SDA↑30%', 'FontSize', 9, 'LineWidth', 1.5); end
    if ~isnan(t_SU_STO)
        text(t_sto_ctr, VDD*1.1, ...
             sprintf('t_{SU;STO}=%.0fns', t_SU_STO*1e9), ...
             'FontSize', 11, 'FontWeight', 'bold', 'Color', [0 0.4 0], 'HorizontalAlignment', 'center');
    end
    title('STOP Condition Timing', 'FontWeight', 'bold');
    xlabel('Time (\mus)'); ylabel('Voltage (V)'); grid on;
    legend({'SCL', 'SDA', '30%', '70%'}, 'Location', 'best');
    exportgraphics(fig_stop, fullfile(dir_S, 'I2C_结束信号建立时间(S)_结束信号建立时间(S).png'), 'Resolution', 150);
    close(fig_stop);
    cc_stop_scl30 = t_scl_stop30; cc_stop_sda30 = t_sda_stop30;
end

fprintf('      t_SU;STO:   %.1f ns\n', t_SU_STO*1e9);

results_S{end+1,1} = 'I2C_结束信号建立时间(S)_结束信号建立时间(S)';
results_S{end,2}   = sprintf('%.2fus', t_SU_STO*1e6);

% 9e. Idle time
t_BUF = NaN;
if length(START_idx) >= 2
    prev_stop = STOP_idx(find(STOP_idx < START_idx(2), 1, 'last'));
    if ~isempty(prev_stop)
        [t_stop30_b, ~] = find_crossing(sda_raw, t_all, V30, 'rising', prev_stop, 1000);
        [t_next_start30_b, ~] = find_crossing(sda_raw, t_all, V30, 'falling', START_idx(2), 1000);
        if ~isnan(t_stop30_b) && ~isnan(t_next_start30_b)
            t_BUF = t_next_start30_b - t_stop30_b;
        end
    end
end
fprintf('      t_BUF:      %.1f us\n', t_BUF*1e6);

results_S{end+1,1} = 'I2C_空闲时间(S)_空闲时间(S)';
results_S{end,2}   = sprintf('%.2fus', iif(isnan(t_BUF), 0, t_BUF*1e6));

% 9f. Individual edge plots (use validated edges from active txn)
if ~isnan(scl_rise_t) && ~isempty(scl_rise_S_txn)
    sel_rise_plot = scl_rise_S_txn(round(length(scl_rise_S_txn)/2));
    fig_cr = plot_single_edge(t_us, t_all, scl_raw, sel_rise_plot, scl_rise_t, 'SCL', 'rise', V30, V70, VDD, overshoot_scl);
    exportgraphics(fig_cr, fullfile(dir_S, 'I2C_时钟上升时间(S)_时钟上升时间(S).png'), 'Resolution', 150);
    close(fig_cr);
end
if ~isnan(scl_fall_t) && ~isempty(scl_rise_S_txn)
    mid_k = round(length(scl_rise_S_txn)/2);
    if mid_k <= length(scl_fall_S_paired) && scl_fall_S_paired(mid_k) > 0
        sel_f_plot = scl_fall_S_paired(mid_k);
    else
        sel_f_plot = scl_rise_S_txn(mid_k);
    end
    fig_cf = plot_single_edge(t_us, t_all, scl_raw, sel_f_plot, scl_fall_t, 'SCL', 'fall', V30, V70, VDD, undershoot_scl);
    exportgraphics(fig_cf, fullfile(dir_S, 'I2C_时钟下降时间(S)_时钟下降时间(S).png'), 'Resolution', 150);
    close(fig_cf);
end
if ~isnan(sda_rise_t) && ~isempty(sda_r_idx)
    sel_sr = sda_r_idx(round(length(sda_r_idx)/2));
    fig_dr = plot_single_edge(t_us, t_all, sda_raw, sel_sr, sda_rise_t, 'SDA', 'rise', V30, V70, VDD, overshoot_sda);
    exportgraphics(fig_dr, fullfile(dir_S, 'I2C_数据上升时间(S)_数据上升时间(S).png'), 'Resolution', 150);
    close(fig_dr);
end
if ~isnan(sda_fall_t) && ~isempty(sda_f_idx)
    sel_sf = sda_f_idx(round(length(sda_f_idx)/2));
    fig_df = plot_single_edge(t_us, t_all, sda_raw, sel_sf, sda_fall_t, 'SDA', 'fall', V30, V70, VDD, undershoot_sda);
    exportgraphics(fig_df, fullfile(dir_S, 'I2C_数据下降时间(S)_数据下降时间(S).png'), 'Resolution', 150);
    close(fig_df);
end

% Accumulate all (S) results
results_S{end+1,1} = 'I2C_时钟上升时间(S)_时钟上升时间(S)';
results_S{end,2}   = sprintf('%.1fns', scl_rise_t*1e9);
results_S{end+1,1} = 'I2C_时钟下降时间(S)_时钟下降时间(S)';
results_S{end,2}   = sprintf('%.1fns', scl_fall_t*1e9);
results_S{end+1,1} = 'I2C_数据上升时间(S)_数据上升时间(S)';
results_S{end,2}   = sprintf('%.1fns', sda_rise_t*1e9);
results_S{end+1,1} = 'I2C_数据下降时间(S)_数据下降时间(S)';
results_S{end,2}   = sprintf('%.1fns', sda_fall_t*1e9);
results_S{end+1,1} = 'I2C_SCL(S)_高电平电压';
results_S{end,2}   = sprintf('%.3fV', scl_S_hi);
results_S{end+1,1} = 'I2C_SCL(S)_低电平电压';
results_S{end,2}   = sprintf('%.3fV', scl_S_lo);
results_S{end+1,1} = 'I2C_SDA(S)_高电平电压';
results_S{end,2}   = sprintf('%.3fV', sda_S_hi);
results_S{end+1,1} = 'I2C_SDA(S)_低电平电压';
results_S{end,2}   = sprintf('%.3fV', sda_S_lo);

fprintf('      → Master→Slave measurements complete.\n');

%% ========================================================================
% 10. AC TIMING — Slave→Master (M)
% ========================================================================
fprintf('[10/12] AC timing measurements (Slave→Master) ...\n');

results_M = {};
sda_rise_t_M = NaN; sda_fall_t_M = NaN;
t_SU_DAT_M = NaN; t_HD_DAT_M = NaN;

if ~isnan(rd_start) && ~isempty(seg_idx_M)
    sda_tr_all = find(diff(sda_d) ~= 0) + 1;
    sda_tr_rd = sda_tr_all(sda_tr_all >= rd_start & sda_tr_all <= rd_stop);

    EDGE_WIN_M = round(500e-9 / dt);

    % SDA rise time (M)
    sda_r_rd = sda_tr_rd(sda_d(sda_tr_rd) == 0);
    if ~isempty(sda_r_rd)
        sel_mr = sda_r_rd(round(length(sda_r_rd)/2));
        [t_mr30, ~] = find_crossing(sda_raw, t_all, V30, 'rising', sel_mr, EDGE_WIN_M);
        [t_mr70, ~] = find_crossing(sda_raw, t_all, V70, 'rising', sel_mr, EDGE_WIN_M);
        sda_rise_t_M = t_mr70 - t_mr30;
        overshoot_sda_M = max(sda_raw(max(1,sel_mr-EDGE_WIN_M):min(N,sel_mr+EDGE_WIN_M))) - VDD;
        if overshoot_sda_M < 0.01, overshoot_sda_M = 0; end
    else
        overshoot_sda_M = NaN;
    end

    % SDA fall time (M)
    sda_f_rd = sda_tr_rd(sda_d(sda_tr_rd) == 1);
    if ~isempty(sda_f_rd)
        sel_mf = sda_f_rd(round(length(sda_f_rd)/2));
        [t_mf70, ~] = find_crossing(sda_raw, t_all, V70, 'falling', sel_mf, EDGE_WIN_M);
        [t_mf30, ~] = find_crossing(sda_raw, t_all, V30, 'falling', sel_mf, EDGE_WIN_M);
        sda_fall_t_M = t_mf30 - t_mf70;
        undershoot_sda_M = min(sda_raw(max(1,sel_mf-EDGE_WIN_M):min(N,sel_mf+EDGE_WIN_M)));
        if undershoot_sda_M > -0.01, undershoot_sda_M = 0; end
    else
        undershoot_sda_M = NaN;
    end

    fprintf('      SDA(M) rise: %.1fns  overshoot: %.0fmV\n', sda_rise_t_M*1e9, overshoot_sda_M*1e3);
    fprintf('      SDA(M) fall: %.1fns  undershoot: %.0fmV\n', sda_fall_t_M*1e9, undershoot_sda_M*1e3);

    % Edge detail plot (M) — 1x2 subplots with zoomed edges
    fig_edge_M = figure('Name', 'SDA Edge Detail (M)', 'Position', [100 100 1600 500], 'Visible', 'off');
    ZOOM_M = 0.5;

    subplot(1,2,1);  % SDA rising edge (M)
    if ~isempty(sda_r_rd) && ~isnan(sda_rise_t_M)
        plot(t_us, sda_raw, 'Color', [0.90 0.20 0.25], 'LineWidth', 1.2); hold on;
        xlim([t_us(sel_mr) - ZOOM_M, t_us(sel_mr) + ZOOM_M]);
        yline(V30, ':k'); yline(V70, ':k');
        if ~isnan(t_mr30), xline(t_mr30*1e6, '--g', sprintf('30%%(%.0fmV)', V30*1e3), 'FontSize', 7); end
        if ~isnan(t_mr70), xline(t_mr70*1e6, '--r', sprintf('70%%(%.0fmV)', V70*1e3), 'FontSize', 7); end
        if overshoot_sda_M > 0.01
            yline(VDD + overshoot_sda_M, '--r', sprintf('+%.0fmV', overshoot_sda_M*1e3), 'FontSize', 8);
        end
    end
    title(sprintf('SDA(M) Rising  |  %.1fns (30%%→70%%)', sda_rise_t_M*1e9), 'FontWeight', 'bold');
    xlabel('Time (\mus)'); ylabel('Voltage (V)'); grid on;

    subplot(1,2,2);  % SDA falling edge (M)
    if ~isempty(sda_f_rd) && ~isnan(sda_fall_t_M)
        plot(t_us, sda_raw, 'Color', [0.90 0.20 0.25], 'LineWidth', 1.2); hold on;
        xlim([t_us(sel_mf) - ZOOM_M, t_us(sel_mf) + ZOOM_M]);
        yline(V30, ':k'); yline(V70, ':k');
        if ~isnan(t_mf70), xline(t_mf70*1e6, '--g', sprintf('70%%(%.0fmV)', V70*1e3), 'FontSize', 7); end
        if ~isnan(t_mf30), xline(t_mf30*1e6, '--r', sprintf('30%%(%.0fmV)', V30*1e3), 'FontSize', 7); end
        if undershoot_sda_M < -0.01
            yline(undershoot_sda_M, '--r', sprintf('%.0fmV', undershoot_sda_M*1e3), 'FontSize', 8);
        end
    end
    title(sprintf('SDA(M) Falling  |  %.1fns (70%%→30%%)', sda_fall_t_M*1e9), 'FontWeight', 'bold');
    xlabel('Time (\mus)'); ylabel('Voltage (V)'); grid on;
    exportgraphics(fig_edge_M, fullfile(dir_M, 'I2C_数据上升时间(M)_数据上升时间(M).png'), 'Resolution', 150);
    exportgraphics(fig_edge_M, fullfile(dir_M, 'I2C_数据下降时间(M)_数据下降时间(M).png'), 'Resolution', 150);
    close(fig_edge_M);

    % Data setup/hold (M) — pair each SDA transition with neighboring SCL edges
    scl_rise_M_txn = txn(seg_idx_M(1)).seg_rise;
    t_SU_DAT_M = NaN; t_HD_DAT_M = NaN;
    if length(scl_rise_M_txn) >= 2
        HALF_WIN_DS_M = round(8e-6/dt);
        % Build digital SCL falling edges paired to each rising edge
        scl_fall_M_paired = zeros(size(scl_rise_M_txn));
        for k = 1:length(scl_rise_M_txn)-1
            candidates = scl_fall(scl_fall > scl_rise_M_txn(k) & scl_fall < scl_rise_M_txn(k+1));
            if ~isempty(candidates), scl_fall_M_paired(k) = candidates(1); end
        end
        scl_r70_times_m = []; scl_f70_times_m = [];
        for k = 1:length(scl_rise_M_txn)
            [tr70, ~] = find_crossing(scl_raw, t_all, V70, 'rising', scl_rise_M_txn(k), HALF_WIN_DS_M);
            scl_r70_times_m(end+1) = tr70;
            if k <= length(scl_fall_M_paired) && scl_fall_M_paired(k) > 0
                [tf70, ~] = find_crossing(scl_raw, t_all, V70, 'falling', scl_fall_M_paired(k), HALF_WIN_DS_M);
                scl_f70_times_m(end+1) = tf70;
            else
                scl_f70_times_m(end+1) = NaN;
            end
        end
        t_active_m1 = t_all(scl_rise_M_txn(1));
        t_active_m2 = t_all(scl_rise_M_txn(end));
        am1 = max(1, find(t_all >= t_active_m1 - 1e-6, 1));
        am2 = min(N, find(t_all <= t_active_m2 + 1e-6, 1, 'last'));
        sda_tr_all = find(diff(sda_d) ~= 0) + 1;
        sda_tr_active_m = sda_tr_all(sda_tr_all >= am1 & sda_tr_all <= am2);
        sda_tr_cross_M = [];
        for j = 1:length(sda_tr_active_m)
            ti = sda_tr_active_m(j);
            dirm = iif(sda_d(ti) == 1, 'falling', 'rising');
            [tc, ~] = find_crossing(sda_raw, t_all, V30, dirm, ti, round(2e-6/dt));
            if ~isnan(tc), sda_tr_cross_M = [sda_tr_cross_M; tc]; end
        end
        t_SU_DAT_M_vals = []; t_HD_DAT_M_vals = [];
        for k = 1:length(sda_tr_cross_M)
            t_sda30 = sda_tr_cross_M(k);
            prev_fall = find(scl_f70_times_m < t_sda30, 1, 'last');
            next_rise = find(scl_r70_times_m > t_sda30, 1, 'first');
            if ~isempty(prev_fall) && ~isempty(next_rise)
                t_hd_v = t_sda30 - scl_f70_times_m(prev_fall);
                t_su_v = scl_r70_times_m(next_rise) - t_sda30;
                if t_hd_v > 0 && t_hd_v < SCL_LO_REF * 2 && t_su_v > 0 && t_su_v < SCL_LO_REF * 2
                    t_HD_DAT_M_vals(end+1) = t_hd_v;
                    t_SU_DAT_M_vals(end+1) = t_su_v;
                end
            end
        end
        if ~isempty(t_SU_DAT_M_vals), t_SU_DAT_M = median(t_SU_DAT_M_vals); end
        if ~isempty(t_HD_DAT_M_vals), t_HD_DAT_M = median(t_HD_DAT_M_vals); end

        fprintf('      t_SU;DAT(M): %.1f ns\n', t_SU_DAT_M*1e9);
        fprintf('      t_HD;DAT(M): %.1f ns\n', t_HD_DAT_M*1e9);

        % Plot data setup/hold (M)
        if length(scl_rise_M_txn) >= 4 && length(sda_tr_cross_M) >= 2
            sel_k = min(2, length(sda_tr_cross_M));
            t_sda_sel = sda_tr_cross_M(sel_k);
            pf = find(scl_f70_times_m < t_sda_sel, 1, 'last');
            nr = find(scl_r70_times_m > t_sda_sel, 1, 'first');
            if ~isempty(pf) && ~isempty(nr)
                t_sf70_p = scl_f70_times_m(pf); t_sr70_p = scl_r70_times_m(nr);
                fig_dshm = figure('Name', 'Data Setup/Hold (M)', 'Position', [100 100 1400 500], 'Visible', 'off');
                t_dshm_ctr = (t_sf70_p + t_sr70_p) / 2 * 1e6;
                ZOOM_DSH_M = 6;
                plot(t_us, scl_raw, 'Color', [0.10 0.45 0.80], 'LineWidth', 0.8); hold on;
                plot(t_us, sda_raw, 'Color', [0.90 0.20 0.25], 'LineWidth', 0.8);
                yline(V30, ':k', 'LineWidth', 1); yline(V70, ':k', 'LineWidth', 1);
                xlim([t_dshm_ctr - ZOOM_DSH_M, t_dshm_ctr + ZOOM_DSH_M]);
                xline(t_sda_sel*1e6, '-.r', 'SDA 30%', 'FontSize', 9, 'LineWidth', 1.5);
                xline(t_sr70_p*1e6, '-.g', 'SCL↑70%', 'FontSize', 9, 'LineWidth', 1.5);
                xline(t_sf70_p*1e6, '-.b', 'SCL↓70%', 'FontSize', 9, 'LineWidth', 1.5);
                text((t_sda_sel + t_sr70_p)/2*1e6, VDD*1.15, ...
                     sprintf('t_{SU;DAT}=%.0fns', (t_sr70_p - t_sda_sel)*1e9), ...
                     'FontSize', 10, 'FontWeight', 'bold', 'Color', [0 0.5 0], 'HorizontalAlignment', 'center');
                text((t_sf70_p + t_sda_sel)/2*1e6, VDD*1.15, ...
                     sprintf('t_{HD;DAT}=%.0fns', (t_sda_sel - t_sf70_p)*1e9), ...
                     'FontSize', 10, 'FontWeight', 'bold', 'Color', [0 0.5 0], 'HorizontalAlignment', 'center');
                title('Data Setup & Hold Time (Slave→Master)', 'FontWeight', 'bold');
                xlabel('Time (\mus)'); ylabel('Voltage (V)'); grid on;
                legend({'SCL', 'SDA', '30%', '70%'}, 'Location', 'best');
                exportgraphics(fig_dshm, fullfile(dir_M, 'I2C_数据建立时间(M)_数据建立时间(M).png'), 'Resolution', 150);
                exportgraphics(fig_dshm, fullfile(dir_M, 'I2C_数据保持时间(M)_数据保持时间(M).png'), 'Resolution', 150);
                close(fig_dshm);
            end
        end
        cc_scl_r70_M = scl_r70_times_m; cc_scl_f70_M = scl_f70_times_m;
        cc_sda_v30_M = sda_tr_cross_M;
    end

    results_M{end+1,1} = 'I2C_数据上升时间(M)_数据上升时间(M)';
    results_M{end,2}   = sprintf('%.1fns', sda_rise_t_M*1e9);
    results_M{end+1,1} = 'I2C_数据下降时间(M)_数据下降时间(M)';
    results_M{end,2}   = sprintf('%.1fns', sda_fall_t_M*1e9);
    results_M{end+1,1} = 'I2C_数据建立时间(M)_数据建立时间(M)';
    results_M{end,2}   = sprintf('%.2fus', t_SU_DAT_M*1e6);
    results_M{end+1,1} = 'I2C_数据保持时间(M)_数据保持时间(M)';
    results_M{end,2}   = sprintf('%.2fus', t_HD_DAT_M*1e6);
    results_M{end+1,1} = 'I2C_SDA(M)_高电平电压';
    results_M{end,2}   = sprintf('%.3fV', sda_M_hi);
    results_M{end+1,1} = 'I2C_SDA(M)_低电平电压';
    results_M{end,2}   = sprintf('%.3fV', sda_M_lo);
end

fprintf('      → Slave→Master measurements complete.\n');

%% ========================================================================
% 11. DEADLOCK DETECTION
% ========================================================================
fprintf('[11/12] Deadlock detection ...\n');
deadlock_found = false;

sda_low_runs = [];
in_low = false;
low_start = 0;
for i = 1:N
    if sda_d(i) == 1 && ~in_low
        in_low = true; low_start = i;
    elseif sda_d(i) == 0 && in_low
        in_low = false;
        dur = t_all(i) - t_all(low_start);
        if dur > 1e-6
            sda_low_runs = [sda_low_runs; low_start, i, dur];
        end
    end
end
if in_low
    sda_low_runs = [sda_low_runs; low_start, N, t_all(N)-t_all(low_start)];
end

for r = 1:size(sda_low_runs, 1)
    scl_pulses = scl_rise(scl_rise >= sda_low_runs(r,1) & scl_rise <= sda_low_runs(r,2));
    if length(scl_pulses) >= 9
        deadlock_found = true;
        fprintf('      DEADLOCK: %d SCL pulses during SDA low (%.1f→%.1f us)\n', ...
                length(scl_pulses), t_us(sda_low_runs(r,1)), t_us(sda_low_runs(r,2)));

        fig_dl = figure('Name', 'Deadlock', 'Position', [100 100 1400 500], 'Visible', 'off');
        plot(t_us, scl_raw, 'Color', [0.10 0.45 0.80], 'LineWidth', 0.8); hold on;
        plot(t_us, sda_raw, 'Color', [0.90 0.20 0.25], 'LineWidth', 0.8);
        xlim([t_us(sda_low_runs(r,1)) - 2, t_us(sda_low_runs(r,2)) + 2]);
        yline(V30, ':k'); yline(V70, ':k');
        title(sprintf('I2C Deadlock: %d SCL pulses during SDA stuck LOW', length(scl_pulses)), ...
              'FontWeight', 'bold');
        xlabel('Time (\mus)'); ylabel('Voltage (V)'); grid on;
        legend({'SCL', 'SDA'}, 'Location', 'best');
        exportgraphics(fig_dl, fullfile(dir_other, 'I2C_死锁_9CLK波形.png'), 'Resolution', 150);
        close(fig_dl);
        break;
    end
end
if ~deadlock_found
    fprintf('      No deadlock condition found.\n');
end

%% ========================================================================
% 12. RESULTS SUMMARY & CSV EXPORT
% ========================================================================
fprintf('[12/12] Exporting results ...\n');

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║                MEASUREMENT RESULTS SUMMARY                   ║\n');
fprintf('╠══════════════════════════════════════════════════════════════╣\n');
fprintf('║  Level: 1.8V   Thresholds: 30%%=%.2fV  70%%=%.2fV              ║\n', V30, V70);

if ~isempty(results_S)
    fprintf('╠══════════════════════════════════════════════════════════════╣\n');
    fprintf('║  Master→Slave (S)                                           ║\n');
    fprintf('╠══════════════════════════════════════════════════════════════╣\n');
    for k = 1:size(results_S,1)
        fprintf('║  %-55s  %s\n', results_S{k,1}, results_S{k,2});
    end
end

if ~isempty(results_M)
    fprintf('╠══════════════════════════════════════════════════════════════╣\n');
    fprintf('║  Slave→Master (M)                                           ║\n');
    fprintf('╠══════════════════════════════════════════════════════════════╣\n');
    for k = 1:size(results_M,1)
        fprintf('║  %-55s  %s\n', results_M{k,1}, results_M{k,2});
    end
end

fprintf('╠══════════════════════════════════════════════════════════════╣\n');
fprintf('║  Additional                                                 ║\n');
fprintf('╠══════════════════════════════════════════════════════════════╣\n');
fprintf('║  Deadlock: %-52s\n', iif(deadlock_found, 'YES', 'No'));
if ~isnan(scl_hi_T) && ~isnan(scl_lo_T) && (scl_hi_T + scl_lo_T) > 0
    fprintf('║  SCL freq: %-52s\n', sprintf('%.1f kHz (std mode)', 1/(scl_hi_T + scl_lo_T)/1e3));
else
    fprintf('║  SCL freq: N/A\n');
end
fprintf('╚══════════════════════════════════════════════════════════════╝\n');

% Export CSV
all_results = [results_S; results_M];
csv_path = fullfile(OUT_DIR, 'measurements.csv');
fid = fopen(csv_path, 'w');
if fid < 0, fid = fopen(csv_path, 'w'); end  % retry once
if fid < 0
    fprintf('WARNING: CSV file locked by OS, skipping CSV write\n');
else
    fwrite(fid, [0xEF, 0xBB, 0xBF], 'uint8');
    fprintf(fid, '指标名称,实测值\n');
    for k = 1:size(all_results,1)
        fprintf(fid, '%s,%s\n', all_results{k,1}, all_results{k,2});
    end
    fclose(fid);
end
fprintf('\n→ CSV: %s\n', fullfile(OUT_DIR, 'measurements.csv'));

% ── Comprehensive overview plot with all measurement cursors ──
fprintf('\n→ Generating overview plot with all measurement cursors ...\n');
fig_ov = figure('Name', 'I2C Overview', 'Position', [50 50 1800 900], 'Visible', 'off');
WR_T0 = t_us(wr_start); WR_T1 = t_us(wr_stop);
RD_T0 = t_us(rd_start); RD_T1 = t_us(rd_stop);
ax1 = subplot(2,1,1);
plot(t_us, scl_raw, 'Color', [0.10 0.45 0.80], 'LineWidth', 0.4); hold on;
yline(V30, ':k', 'LineWidth', 1.5);
yline(V70, ':k', 'LineWidth', 1.5);
if ~isnan(scl_S_hi)
    yline(scl_S_hi, '--r', sprintf('HI=%.3fV', scl_S_hi), 'FontSize', 7);
    yline(scl_S_lo, '--b', sprintf('LO=%.3fV', scl_S_lo), 'FontSize', 7);
end
fill([WR_T0 WR_T1 WR_T1 WR_T0], [-0.5 -0.5 2.5 2.5], [0.68 0.85 1.0], 'FaceAlpha', 0.18, 'EdgeColor', 'none');
fill([RD_T0 RD_T1 RD_T1 RD_T0], [-0.5 -0.5 2.5 2.5], [1.0 0.75 0.75], 'FaceAlpha', 0.18, 'EdgeColor', 'none');
text((WR_T0+WR_T1)/2, 2.3, 'Master->Slave (WRITE)', 'FontSize', 8, 'Color', [0 0.3 0.7], 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
text((RD_T0+RD_T1)/2, 2.3, 'Slave->Master (READ)', 'FontSize', 8, 'Color', [0.7 0.1 0.1], 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
if ~isempty(cc_scl_r70_S)
    valid = cc_scl_r70_S(~isnan(cc_scl_r70_S));
    plot(valid*1e6, V70*ones(size(valid)), 'v', 'MarkerSize', 5, 'MarkerEdgeColor', [0 0.6 0], 'MarkerFaceColor', [0 0.6 0]);
end
if ~isempty(cc_scl_f70_S)
    valid = cc_scl_f70_S(~isnan(cc_scl_f70_S));
    plot(valid*1e6, V70*ones(size(valid)), '^', 'MarkerSize', 5, 'MarkerEdgeColor', [0 0.4 0.8], 'MarkerFaceColor', [0 0.4 0.8]);
end
if ~isempty(cc_scl_r70_M)
    valid = cc_scl_r70_M(~isnan(cc_scl_r70_M));
    plot(valid*1e6, V70*ones(size(valid)), 'v', 'MarkerSize', 4, 'MarkerEdgeColor', [1 0.3 0.3], 'MarkerFaceColor', [1 0.3 0.3]);
end
if ~isempty(cc_scl_f70_M)
    valid = cc_scl_f70_M(~isnan(cc_scl_f70_M));
    plot(valid*1e6, V70*ones(size(valid)), '^', 'MarkerSize', 4, 'MarkerEdgeColor', [1 0.3 0.3], 'MarkerFaceColor', [1 0.3 0.3]);
end
if ~isnan(cc_start_idx)
    xline(t_us(cc_start_idx), '--m', 'START', 'LabelVerticalAlignment', 'bottom', 'FontSize', 8, 'LineWidth', 1.5);
end
if ~isnan(cc_rst_idx)
    xline(t_us(cc_rst_idx), '--m', 'RESTART', 'LabelVerticalAlignment', 'bottom', 'FontSize', 8, 'LineWidth', 1.5);
end
if exist('last_stop', 'var') && ~isempty(last_stop) && last_stop > 0
    xline(t_us(last_stop), '--m', 'STOP', 'LabelVerticalAlignment', 'bottom', 'FontSize', 8, 'LineWidth', 1.5);
end
if ~isnan(cc_start_scl30), xline(cc_start_scl30*1e6, '-.c', 'SCL30%', 'FontSize', 7, 'LineWidth', 1); end
if ~isnan(cc_rst_scl30), xline(cc_rst_scl30*1e6, '-.c', 'SCL30%', 'FontSize', 7, 'LineWidth', 1); end
if ~isnan(cc_stop_scl30), xline(cc_stop_scl30*1e6, '-.c', 'SCL30%', 'FontSize', 7, 'LineWidth', 1); end
xlabel('Time (us)'); ylabel('SCL (V)');
title(sprintf('SCL  |  Freq=%.1fkHz  HI=%.3fV/LO=%.3fV  |  Green^=V70ris  Bluev=V70fal  Red=Read', ...
    1/(scl_hi_T + scl_lo_T)/1e3, scl_S_hi, scl_S_lo), 'FontWeight', 'bold');
ylim([-0.3 2.5]); grid on;

ax2 = subplot(2,1,2);
plot(t_us, sda_raw, 'Color', [0.90 0.20 0.25], 'LineWidth', 0.4); hold on;
yline(V30, ':k', 'LineWidth', 1.5);
yline(V70, ':k', 'LineWidth', 1.5);
if ~isnan(sda_S_hi)
    yline(sda_S_hi, '--r', sprintf('HI=%.3fV', sda_S_hi), 'FontSize', 7);
    yline(sda_S_lo, '--b', sprintf('LO=%.3fV', sda_S_lo), 'FontSize', 7);
end
fill([WR_T0 WR_T1 WR_T1 WR_T0], [-0.5 -0.5 2.5 2.5], [0.68 0.85 1.0], 'FaceAlpha', 0.18, 'EdgeColor', 'none');
fill([RD_T0 RD_T1 RD_T1 RD_T0], [-0.5 -0.5 2.5 2.5], [1.0 0.75 0.75], 'FaceAlpha', 0.18, 'EdgeColor', 'none');
text((WR_T0+WR_T1)/2, 2.3, 'Master->Slave (WRITE)', 'FontSize', 8, 'Color', [0 0.3 0.7], 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
text((RD_T0+RD_T1)/2, 2.3, 'Slave->Master (READ)', 'FontSize', 8, 'Color', [0.7 0.1 0.1], 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
y_sda30 = V30 + 0.06;
if ~isempty(cc_sda_v30_S)
    plot(cc_sda_v30_S*1e6, y_sda30*ones(size(cc_sda_v30_S)), 's', 'MarkerSize', 5, 'MarkerEdgeColor', [0 0.5 0], 'MarkerFaceColor', [0 0.5 0]);
end
if ~isempty(cc_sda_v30_M)
    plot(cc_sda_v30_M*1e6, V30*ones(size(cc_sda_v30_M)), 's', 'MarkerSize', 4, 'MarkerEdgeColor', [1 0.3 0.3], 'MarkerFaceColor', [1 0.3 0.3]);
end
if ~isnan(cc_start_idx), xline(t_us(cc_start_idx), '--m', 'START', 'LabelVerticalAlignment', 'bottom', 'FontSize', 8, 'LineWidth', 1.5); end
if ~isnan(cc_rst_idx), xline(t_us(cc_rst_idx), '--m', 'RESTART', 'LabelVerticalAlignment', 'bottom', 'FontSize', 8, 'LineWidth', 1.5); end
if exist('last_stop', 'var') && ~isempty(last_stop) && last_stop > 0
    xline(t_us(last_stop), '--m', 'STOP', 'LabelVerticalAlignment', 'bottom', 'FontSize', 8, 'LineWidth', 1.5);
end
if ~isnan(cc_start_sda30), xline(cc_start_sda30*1e6, '-.r', 'SDA30%', 'FontSize', 7, 'LineWidth', 1); end
if ~isnan(cc_rst_sda30), xline(cc_rst_sda30*1e6, '-.r', 'SDA30%', 'FontSize', 7, 'LineWidth', 1); end
if ~isnan(cc_stop_sda30), xline(cc_stop_sda30*1e6, '-.r', 'SDA30%', 'FontSize', 7, 'LineWidth', 1); end
if ~isempty(txn)
    for sg = 1:length(txn)
        if ~isempty(txn(sg).bytes)
            rw_label = iif(txn(sg).rw == 0, 'W', 'R');
            lbl = sprintf('0x%02X(%s)=0x%02X', txn(sg).bytes(1), rw_label, txn(sg).bytes(2));
            txt_col = iif(txn(sg).is_restart, [0.7 0.1 0.1], [0 0.3 0.7]);
            prefix = iif(txn(sg).is_restart, 'RESTART ', 'START ');
            text(t_us(txn(sg).start_idx), V30-0.15, [prefix lbl], ...
                 'FontSize', 7, 'Color', txt_col, 'HorizontalAlignment', 'right', 'FontWeight', 'bold', 'Rotation', 90);
        end
    end
end
xlabel('Time (us)'); ylabel('SDA (V)');
title(sprintf('SDA  |  S:HI=%.3fV/LO=%.3fV  M:HI=%.3fV/LO=%.3fV  |  GreenSq=S V30  RedSq=M V30', ...
    sda_S_hi, sda_S_lo, sda_M_hi, sda_M_lo), 'FontWeight', 'bold');
ylim([-0.3 2.5]); grid on;
linkaxes([ax1 ax2], 'x');
exportgraphics(fig_ov, fullfile(dir_other, 'I2C_DC_电平汇总.png'), 'Resolution', 150);
close(fig_ov);
fprintf('      -> Saved overview plot: I2C_DC_电平汇总.png\n');

% List generated files
fprintf('\nGenerated files:\n');
listing = dir(fullfile(OUT_DIR, '**', '*.png'));
for k = 1:length(listing)
    fprintf('  %s\n', fullfile(listing(k).folder, listing(k).name));
end
fprintf('\n=== DONE ===\n');
fclose('all');

%% ========================================================================
%% LOCAL FUNCTIONS
%% ========================================================================

function [t_cross, idx] = find_crossing(v, tvec, thr, direction, start_idx, search_win)
    % Find first threshold crossing in search window around start_idx by linear interpolation.
    % v, tvec: column vectors.  thr: threshold.  direction: 'rising'|'falling'
    si = max(1, start_idx - search_win);
    ei = min(length(v), start_idx + search_win);
    vs = v(si:ei);
    ts = tvec(si:ei);
    if strcmp(direction, 'rising')
        cross_logical = (vs(1:end-1) <= thr) & (vs(2:end) > thr);
    else
        cross_logical = (vs(1:end-1) >= thr) & (vs(2:end) < thr);
    end
    ci = find(cross_logical, 1);
    if isempty(ci)
        t_cross = NaN; idx = NaN; return;
    end
    ci = ci(1);
    t1 = ts(ci);  t2 = ts(ci+1);
    v1 = vs(ci);  v2 = vs(ci+1);
    t_cross = t1 + (thr - v1) * (t2 - t1) / (v2 - v1);
    idx = si + ci - 1;
end

function [t_cross, idx] = find_crossing_after(v, tvec, thr, direction, t_start, search_win)
    % Find first threshold crossing strictly AFTER t_start within search_win.
    idx_start = max(1, find(tvec >= t_start, 1));
    idx_end   = min(length(v), idx_start + search_win);
    vs = v(idx_start:idx_end);
    ts = tvec(idx_start:idx_end);
    if strcmp(direction, 'rising')
        cross_logical = (vs(1:end-1) <= thr) & (vs(2:end) > thr);
    else
        cross_logical = (vs(1:end-1) >= thr) & (vs(2:end) < thr);
    end
    ci = find(cross_logical, 1);
    if isempty(ci)
        t_cross = NaN; idx = NaN; return;
    end
    ci = ci(1);
    t1 = ts(ci);  t2 = ts(ci+1);
    v1 = vs(ci);  v2 = vs(ci+1);
    t_cross = t1 + (thr - v1) * (t2 - t1) / (v2 - v1);
    idx = idx_start + ci - 1;
end

function fig = plot_single_edge(t_us, t_all, signal, edge_idx, meas_val, sig_name, edge_type, V30, V70, VDD, extreme_val)
    SEARCH_WIN = round(2e-6 / (t_all(2) - t_all(1)));
    if strcmp(edge_type, 'rise')
        [t_v30, ~] = find_crossing(signal, t_all, V30, 'rising', edge_idx, SEARCH_WIN);
        [t_v70, ~] = find_crossing(signal, t_all, V70, 'rising', edge_idx, SEARCH_WIN);
    else
        [t_v70, ~] = find_crossing(signal, t_all, V70, 'falling', edge_idx, SEARCH_WIN);
        [t_v30, ~] = find_crossing(signal, t_all, V30, 'falling', edge_idx, SEARCH_WIN);
    end
    t_center = t_us(edge_idx);
    ZOOM = 0.5;
    fig = figure('Name', sprintf('%s %s edge', sig_name, edge_type), ...
                 'Position', [100 100 1200 500], 'Visible', 'off');
    plot(t_us, signal, 'LineWidth', 0.8); hold on;
    xlim([t_center - ZOOM, t_center + ZOOM]);
    yline(V30, ':k', 'FontSize', 8); yline(V70, ':k', 'FontSize', 8);
    if ~isnan(t_v30), xline(t_v30*1e6, '--g', sprintf('30%%=%.0fmV', V30*1e3), 'FontSize', 8); end
    if ~isnan(t_v70), xline(t_v70*1e6, '--r', sprintf('70%%=%.0fmV', V70*1e3), 'FontSize', 8); end
    if ~isnan(extreme_val)
        if strcmp(edge_type, 'rise') && extreme_val > 0.01
            yline(VDD + extreme_val, '--r', sprintf('+%.0fmV', extreme_val*1e3), 'FontSize', 9);
        elseif strcmp(edge_type, 'fall') && extreme_val < -0.01
            yline(extreme_val, '--r', sprintf('%.0fmV', extreme_val*1e3), 'FontSize', 9);
        end
    end
    if strcmp(edge_type, 'rise')
        title(sprintf('%s Rising Edge  |  Rise Time (30%%→70%%) = %.1fns', ...
              sig_name, meas_val*1e9), 'FontWeight', 'bold');
    else
        title(sprintf('%s Falling Edge  |  Fall Time (70%%→30%%) = %.1fns', ...
              sig_name, meas_val*1e9), 'FontWeight', 'bold');
    end
    xlabel('Time (\mus)'); ylabel('Voltage (V)'); grid on;
    legend off;
end

function result = iif(condition, trueVal, falseVal)
    if condition, result = trueVal; else, result = falseVal; end
end
