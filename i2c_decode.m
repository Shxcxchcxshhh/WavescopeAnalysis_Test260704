%% I2C Decoder for ZDS3024 Plus Oscilloscope Export
% CH1 = SDA, CH2 = SCL  (verified by START condition detection)

clear; clc;

%% Load
filename = '996D-CWWB-MAIN1.csv';
fprintf('Loading %s ...\n', filename);
opts = detectImportOptions(filename, 'NumHeaderLines', 22);
data = readmatrix(filename, opts);
t_all = data(:,1);
sda_all = data(:,2);
scl_all = data(:,3);

fprintf('  Total: %d samples, %.1f us to %.1f us\n', ...
    length(t_all), t_all(1)*1e6, t_all(end)*1e6);

%% Focus window
t_start = -1e-6;
t_end   = 150e-6;
idx = (t_all >= t_start) & (t_all <= t_end);
t = t_all(idx);
sda = sda_all(idx);
scl = scl_all(idx);

%% Threshold
idle_v = max(sda(1:20));
threshold = idle_v * 0.5;
fprintf('  Idle: %.2fV, Threshold: %.2fV\n', idle_v, threshold);

sda_d = double(sda < threshold);
scl_d = double(scl < threshold);

%% Find SCL rising edges and filter glitches (min 1us between edges)
scl_diff = diff(scl_d);
raw_rising = find(scl_diff > 0.5) + 1;

scl_rising = raw_rising(1);
for i = 2:length(raw_rising)
    if t(raw_rising(i)) - t(scl_rising(end)) > 1e-6
        scl_rising = [scl_rising; raw_rising(i)];
    end
end

fprintf('  SCL rising: %d raw -> %d clean\n', length(raw_rising), length(scl_rising));

%% Detect START: SDA falls while SCL is high
sda_diff = diff(sda_d);
sda_falling = find(sda_diff < -0.5) + 1;
sda_rising  = find(sda_diff > 0.5) + 1;

start_idx = [];
for i = 1:length(sda_falling)
    si = sda_falling(i);
    if si > 2 && scl_d(si-1) > 0.5
        start_idx = [start_idx; si];
    end
end

stop_idx = [];
for i = 1:length(sda_rising)
    si = sda_rising(i);
    if si > 2 && scl_d(si-1) > 0.5
        stop_idx = [stop_idx; si];
    end
end

fprintf('  START: %d, STOP: %d\n', length(start_idx), length(stop_idx));

%% Take SCL rising edges after START
if isempty(start_idx)
    valid_rising = scl_rising;
    fprintf('  WARNING: No START condition detected\n');
else
    valid_rising = scl_rising(scl_rising >= start_idx(1));
end

%% Decode bits (SDA value at each SCL rising edge)
bits = [];
bit_times = [];
for i = 1:length(valid_rising)
    ri = valid_rising(i);
    bits = [bits; sda_d(ri)];
    bit_times = [bit_times; t(ri)*1e6];
end

fprintf('  Bits extracted: %d\n', length(bits));

%% Display bits
if length(bits) > 0
    fprintf('  Bit stream: ');
    for i = 1:min(60, length(bits))
        if mod(i,9)==1 && i>1
            fprintf(' ');
        end
        fprintf('%d', bits(i));
    end
    fprintf('\n');
end

%% Decode bytes
num_bytes = floor(length(bits) / 9);
fprintf('\n========== I2C Decode ==========\n\n');

if num_bytes >= 1
    for b = 1:num_bytes
        bp = (b-1)*9 + 1;
        byte_bits = bits(bp:bp+7);
        ack = bits(bp+8);

        val = 0;
        for k = 1:8
            val = val + byte_bits(9-k) * 2^(k-1);
        end

        bitstr = sprintf('%d%d%d%d%d%d%d%d', byte_bits(1), byte_bits(2), ...
            byte_bits(3), byte_bits(4), byte_bits(5), byte_bits(6), ...
            byte_bits(7), byte_bits(8));

        fprintf('  Byte %d: 0x%02X (%3d) [%s] %s @ %.3f us\n', ...
            b, val, val, bitstr, iif(ack==0,'ACK ','NACK'), bit_times(bp));
    end

    %% Interpretation
    fprintf('\n--- Interpretation ---\n');
    addr = 0;
    for k = 1:8
        addr = addr + bits(9-k) * 2^(k-1);
    end
    addr7 = bitshift(addr, -1);
    rw = bitand(addr, 1);
    rw_str = 'WRITE';
    if rw == 1
        rw_str = 'READ';
    end
    fprintf('  Address: 0x%02X (7-bit: 0x%02X), %s\n', addr, addr7, rw_str);

    if num_bytes >= 2
        fprintf('  Data:    ');
        for b = 2:num_bytes
            dv = 0; bp = (b-1)*9 + 1;
            for k = 1:8
                dv = dv + bits(bp+7-k) * 2^(k-1);
            end
            fprintf('0x%02X ', dv);
        end
        fprintf('\n');

        chars = '';
        for b = 2:num_bytes
            dv = 0; bp = (b-1)*9 + 1;
            for k = 1:8
                dv = dv + bits(bp+7-k) * 2^(k-1);
            end
            if dv >= 32 && dv <= 126
                chars(end+1) = char(dv);
            else
                chars(end+1) = '.';
            end
        end
        fprintf('  ASCII:   "%s"\n', chars);
    end
elseif length(bits) > 0
    fprintf('  Partial: %d bits (need %d more for complete byte)\n', ...
        length(bits), 9 - mod(length(bits),9));
end

%% Clock analysis
if length(bit_times) >= 2
    periods = diff(bit_times);
    periods = periods(periods > 1);
    fprintf('\n  SCL period: %.3f us (%.1f kHz)\n', ...
        mean(periods), 1000/mean(periods));
end

%% Plot
figure('Name', 'I2C Decode', 'Position', [50, 50, 1500, 900]);

subplot(4,1,1);
plot(t*1e6, scl, 'b-', 'LineWidth', 0.6);
hold on;
plot(t*1e6, sda, 'r-', 'LineWidth', 0.6);
yline(threshold, 'k--', 'LineWidth', 1);
xlabel('Time (us)'); ylabel('Voltage (V)');
title(sprintf('I2C Bus: SCL (CH2/Blue) & SDA (CH1/Red), Threshold=%.2fV', threshold));
legend('SCL (CH2)', 'SDA (CH1)', 'Threshold', 'Location','best');
grid on;

subplot(4,1,2);
stairs(t*1e6, scl_d*0.9+0.05, 'b-', 'LineWidth', 1);
hold on;
stairs(t*1e6, sda_d*0.9-0.05, 'r-', 'LineWidth', 1);
if ~isempty(valid_rising)
    stem(t(valid_rising)*1e6, ones(size(valid_rising))*1.05, 'k.', 'MarkerSize', 4);
end
xlabel('Time (us)'); ylabel('Logic');
title('Digital: SCL (Blue) & SDA (Red) with SCL rising edges');
ylim([-0.15, 1.15]);
grid on;

subplot(4,1,3);
plot(t*1e6, sda, 'r-', 'LineWidth', 0.5);
hold on;
if ~isempty(start_idx)
    xline(t(start_idx)*1e6, 'g--', 'START', 'LineWidth', 1.5);
end
if ~isempty(stop_idx)
    xline(t(stop_idx)*1e6, 'm--', 'STOP', 'LineWidth', 1.5);
end
plot(t(valid_rising)*1e6, sda(valid_rising), 'k.', 'MarkerSize', 8);
xlabel('Time (us)'); ylabel('Voltage (V)');
title('SDA with START/STOP & Sampling Points');
grid on;

subplot(4,1,4);
plot(t*1e6, sda, 'Color', [0.7 0 0], 'LineWidth', 0.5);
hold on;
if num_bytes > 0
    for b = 1:num_bytes
        bp = (b-1)*9 + 1;
        byte_bits_plot = bits(bp:bp+7);
        dv = 0;
        for k = 1:8
            dv = dv + byte_bits_plot(9-k) * 2^(k-1);
        end
        text(bit_times(bp), max(sda)*0.55, sprintf('0x%02X', dv), ...
            'Rotation', 90, 'FontSize', 10, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center', 'Color', [0 0 0.5]);
    end
end
xlabel('Time (us)'); ylabel('Voltage (V)');
title('Decoded Bytes on SDA');
grid on;

%% Zoomed view
figure('Name', 'I2C Decode Zoom', 'Position', [50, 50, 1500, 600]);
t_us = t*1e6;

subplot(2,1,1);
plot(t_us, scl, 'b-', 'LineWidth', 1);
hold on;
plot(t_us, sda, 'r-', 'LineWidth', 1);
yline(threshold, 'k--');
xlabel('Time (us)'); ylabel('Voltage (V)');
title('I2C Bus Zoom');
legend('SCL', 'SDA', 'Threshold');
grid on;
% Zoom to first 20us
if length(bit_times) >= 9
    xlim([-1, max(bit_times(1:9))*1.1]);
end

subplot(2,1,2);
stairs(t_us, scl_d, 'b-', 'LineWidth', 1);
hold on;
stairs(t_us, sda_d, 'r-', 'LineWidth', 1);
xlabel('Time (us)'); ylabel('Logic');
title('Digital Zoom');
legend('SCL', 'SDA');
if length(bit_times) >= 9
    xlim([-1, max(bit_times(1:9))*1.1]);
end
ylim([-0.1, 1.2]);
grid on;

fprintf('\nDone.\n');

%% Helper
function result = iif(condition, trueVal, falseVal)
    if condition
        result = trueVal;
    else
        result = falseVal;
    end
end
