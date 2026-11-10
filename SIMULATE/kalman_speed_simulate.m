%% ========================================================================
%  MÔ PHỎNG & KIỂM CHỨNG BỘ LỌC KALMAN VẬN TỐC BÁNH XE (WHEEL SPEED KALMAN FILTER)
%  Học phần: Bài tập lớn Nhập môn Điều khiển Thông minh (BTL_NMDKTM)
% =========================================================================
clear; clc; close all;

%% 1. THIẾT LẬP THÔNG SỐ HỆ THỐNG & ĐỘNG CƠ JGB37-520
dt = 0.010;              % Chu kỳ lấy mẫu 10ms (100 Hz) trong vTaskEncoder
T_total = 4.0;           % Thời gian mô phỏng: 4 giây
t = 0:dt:T_total;
N = length(t);

% Thông số Encoder: 330 xung/vòng, chế độ 4x -> 1320 xung/vòng
% Độ phân giải lượng tử hóa tại dt = 0.010s:
% delta_speed = 60 / (1320 * 0.010) = 4.54545 RPM / xung
counts_per_rpm = (1320 * dt) / 60.0;
quant_step_rpm = 1.0 / counts_per_rpm;

fprintf('=== THÔNG SỐ ĐỘNG CƠ & ENCODER ===\n');
fprintf('Chu kỳ lấy mẫu dt:        %.3f s (%.0f Hz)\n', dt, 1/dt);
fprintf('Độ phân giải lượng tử:    %.3f RPM / xung\n\n', quant_step_rpm);

%% 2. TẠO TÍN HIỆU VẬN TỐC THỰC TẾ (GROUND TRUTH PROFILE)
% Quá trình vận hành điển hình của xe:
% - 0.0s -> 0.5s: Xe đứng yên (0 RPM)
% - 0.5s -> 1.5s: Tăng tốc tuyến tính lên 60 RPM (a = 60 RPM/s)
% - 1.5s -> 2.5s: Chạy đều ở 60 RPM
% - 2.5s -> 3.2s: Hãm tốc đột ngột xuống -30 RPM (đổi chiều quay)
% - 3.2s -> 4.0s: Chạy đều ở -30 RPM
v_true = zeros(N, 1);
a_true = zeros(N, 1);

for k = 1:N
    if t(k) < 0.5
        v_true(k) = 0.0;
        a_true(k) = 0.0;
    elseif t(k) < 1.5
        v_true(k) = 60.0 * (t(k) - 0.5);
        a_true(k) = 60.0;
    elseif t(k) < 2.5
        v_true(k) = 60.0;
        a_true(k) = 0.0;
    elseif t(k) < 3.2
        v_true(k) = 60.0 - (90.0 / 0.7) * (t(k) - 2.5);
        a_true(k) = -90.0 / 0.7;
    else
        v_true(k) = -30.0;
        a_true(k) = 0.0;
    end
end

%% 3. MÔ PHỎNG QUÁ TRÌNH LƯỢNG TỬ HÓA TỪ PHẦN CỨNG ENCODER (RAW SPEED)
raw_speed = zeros(N, 1);
pulse_accum = 0.0;

for k = 1:N
    pulse_accum = pulse_accum + v_true(k) * counts_per_rpm;
    pulses = round(pulse_accum);
    pulse_accum = pulse_accum - pulses;
    raw_speed(k) = pulses / counts_per_rpm;
end

%% 4. THUẬT TOÁN 1: BỘ LỌC THÔNG THẤP IIR BẬC 1 (LPF)
alpha_lpf = 0.70;
v_lpf = zeros(N, 1);
for k = 2:N
    v_lpf(k) = alpha_lpf * v_lpf(k-1) + (1.0 - alpha_lpf) * raw_speed(k);
end

%% 5. THUẬT TOÁN 2: BỘ LỌC KALMAN 2 CHIỀU (SPEED & ACCELERATION)
% Tham số cấu hình giống hệt trong kalman.c:
Q_speed   = 0.05;   % Phương sai nhiễu mô hình vận tốc
Q_accel   = 50.0;   % Phương sai nhiễu mô hình gia tốc
R_measure = 5.0;    % Phương sai nhiễu lượng tử hóa đo lường từ encoder

v_kalman = zeros(N, 1);
a_kalman = zeros(N, 1);
P = eye(2);         % Khởi tạo hiệp phương sai sai số

% Chạy từng chu kỳ 10ms giống STM32:
for k = 2:N
    % --- BƯỚC 1: DỰ ĐOÁN (PREDICT) ---
    v_pred = v_kalman(k-1) + a_kalman(k-1) * dt;
    a_pred = a_kalman(k-1);
    
    % P^- = F * P * F' + Q, với F = [1 dt; 0 1]
    p00_temp = P(1,1) + dt * (2.0 * P(1,2) + dt * P(2,2)) + Q_speed;
    p01_temp = P(1,2) + dt * P(2,2);
    p11_temp = P(2,2) + Q_accel;
    
    % --- BƯỚC 2: CẬP NHẬT ĐO LƯỜNG (CORRECT / UPDATE) ---
    S = p00_temp + R_measure;
    K0 = p00_temp / S;
    K1 = p01_temp / S;
    
    y = raw_speed(k) - v_pred;
    
    v_kalman(k) = v_pred + K0 * y;
    a_kalman(k) = a_pred + K1 * y;
    
    % Cập nhật ma trận P = (I - K*H) * P^-
    P(1,1) = (1.0 - K0) * p00_temp;
    P(1,2) = (1.0 - K0) * p01_temp;
    P(2,1) = P(1,2);
    P(2,2) = p11_temp - K1 * p01_temp;
end

%% 6. ĐÁNH GIÁ CHỈ SỐ ĐỊNH LƯỢNG (PHỤC VỤ BÁO CÁO)
% 1. Đánh giá sai số RMSE ở trạng thái xác lập (từ 1.7s đến 2.4s):
ss_idx = (t >= 1.7) & (t <= 2.4);
rmse_raw    = std(raw_speed(ss_idx) - v_true(ss_idx));
rmse_lpf    = std(v_lpf(ss_idx)     - v_true(ss_idx));
rmse_kalman = std(v_kalman(ss_idx)  - v_true(ss_idx));

% 2. Đánh giá độ trễ pha khi tăng tốc (từ 0.6s đến 1.4s):
ramp_idx = (t >= 0.6) & (t <= 1.4);
lag_lpf    = mean(v_true(ramp_idx) - v_lpf(ramp_idx));
lag_kalman = mean(v_true(ramp_idx) - v_kalman(ramp_idx));

fprintf('=== KẾT QUẢ ĐÁNH GIÁ ĐỊNH LƯỢNG ===\n');
fprintf('1. Mức độ khử nhiễu lượng tử (RMSE ở 60 RPM):\n');
fprintf('   - Raw Encoder:  %.3f RPM\n', rmse_raw);
fprintf('   - LPF (alpha=0.70): %.3f RPM (Giảm %.1f%% nhiễu)\n', rmse_lpf, (1 - rmse_lpf/rmse_raw)*100);
fprintf('   - Kalman Filter:    %.3f RPM (Giảm %.1f%% nhiễu)\n\n', rmse_kalman, (1 - rmse_kalman/rmse_raw)*100);

fprintf('2. Độ trễ quá độ khi tăng tốc (Mean Lag):\n');
fprintf('   - LPF:          %.3f RPM (Có trễ pha do hằng số thời gian tau)\n', lag_lpf);
fprintf('   - Kalman:       %.3f RPM (Gần như triệt tiêu hoàn toàn trễ pha!)\n', lag_kalman);

%% 7. VẼ ĐỒ THỊ SO SÁNH TRỰC QUAN
figure('Name', 'So sánh Bộ lọc Vận tốc Bánh xe', 'Position', [100, 100, 1050, 680]);

% Đồ thị 1: Toàn bộ quá trình vận hành
subplot(3, 1, 1);
plot(t, raw_speed, 'Color', [0.8 0.8 0.8], 'LineWidth', 1.0, 'DisplayName', 'Raw Encoder (Lượng tử hóa)');
hold on;
plot(t, v_true, 'k--', 'LineWidth', 1.5, 'DisplayName', 'True Speed (Lý tưởng)');
plot(t, v_lpf, 'r-', 'LineWidth', 1.5, 'DisplayName', 'LPF IIR (alpha=0.70)');
plot(t, v_kalman, 'b-', 'LineWidth', 1.8, 'DisplayName', 'Kalman Filter (STM32)');
grid on;
ylabel('Vận tốc (RPM)', 'FontSize', 11);
title('SO SÁNH BỘ LỌC VẬN TỐC 2 BÁNH XE GIỮA RAW, LPF VÀ KALMAN FILTER', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 10);

% Đồ thị 2: Phóng to vùng tăng tốc (chứng minh Zero Phase Lag của Kalman)
subplot(3, 1, 2);
zoom_idx = (t >= 0.4) & (t <= 1.6);
plot(t(zoom_idx), raw_speed(zoom_idx), 'Color', [0.75 0.75 0.75], 'LineWidth', 1.0);
hold on;
plot(t(zoom_idx), v_true(zoom_idx), 'k--', 'LineWidth', 1.5);
plot(t(zoom_idx), v_lpf(zoom_idx), 'r-', 'LineWidth', 1.5);
plot(t(zoom_idx), v_kalman(zoom_idx), 'b-', 'LineWidth', 1.8);
grid on;
ylabel('Vận tốc (RPM)', 'FontSize', 11);
title('Phóng to vùng tăng tốc: Kalman bám sát True Speed (Zero Phase Lag), LPF bị trễ', 'FontSize', 11);

% Đồ thị 3: Gia tốc ước lượng từ bộ lọc Kalman
subplot(3, 1, 3);
plot(t, a_true, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Gia tốc thực');
hold on;
plot(t, a_kalman, 'm-', 'LineWidth', 1.5, 'DisplayName', 'Gia tốc ước lượng (Kalman Accel)');
grid on;
xlabel('Thời gian (giây)', 'FontSize', 11);
ylabel('Gia tốc (RPM/s)', 'FontSize', 11);
title('Ước lượng Gia tốc động cơ từ bộ lọc Kalman (State 2)', 'FontSize', 11);
legend('Location', 'northeast', 'FontSize', 10);

