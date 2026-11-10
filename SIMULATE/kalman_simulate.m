%% ========================================================================
%  KIỂM CHỨNG & SO SÁNH BỘ LỌC KALMAN GIỮA THỰC TẾ (STM32) VÀ MATLAB
%  Học phần: Bài tập lớn Nhập môn Điều khiển Thông minh (BTL_NMDKTM)
% =========================================================================
clear; clc; close all;

%% 1. ĐỌC DỮ LIỆU THỰC TẾ XUẤT TỪ GIAO DIỆN STM32 (data.csv)
filename = 'data.csv';
if ~isfile(filename)
    error('Không tìm thấy file %s! Hãy chắc chắn file CSV nằm cùng thư mục.', filename);
end

data = readtable(filename);

% Lấy đúng tên các cột từ file CSV thực tế:
time_s             = data.Time_s;
rawPitch_STM32     = data.RawPitch_NoFilter_deg;      % Cột góc đo thô từ Accelerometer
kalmanPitch_STM32  = data.KalmanPitch_Filtered_deg;   % Cột góc do STM32F4 tính ra bằng Kalman
gyroRate_STM32     = data.GyroRate_degs;              % Cột vận tốc góc Gyro
noiseDiff_STM32    = data.Noise_Difference_deg;       % Độ chênh lệch [Raw - Filtered]

N = length(time_s);
fprintf('Đã tải thành công %d mẫu dữ liệu từ file %s.\n\n', N, filename);

%% 2. CHẠY LẠI THUẬT TOÁN KALMAN TRÊN MATLAB BẰNG CÔNG THỨC LÝ THUYẾT
% Tham số bộ lọc (Giống hệt cấu hình trong kalman.c)
Q_angle   = 0.001;
Q_bias    = 0.003;
R_measure = 0.03;

% Biến trạng thái mô phỏng trên MATLAB
kalmanPitch_MATLAB = zeros(N, 1);
kalmanBias_MATLAB  = zeros(N, 1);
P = zeros(2, 2);

% --- First Sample: Khởi tạo góc bắt đầu giống hệt STM32 ---
kalmanPitch_MATLAB(1) = rawPitch_STM32(1);
kalmanBias_MATLAB(1)  = 0;

for k = 2:N
    dt = time_s(k) - time_s(k-1);
    if dt <= 0 || dt > 0.2
        dt = 0.05; % Chu kỳ chuẩn nếu có khoảng trống
    end
    
    newAngle = rawPitch_STM32(k);
    newRate  = gyroRate_STM32(k);
    
    % --- BƯỚC 1: DỰ ĐOÁN (PREDICT) ---
    rate = newRate - kalmanBias_MATLAB(k-1);
    angle_pred = kalmanPitch_MATLAB(k-1) + dt * rate;
    
    P(1,1) = P(1,1) + dt * (dt * P(2,2) - P(1,2) - P(2,1) + Q_angle);
    P(1,2) = P(1,2) - dt * P(2,2);
    P(2,1) = P(2,1) - dt * P(2,2);
    P(2,2) = P(2,2) + Q_bias * dt;
    
    % --- BƯỚC 2: CẬP NHẬT ĐO LƯỜNG (CORRECT / UPDATE) ---
    S = P(1,1) + R_measure;
    K0 = P(1,1) / S;
    K1 = P(2,1) / S;
    
    y = newAngle - angle_pred;
    
    kalmanPitch_MATLAB(k) = angle_pred + K0 * y;
    kalmanBias_MATLAB(k)  = kalmanBias_MATLAB(k-1) + K1 * y;
    
    P00_temp = P(1,1);
    P01_temp = P(1,2);
    P(1,1) = P(1,1) - K0 * P00_temp;
    P(1,2) = P(1,2) - K0 * P01_temp;
    P(2,1) = P(2,1) - K1 * P00_temp;
    P(2,2) = P(2,2) - K1 * P01_temp;
end

%% 3. TÍNH TOÁN CÁC CHỈ SỐ ĐỊNH LƯỢNG (PHỤC VỤ BÁO CÁO BTL)
% 1. Đánh giá mức độ làm mượt (Độ lệch chuẩn của đạo hàm góc)
std_raw    = std(diff(rawPitch_STM32));
std_kalman = std(diff(kalmanPitch_STM32));
noise_reduction_pct = (1 - std_kalman / std_raw) * 100;

% 2. Độ tương đồng giữa STM32 và MATLAB
corr_coef = corrcoef(kalmanPitch_STM32, kalmanPitch_MATLAB);
similarity_pct = corr_coef(1,2) * 100;

fprintf('================ KẾT QUẢ ĐÁNH GIÁ ĐỊNH LƯỢNG ================\n');
fprintf('• Độ biến động tín hiệu thô (Không lọc):      %.3f\n', std_raw);
fprintf('• Độ biến động sau khi lọc Kalman (STM32):    %.3f\n', std_kalman);
fprintf('• HIỆU QUẢ KHỬ NHIỄU CỦA BỘ LỌC KALMAN:       %.1f %%\n', noise_reduction_pct);
fprintf('• ĐỘ TƯƠNG ĐỒNG GIỮA CODE STM32 VÀ LÝ THUYẾT: %.2f %%\n', similarity_pct);
fprintf('=============================================================\n');

%% 4. VẼ ĐỒ THỊ SO SÁNH CHI TIẾT
figure('Name', 'So sánh Bộ lọc Kalman STM32 vs MATLAB', 'Color', [1 1 1], 'Position', [100 100 1000 650]);

% Đồ thị 1: So sánh góc thô và góc lọc
subplot(2,1,1);
plot(time_s, rawPitch_STM32,    'Color', [0.85 0.35 0.45], 'LineWidth', 0.9); hold on;
plot(time_s, kalmanPitch_STM32, 'Color', [0.00 0.65 0.85], 'LineWidth', 2.0);
plot(time_s, kalmanPitch_MATLAB,'Color', [0.10 0.10 0.10], 'LineStyle', '--', 'LineWidth', 1.2);
grid on;
title('So sánh: Góc thô (Không lọc) vs Kalman STM32 vs Kalman MATLAB', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Thời gian (giây)');
ylabel('Góc nghiêng Pitch (độ)');
legend('Góc thô Accelerometer (Không lọc)', ...
       'Góc lọc Kalman trên STM32 (Thực tế nhúng)', ...
       'Góc lọc Kalman trên MATLAB (Mô phỏng lý thuyết)', ...
       'Location', 'best');

% Đồ thị 2: Vận tốc góc Gyroscope và phần dư sai lệch
subplot(2,1,2);
yyaxis left;
plot(time_s, gyroRate_STM32, 'Color', [0.30 0.70 0.30], 'LineWidth', 1.2);
ylabel('Vận tốc góc Gyro (°/s)');
grid on;

yyaxis right;
plot(time_s, noiseDiff_STM32, 'Color', [0.90 0.50 0.10], 'LineWidth', 0.8);
ylabel('Nhiễu tức thời |Raw - Filtered| (độ)');
title('Vận tốc góc Gyro và Biên độ nhiễu bị triệt tiêu', 'FontSize', 11, 'FontWeight', 'bold');
xlabel('Thời gian (giây)');
legend('Vận tốc góc Gyro (°/s)', 'Biên độ nhiễu đã bị lọc bỏ (độ)', 'Location', 'best');