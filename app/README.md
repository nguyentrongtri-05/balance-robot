# GIAO DIỆN ĐIỀU KHIỂN & GIÁM SÁT ROBOT CÂN BẰNG (STM32F4 + FreeRTOS)

Ứng dụng giao diện đồ họa (GUI) viết bằng **Python (Tkinter + Matplotlib + PySerial)** được thiết kế riêng để giao tiếp 2 chiều với firmware STM32F4 chạy FreeRTOS qua chuẩn UART DMA (20Hz / 50ms).

---

## 1. Hướng Dẫn Khởi Chạy Nhanh

### Cách 1: Nhấp đúp chuột (Khuyên dùng)
- Vào thư mục `app`, nhấp đúp vào file **`run.bat`**.
- Script sẽ tự động kiểm tra thư viện và khởi động giao diện.

### Cách 2: Chạy từ Terminal / PowerShell
```bash
cd e:\NMDKTMN\BTL\app
python gui_app.py
```

> **Lưu ý:** Nếu chưa cài thư viện, chạy:
> ```bash
> pip install -r requirements.txt
> ```

---

## 2. Kết Nối Phần Cứng Với Máy Tính

Sử dụng mạch chuyển đổi **USB-UART** (như CP2102, CH340, FT232) nối với cổng **USART3** của STM32F4:

| Chân trên STM32F4 | Chân Module USB-UART | Ghi chú |
| :--- | :--- | :--- |
| **PD8 (USART3 TX)** | **RX** | Nhận dữ liệu Telemetry 20Hz từ STM32 |
| **PD9 (USART3 RX)** | **TX** | Gửi lệnh Start, Stop, Calib, Setpoint |
| **GND** | **GND** | Nối chung mass (bắt buộc) |

- **Baudrate:** `115200`
- **Data Bits:** `8`
- **Parity:** `None`
- **Stop Bits:** `1`

---

## 3. Các Tính Năng Chính Của Giao Diện

1. **Quản lý kết nối & Chế độ Demo:**
   - Tự động quét danh sách cổng COM khi nhấn nút `🔄`.
   - Chọn tốc độ Baud (mặc định 115200).
   - Đèn LED chỉ thị trạng thái kết nối và bộ đếm tần số nhận gói tin (Hz).
   - **Chế độ Mô phỏng (Demo Mode):** Tích chọn `🎮 Chế độ Mô phỏng (Demo)` để kiểm tra giao diện, đồ thị và các nút bấm ngay cả khi chưa cắm kit phần cứng!

2. **Khung điều khiển hoạt động nhanh:**
   - **▶ START (Chạy):** Gửi gói ID `0x01` và ký tự `'r'` cho phép xe bật cân bằng.
   - **⏹ STOP (Dừng khẩn cấp):** Gửi gói ID `0x02` và ký tự `'s'` ngắt lập tức PWM 2 động cơ.
   - **⚖ CALIB (Cân chỉnh tự động):** Gửi gói ID `0x03` và ký tự `'c'` để xe tự đo gyro offset và góc 0 điểm cân bằng.

3. **Cài đặt thông số mục tiêu (Setpoint):**
   - **Góc mục tiêu (Target Angle):** Kéo thanh trượt hoặc nhập góc (-15° đến +15°) rồi nhấn "Gửi" (ID `0x04`). Có nút bấm về nhanh `0°`.
   - **Vận tốc mục tiêu (Target Speed):** Kéo thanh trượt hoặc nhập tốc độ (-50 đến +50) rồi nhấn "Gửi" (ID `0x05`).

4. **Bảng thẻ đo đạc tức thời (So sánh bộ lọc - 20Hz):**
   - **Góc thô (Raw Pitch):** Góc đọc trực tiếp từ gia tốc kế (KHÔNG BỘ LỌC), thấy rõ rung động cơ học.
   - **Góc lọc (Kalman Pitch):** Góc sau lọc Kalman mượt mà (CÓ BỘ LỌC).
   - **Độ lệch nhiễu (|Raw - Kalman|):** Biên độ nhiễu đã được bộ lọc Kalman triệt tiêu.
   - **Vận tốc góc (Gyro Rate):** Tốc độ xoay góc (°/s).
   - **PWM Động cơ (PWM Output):** Phần trăm công suất đang xuất (-100% đến +100%).
   - **Tốc độ xe (Encoder Speed):** Vận tốc phản hồi từ Encoder bánh xe.

5. **Tab 1: Robot Cân Bằng & Bộ Lọc Kalman (3 Subplots):**
   - **Đồ thị 1 (So sánh bộ lọc):** Vẽ đồng thời `Raw Pitch (Không bộ lọc - Đỏ)`, `Kalman Filter (Có bộ lọc - Xanh cyan)` và `Target Angle (Mục tiêu - Vàng đứt nét)`.
   - **Đồ thị 2 (Vận tốc góc):** Giám sát `Gyro Rate (°/s)`.
   - **Đồ thị 3 (Đáp ứng động cơ):** Theo dõi `Motor PWM (%)` và `Motor Speed`.

6. **Tab 2: Giám Sát Đáp Ứng PID Động Cơ (Chuyên sâu Test Step Response):**
   - **Thử nghiệm Bước Nhảy (Step Test):** Các nút kích xung nhanh (`0`, `+10`, `+20`, `+30`, `+45`, `-25`).
   - **Bộ phát xung vuông tuần hoàn (Auto Square Wave):** Tự động phát xung đảo chiều `+A -> -A` theo chu kỳ định trước (ví dụ 3 giây) để quan sát quá trình quá độ (transient) liên tục.
   - **Đồ thị Đáp ứng Vận tốc (Speed Response Plot):** Vẽ đường bậc thang `Vận tốc đặt r(t)`, đường cong đáp ứng thực tế `Vận tốc thực y(t)`, và đường `Sai số e(t) = r - y`.
   - **Đồ thị PWM Ngõ ra (Control Effort):** Quan sát điện áp điều khiển xuất ra động cơ và kiểm tra hiện tượng bão hòa PWM (Saturation).
   - **Phân tích chỉ số PID:** Tính toán trực tiếp sai số xác lập ($e_{ss}$) và độ vọt lố quan sát ($M_p$ %).
   - **Cài đặt Online PID Tuning (ID 0x06):** Thay đổi trực tiếp $K_p$, $K_i$, $K_d$ từ giao diện và nạp xuống STM32 tức thì.

7. **Xuất dữ liệu ra file CSV (Phục vụ làm Báo cáo BTL):**
   - **Xuất CSV Cân Bằng:** Lưu `Time_s, RawPitch, FilteredPitch, Noise, GyroRate, Speed, PWM, TargetAngle`.
   - **Xuất CSV Test PID:** Lưu `Time_s, Target_Speed_r, Actual_Speed_y, Speed_Error_e, Motor_PWM_Output_pct` để tính thời gian lên ($T_r$), thời gian xác lập ($T_s$) và độ vọt lố ($M_p$).

8. **UART Terminal / Debug Log:**
   - Hỗ trợ hiển thị phản hồi text và tự động giải mã cả chế độ chuỗi CSV `UART_TELEMETRY_MODE == 1` (`Raw_NoFilter:...,Kalman_Filtered:...`).

---

## 4. Định Nghĩa Giao Thức Gói Tin UART

Cấu trúc khung dữ liệu:
```
[0xAA] [ID] [len] [Data...] [Checksum] [0xFF]
```
- **Header:** `0xAA` (1 byte)
- **ID:** Mã lệnh hoặc mã dữ liệu (1 byte)
- **len:** Số byte của trường Data (1 byte)
- **Data:** Payload chứa dữ liệu nhị phân (Little-Endian)
- **Checksum:** Tính theo bitwise OR: `cs = ID | len | Data[0] | Data[1] | ...`
- **Tail:** `0xFF` (1 byte)

### A. Gói Telemetry từ STM32 gửi lên PC (Chu kỳ 50ms):
- **ID:** `0x01`
- **len:** `0x20` (32 byte = 8 số float 32-bit little-endian)
- **Data:**
  1. `rawPitch` (float, 4 byte): Góc thô gia tốc kế (KHÔNG BỘ LỌC)
  2. `filteredPitch` (float, 4 byte): Góc sau lọc Kalman (CÓ BỘ LỌC)
  3. `gyroRate` (float, 4 byte): Vận tốc góc thực tế (độ/s)
  4. `motorSpeed_left` (float, 4 byte): Vận tốc thực tế Bánh Trái từ Encoder
  5. `motorSpeed_right` (float, 4 byte): Vận tốc thực tế Bánh Phải từ Encoder
  6. `motorPWM_left` (float, 4 byte): PWM Bánh Trái (-100 đến 100 %)
  7. `motorPWM_right` (float, 4 byte): PWM Bánh Phải (-100 đến 100 %)
  8. `targetAngle` (float, 4 byte): Điểm cân bằng mục tiêu (độ)

### B. Lệnh từ PC gửi xuống STM32:
- **0x01 (START):** `AA 01 00 01 FF`
- **0x02 (STOP):** `AA 02 00 02 FF`
- **0x03 (CALIB):** `AA 03 00 03 FF`
- **0x04 (SET TARGET ANGLE):** `AA 04 04 [float 4B] [CS] FF`
- **0x05 (SET TARGET SPEED 2 BÁNH):** `AA 05 08 [Target_L (4B)][Target_R (4B)] [CS] FF` (hoặc 4B chung)
- **0x06 (SET PID 2 BÁNH):** `AA 06 18 [Kp_L][Ki_L][Kd_L][Kp_R][Ki_R][Kd_R] [CS] FF` (hoặc 12B chung)

