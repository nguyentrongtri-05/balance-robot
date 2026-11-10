# Dự Án Robot Hai Bánh Tự Cân Bằng (TWIP) — Toàn Bộ Hệ Thống Cơ Điện Tử & Điều Khiển

[![MCU](https://img.shields.io/badge/MCU-STM32F407VGT6%20(168MHz)-03234B?logo=stmicroelectronics)](https://www.st.com/en/microcontrollers-microprocessors/stm32f407vg.html)
[![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS%20v11.1%20(200Hz)-007ACC)](https://www.freertos.org/)
[![Controller](https://img.shields.io/badge/Control-Cascaded%20Dual--FuzzyPD%20%2B%20Kalman-2E7D32)](./Firmware/README.md)
[![Paper](https://img.shields.io/badge/PDF-Lưu%20Đồ%20%26%20Tổng%20Hợp%20Giải%20Thuật-B71C1C)](./Firmware/luu_do_dieu_khien_fuzzy.pdf)

Repository lưu trữ **toàn bộ dự án Robot hai bánh tự cân bằng (Two-Wheeled Inverted Pendulum — TWIP)** từ thiết kế cơ khí 3D, thiết kế mạch in PCB, mô phỏng động lực học MATLAB/Simulink, phần mềm giám sát/tinh chỉnh trực tuyến trên máy tính (Python GUI) đến mã nguồn nhúng thời gian thực trên vi điều khiển **STM32F407VGT6 + FreeRTOS (200 Hz)**.

📄 **Tài liệu lưu đồ & tổng hợp các giải thuật (Kalman, CoM Decoupling, Rate-Domain Yaw Fusion, Cascaded Dual-FuzzyPD):**
- Bản PDF đầy đủ: [`Firmware/luu_do_dieu_khien_fuzzy.pdf`](./Firmware/luu_do_dieu_khien_fuzzy.pdf)
- Mã nguồn LaTeX: [`Firmware/luu_do_dieu_khien_fuzzy.tex`](./Firmware/luu_do_dieu_khien_fuzzy.tex)
- Tài liệu chi tiết phần Firmware: [`Firmware/README.md`](./Firmware/README.md)

---

## Cấu Trúc Toàn Bộ Dự Án (Workspace)

```text
balance-robot/
├── Firmware/                         # Mã nguồn nhúng STM32F407VG (FreeRTOS 200 Hz, Keil MDK-ARM)
│   ├── MDK_ARM/                      # Project Keil uVision 5, main.c, Objects/ & Listings/
│   ├── MY_LIB/                       # Thư viện giải thuật: fuzzy, kalman, pid, managetask, encoder, motor, i2c, uart, flash
│   ├── FreeRTOS/ & Libraries/        # Nhân FreeRTOS v11.1 & thư viện ngoại vi chuẩn STM32F4xx
│   ├── lan20.csv .. lan39.csv        # Dữ liệu thực nghiệm Telemetry DMA 50 Hz trên robot thực tế
│   └── luu_do_dieu_khien_fuzzy.pdf   # Báo cáo lưu đồ kiến trúc & 29 phương trình giải thuật
├── app/                              # Phần mềm máy tính (Python GUI) giám sát đồ thị 50 Hz, điều khiển & Live Tuning
│   ├── gui_app.py                    # Giao diện điều khiển, nạp nóng Fuzzy/PID, dò vùng chết tự động & xuất CSV
│   ├── requirements.txt              # Các thư viện Python cần thiết
│   └── run.bat                       # Script khởi chạy nhanh giao diện trên Windows
├── 3d/                               # Thiết kế cơ khí 3D khung xe, gá động cơ, bánh xe & lắp ghép tổng thể (SolidWorks / STEP)
├── Hardware/                         # Thiết kế mạch nguyên lý (Schematic) & mạch in (PCB Layout) trên Altium Designer
└── SIMULATE/                         # Mô phỏng động lực học Simscape Multibody (.slx) & kiểm chứng bộ lọc Kalman (.m)
```

---

## Điểm Nổi Bật Kỹ Thuật

1. **Kiến trúc Điều khiển Mờ Vòng Lồng Kép (`Cascaded Dual-FuzzyPD` — 200 Hz / 5 ms):**
   - **Vòng ngoài (`FuzzyPD_Pos`):** Kết hợp tách vận tốc/vị trí Trọng tâm (`CoM Decoupling`), bộ kẹp chống bắn ngược (`Anti-Slingshot Clamping ±4.0 cm`) và tích phân mặt trượt tự học góc cân bằng tĩnh (`Sliding-Mode Auto-Trim`).
   - **Vòng trong (`FuzzyPD_Pitch` — 90% công suất PWM = `±7560/8400`):** Ổn định góc chúc ngóc quanh điểm cân bằng chuẩn `g_targetAngle = -3.56°` với sai số tĩnh $\sigma_{\text{pitch}} = 0.11^\circ$.
   - **Vòng hướng xoay (`Yaw PID` vi phân kép — 10% công suất PWM = `±840/8400`):** Giữ chặt hướng xoay kết hợp bù vùng chết bất đối xứng 4 góc phần tư (`Soft-Knee Deadband`) và `Slew-Rate Limiter`.
2. **Hệ thống 4 Bộ lọc Kalman 2 Trạng thái & Hợp nhất Yaw (`Rate-Domain + ZUPT`):**
   - `kalmanPitch` & `kalmanYaw`: Ước lượng góc thực và độ trôi nhiệt Gyroscope ở tần số 200 Hz.
   - `kalmanSpeed_L` & `kalmanSpeed_R`: Lọc động học vận tốc – gia tốc có hệ số tắt dần gia tốc $\gamma = 0.80$, triệt tiêu bậc thang lượng tử hóa $9.09\text{ RPM/xung}$ với độ trễ pha $< 20\text{ ms}$.
   - Hợp nhất vận tốc góc Yaw 3 chế độ + khóa tĩnh `ZUPT` giúp độ trôi khi đứng yên đạt `0.000 °/s` và không bị Encoder kéo ngược khi có ngoại lực xoay xe.
3. **Phần mềm Giám sát & Tinh chỉnh Trực tuyến (`app/gui_app.py`):**
   - Thu thập khung Telemetry nhị phân 88-byte (`22 float`, kiểm tra `XOR Checksum`) qua `USART3 DMA` ở tần số `50 Hz`.
   - Hỗ trợ nạp nóng thông số `FuzzyPD` (`0x0A`), `PID` (`0x08`), tự động dò vùng chết 2 bánh xe (`0x0B`) và lưu cấu hình hiệu chuẩn vào `Flash Sector 11`.
