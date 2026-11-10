"""
=============================================================================
  APP GIAO TIẾP & ĐIỀU KHIỂN ROBOT CÂN BẰNG (STM32F4 + FreeRTOS)
  Học phần: BTL Nhập môn Điều khiển Thông minh
=============================================================================
  TAB 1 - GIÁM SÁT : Pitch / Yaw (raw & Kalman), PWM 2 động cơ, vận tốc bánh,
                     vị trí hiện tại (odometry tính trên PC từ encoder), nhật ký UART.
  TAB 2 - ĐIỀU KHIỂN: Joystick / WASD / nút bấm lái xe, bản đồ 2D quỹ đạo xe.
  TAB 3 - TUNING   : Chọn bộ điều khiển (Dual PID / Fuzzy), chỉnh Kp-Kd-Ki,
                     hệ số Fuzzy, góc cân bằng; đồ thị debug từng vòng điều khiển.

  Giao thức UART (115200 8N1), khung: [0xAA][ID][LEN][PAYLOAD][CHECKSUM][0xFF]
  Checksum = XOR(ID, LEN, các byte payload)  (khớp UART_CalculateChecksum của STM32)

  PC -> STM32:
    0x01 START | 0x02 STOP | 0x03 CALIB | 0x04 góc Pitch đặt (float)
    0x05 vận tốc đặt (2 float RPM, + = tiến) | 0x06 len0: reset Yaw / len4: Yaw đặt (float °)
    0x07 mode (uint8: 0 = Cascade, 1 = Song song)
    0x08 PID Yaw (3 float): Kp, Ki, Kd
    0x09 yêu cầu gửi cấu hình | 0x0A Fuzzy (6 float): Pitch ke,kde,ku, Pos ke,kde,ku (áp cho chế độ đang chạy)
    0x0B test PWM (2 float): PWM trái, PWM phải (chỉ khi STOP)
  STM32 -> PC:
    0x01 Telemetry 88B (22 float):
      rawPitch, pitch, pitchRate, rawYaw, yaw, yawRate, rawSpdL, rawSpdR, kalSpdL, kalSpdR,
      pwmL, pwmR, targetAngle, outFuzzy (PWM nhánh Pitch), outYaw,
      outPos (Cascade: góc bù °, Song song: PWM nhánh vị trí),
      velCmd, posError (cm), pitchE, pitchDE, posE, posDE  (e/de đã chuẩn hóa [-1, 1])
    0x02 Config 48B (12 float):
      Kp_yaw, Ki_yaw, Kd_yaw, targetAngle, targetYaw, mode,
      Pitch ke,kde,ku, Pos ke,kde,ku (của chế độ đang chạy)
=============================================================================
"""

import math
import time
import struct
import threading
import queue
import csv
import random
import bisect
from datetime import datetime
from typing import Optional, List, Tuple

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from tkinter.scrolledtext import ScrolledText

try:
    import ctypes
    ctypes.windll.shcore.SetProcessDpiAwareness(1)
except Exception:
    pass

import serial
import serial.tools.list_ports

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


# =============================================================================
# HẰNG SỐ GIAO THỨC & VẬT LÝ (khớp firmware)
# =============================================================================
UART_FRAME_HEADER = 0xAA
UART_FRAME_TAIL   = 0xFF

CMD_START, CMD_STOP, CMD_CALIB = 0x01, 0x02, 0x03
CMD_SET_ANGLE, CMD_SET_SPEED, CMD_SET_YAW = 0x04, 0x05, 0x06
CMD_SET_MODE, CMD_SET_PID, CMD_REQ_CONFIG, CMD_SET_FUZZY = 0x07, 0x08, 0x09, 0x0A
CMD_TEST_PWM = 0x0B   # 2 float: PWM thử bánh trái/phải (chỉ khi STOP, tự tắt sau 0.5 s nếu không có lệnh mới)

TELEM_ID_ROBOT, TELEM_ID_CONFIG = 0x01, 0x02
TELEM_LEN, CONFIG_LEN, CONFIG_LEN_EXT = 88, 48, 64   # 22 float / 12 float hoặc 16 float (RobotConfigSync_t)

WHEEL_DIAMETER_CM       = 6.5
WHEEL_RADIUS_CM         = 3.25
WHEEL_CIRCUMFERENCE_CM  = math.pi * WHEEL_DIAMETER_CM        # 20.42 cm/vòng
WHEEL_RPM_TO_CMS        = WHEEL_CIRCUMFERENCE_CM / 60.0
ROBOT_WHEELBASE_CM      = 19.21
ROBOT_MAX_SPEED_CMS     = 10.0
ROBOT_MAX_SPEED_RPM     = ROBOT_MAX_SPEED_CMS / WHEEL_RPM_TO_CMS
PWM_MAX                 = 8400.0
DEFAULT_PITCH_TARGET    = -3.56

TELEM_FIELDS = (
    "raw_pitch", "pitch", "pitch_rate", "raw_yaw", "yaw", "yaw_rate",
    "raw_L", "raw_R", "kal_L", "kal_R", "pwm_L", "pwm_R", "tgt_ang",
    "out_fuzzy", "out_yaw", "out_pos",
    "vel_cmd", "pos_err", "pitch_e", "pitch_de", "pos_e", "pos_de",
)
# Các kênh tự tính trên PC
EXTRA_FIELDS = ("x", "y", "speed_cms", "speed_cmd_cms", "tgt_yaw", "tilt")
ALL_FIELDS = ("t",) + TELEM_FIELDS + EXTRA_FIELDS


CHECKSUM_MODE = "xor"   # "xor": firmware hiện tại | "or": firmware cũ. Tự nhận dạng khi nhận gói hợp lệ (cả RX lẫn TX dùng chung)


def _cs_xor(msg_id: int, length: int, payload: bytes) -> int:
    cs = (msg_id ^ length) & 0xFF
    for b in payload:
        cs ^= b
    return cs


def _cs_or(msg_id: int, length: int, payload: bytes) -> int:
    cs = (msg_id | length) & 0xFF
    for b in payload:
        cs = (cs | b) & 0xFF
    return cs


def calculate_checksum(msg_id: int, length: int, payload: bytes, mode: Optional[str] = None) -> int:
    return (_cs_or if (mode or CHECKSUM_MODE) == "or" else _cs_xor)(msg_id, length, payload)


def build_packet(msg_id: int, payload: bytes = b"") -> bytes:
    length = len(payload)
    cs = calculate_checksum(msg_id, length, payload)
    return bytes([UART_FRAME_HEADER, msg_id, length]) + payload + bytes([cs, UART_FRAME_TAIL])


class PacketParser:
    """Tách khung gói tin từ luồng byte UART (có đồng bộ lại khi lỗi).
    Chấp nhận checksum XOR (firmware mới) hoặc OR (firmware cũ) và tự chuyển chế độ để lệnh gửi xuống cũng khớp."""
    VALID = {TELEM_ID_ROBOT: (TELEM_LEN,), TELEM_ID_CONFIG: (CONFIG_LEN, CONFIG_LEN_EXT)}

    def __init__(self):
        self.buf = bytearray()
        self.n_ok = 0            # số khung hợp lệ
        self.n_bad = 0           # số khung đúng header/độ dài/tail nhưng SAI checksum
        self.mode_changed: Optional[str] = None

    def feed(self, data: bytes) -> List[Tuple[int, bytes]]:
        global CHECKSUM_MODE
        self.buf.extend(data)
        out: List[Tuple[int, bytes]] = []
        while True:
            i = self.buf.find(UART_FRAME_HEADER)
            if i < 0:
                self.buf.clear()
                break
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < 3:
                break
            mid, ln = self.buf[1], self.buf[2]
            if ln not in self.VALID.get(mid, ()):
                del self.buf[:1]
                continue
            size = ln + 5
            if len(self.buf) < size:
                break
            if self.buf[size - 1] != UART_FRAME_TAIL:
                del self.buf[:1]
                continue
            payload = bytes(self.buf[3:3 + ln])
            rx = self.buf[3 + ln]
            cx, co = _cs_xor(mid, ln, payload), _cs_or(mid, ln, payload)
            if rx == cx or rx == co:
                new_mode = None
                if rx != co:
                    new_mode = "xor"
                elif rx != cx:
                    new_mode = "or"
                if new_mode and new_mode != CHECKSUM_MODE:
                    CHECKSUM_MODE = new_mode
                    self.mode_changed = new_mode
                self.n_ok += 1
                out.append((mid, payload))
                del self.buf[:size]
            else:
                self.n_bad += 1
                del self.buf[:1]
        if len(self.buf) > 4096:
            del self.buf[:-256]
        return out


# =============================================================================
# CỬA SỔ DÒ VÙNG CHẾT 2 BÁNH XE (tăng PWM từng bước, ghi PWM lúc bánh bắt đầu quay)
# =============================================================================
class DeadzoneDialog:
    TESTS = [("L", +1), ("R", +1), ("L", -1), ("R", -1)]
    NAMES = {("L", +1): "Trái, PWM +", ("L", -1): "Trái, PWM -", ("R", +1): "Phải, PWM +", ("R", -1): "Phải, PWM -"}
    COLORS = {("L", +1): "#89dceb", ("L", -1): "#74c7ec", ("R", +1): "#f38ba8", ("R", -1): "#fab387"}

    def __init__(self, app):
        self.app = app
        self.running = False
        self.results = {}
        self.curves = {k: ([], []) for k in self.TESTS}
        c = app
        win = self.win = tk.Toplevel(app.root)
        win.title("Dò vùng chết 2 bánh xe")
        win.geometry("820x700")
        win.configure(bg=c.C_BG)
        win.protocol("WM_DELETE_WINDOW", self.close)

        tk.Label(win, text="⚠ NÂNG BÁNH XE KHỎI MẶT ĐẤT (bánh quay tự do), robot ở trạng thái STOP, giữ chắc xe!",
                 fg=c.C_RED, bg=c.C_BG, font=("Segoe UI", 10, "bold")).pack(fill=tk.X, padx=10, pady=(8, 2))
        tk.Label(win, text="Mỗi bánh lần lượt được tăng PWM từng bước (cả 2 chiều). PWM lúc bánh bắt đầu quay là ngưỡng vùng chết.",
                 fg=c.C_DIM, bg=c.C_BG, font=("Segoe UI", 8)).pack(fill=tk.X, padx=10)

        pf = tk.Frame(win, bg=c.C_CARD, bd=1, relief="ridge")
        pf.pack(fill=tk.X, padx=10, pady=6)
        self.ent = {}
        for i, (key, text, default) in enumerate((("max", "PWM tối đa", "1200"), ("step", "Bước PWM", "20"),
                                                  ("tstep", "Thời gian/bước (s)", "0.45"),
                                                  ("thr", "Ngưỡng quay (RPM)", "2.5"), ("rest", "Nghỉ giữa các lần (s)", "1.0"))):
            tk.Label(pf, text=text, fg=c.C_DIM, bg=c.C_CARD, font=("Segoe UI", 8, "bold")).grid(row=0, column=i, padx=6, pady=(4, 0))
            e = tk.Entry(pf, width=8, bg=c.C_BG, fg="#ffffff", justify="center", font=("Consolas", 10, "bold"),
                         insertbackground="#ffffff")
            e.insert(0, default)
            e.grid(row=1, column=i, padx=6, pady=(0, 6))
            self.ent[key] = e

        bf = tk.Frame(win, bg=c.C_BG)
        bf.pack(fill=tk.X, padx=10)
        self.btn_start = tk.Button(bf, text="▶ BẮT ĐẦU DÒ", command=self.start, bg="#2e7d32", fg="#ffffff", relief="flat",
                                   font=("Segoe UI", 9, "bold"), cursor="hand2", pady=4)
        self.btn_start.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        tk.Button(bf, text="⏹ DỪNG", command=self.abort, bg="#b71c1c", fg="#ffffff", relief="flat",
                  font=("Segoe UI", 9, "bold"), cursor="hand2", pady=4).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        tk.Button(bf, text="📋 Sao chép #define", command=self.copy_defines, bg="#1565c0", fg="#ffffff", relief="flat",
                  font=("Segoe UI", 9, "bold"), cursor="hand2", pady=4).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        self.lbl_status = tk.Label(win, text="Sẵn sàng.", fg=c.C_YELLOW, bg=c.C_BG, font=("Segoe UI", 9, "bold"), anchor="w")
        self.lbl_status.pack(fill=tk.X, padx=10, pady=(6, 0))

        rf = tk.Frame(win, bg=c.C_CARD, bd=1, relief="ridge")
        rf.pack(fill=tk.X, padx=10, pady=6)
        self.res_lbl = {}
        self.res_stop_lbl = {}
        self.results_stop = {}
        for i, k in enumerate(self.TESTS):
            tk.Label(rf, text=self.NAMES[k], fg=self.COLORS[k], bg=c.C_CARD, font=("Segoe UI", 8, "bold")).grid(row=0, column=i, padx=14, pady=(4, 0))
            l = tk.Label(rf, text="—", fg="#ffffff", bg=c.C_CARD, font=("Consolas", 12, "bold"))
            l.grid(row=1, column=i, padx=14, pady=(0, 0))
            self.res_lbl[k] = l
            l2 = tk.Label(rf, text="dừng: —", fg=c.C_DIM, bg=c.C_CARD, font=("Consolas", 9, "bold"))
            l2.grid(row=2, column=i, padx=14, pady=(0, 4))
            self.res_stop_lbl[k] = l2
        self.lbl_rec = tk.Label(win, text="", fg=c.C_GREEN, bg=c.C_BG, font=("Consolas", 10, "bold"), justify="left", anchor="w")
        self.lbl_rec.pack(fill=tk.X, padx=10)

        self.fig = Figure(figsize=(7.5, 3.6), dpi=100)
        self.fig.patch.set_facecolor(c.C_PANEL)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor("#11111b")
        self.ax.tick_params(colors=c.C_DIM, labelsize=8)
        for sp in self.ax.spines.values():
            sp.set_color(c.C_BORDER)
        self.ax.grid(True, linestyle="--", alpha=0.3, color=c.C_BORDER)
        self.ax.set_xlabel("PWM đặt (đơn vị timer, ±8400)", color=c.C_TEXT, fontsize=9)
        self.ax.set_ylabel("|Tốc độ bánh| (RPM)", color=c.C_TEXT, fontsize=9)
        self.lines = {}
        for k in self.TESTS:
            self.lines[k], = self.ax.plot([], [], "o-", ms=3, lw=1.2, color=self.COLORS[k], label=self.NAMES[k])
        self.ax.legend(loc="upper left", facecolor=c.C_CARD, edgecolor=c.C_BORDER, labelcolor=c.C_TEXT, fontsize=8)
        self.fig.tight_layout(pad=1.0)
        self.canvas = FigureCanvasTkAgg(self.fig, master=win)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 8))

    # ----------------------------------------------------------------- điều khiển
    def start(self):
        a = self.app
        if self.running:
            return
        if not (a.is_connected or a.is_simulating):
            messagebox.showwarning("Chưa kết nối", "Hãy kết nối cổng COM (hoặc bật mô phỏng) trước!", parent=self.win)
            return
        if a.robot_state == "RUN":
            messagebox.showwarning("Robot đang chạy", "Hãy bấm STOP trước khi dò vùng chết!", parent=self.win)
            return
        try:
            self.p_max = min(2000.0, abs(float(self.ent["max"].get())))
            self.p_step = max(5.0, abs(float(self.ent["step"].get())))
            self.p_tstep = max(0.15, float(self.ent["tstep"].get()))
            self.p_thr = max(0.5, float(self.ent["thr"].get()))
            self.p_rest = max(0.3, float(self.ent["rest"].get()))
        except ValueError:
            messagebox.showerror("Lỗi", "Các tham số phải là số!", parent=self.win)
            return
        if not messagebox.askokcancel("Xác nhận", "Bánh xe đã được nâng lên khỏi mặt đất và xe được giữ chắc?", parent=self.win):
            return
        self.results = {}
        self.results_stop = {}
        self.curves = {k: ([], []) for k in self.TESTS}
        for k in self.TESTS:
            self.res_lbl[k].config(text="—", fg="#ffffff")
            self.res_stop_lbl[k].config(text="dừng: —", fg=self.app.C_DIM)
            self.lines[k].set_data([], [])
        self.lbl_rec.config(text="")
        self.idx = 0
        self.phase = "rest"
        self.phase_t0 = time.time()
        self.pwm = 0.0
        self._last_tx_t = 0.0
        self._last_tx_pwm = None
        self.running = True
        self.btn_start.config(state=tk.DISABLED)
        self.status("Đang dò... (giữ chắc xe)")
        self._tick()

    def abort(self, msg="Đã dừng."):
        was = self.running
        self.running = False
        for _ in range(2):
            self.app._send_test_pwm(0.0, 0.0)
        if was:
            self.status(msg)
        self.btn_start.config(state=tk.NORMAL)

    def close(self):
        self.abort("Đã dừng.")
        self.win.destroy()

    def status(self, text):
        self.lbl_status.config(text=text)

    def _evaluate(self, wheel, t_start):
        a = self.app
        ts = a.hist["t"]
        i0 = bisect.bisect_left(ts, t_start + 0.30 * self.p_tstep)
        sp = a.hist["raw_" + wheel][i0:]
        if not sp:
            return False, 0.0, 0
        mags = [abs(v) for v in sp]
        mean_abs = sum(mags) / len(mags)
        frac = sum(1 for m in mags if m >= 1.0) / len(mags)
        return (mean_abs >= self.p_thr and frac >= 0.5), mean_abs, len(sp)

    def _next_test(self, now):
        self.idx += 1
        self.pwm = 0.0
        self.phase = "rest"
        self.phase_t0 = now
        if self.idx >= len(self.TESTS):
            self.finish()
            return False
        return True

    def _tick(self):
        if not self.running:
            return
        a = self.app
        now = time.time()
        wheel, sign = self.TESTS[self.idx]
        key = (wheel, sign)
        p = self.pwm * sign
        tx_pair = (p if wheel == "L" else 0.0, p if wheel == "R" else 0.0)
        # Chỉ gửi khi đổi mức PWM hoặc mỗi 160ms để giữ sống (tránh nghẽn buffer Bluetooth COM)
        if tx_pair != self._last_tx_pwm or (now - self._last_tx_t) >= 0.16:
            a._send_test_pwm(*tx_pair)
            self._last_tx_pwm = tx_pair
            self._last_tx_t = now

        if self.phase == "rest":
            self.pwm = 0.0
            if now - self.phase_t0 >= self.p_rest:
                self.phase = "ramp"
                self.pwm = self.p_step
                self.step_t0 = now
                self.t_pc = now - a.start_ts
                self.status("Đang dò bánh %s, PWM %s (tăng dần) ..." % ("TRÁI" if wheel == "L" else "PHẢI", "+" if sign > 0 else "−"))
        elif now - self.step_t0 >= self.p_tstep:
            moving, mean_abs, n = self._evaluate(wheel, self.t_pc)
            if n < 2:
                # Nếu Bluetooth gom gói bị trễ nhẹ, chờ thêm tối đa 0.35s; chỉ báo lỗi nếu mất gói > 1.5s
                if (now - self.step_t0) < (self.p_tstep + 0.35) and (a.last_pkt_ts is not None and (now - a.last_pkt_ts) < 1.5):
                    self.win.after(50, self._tick)
                    return
                self.abort("LỖI: không nhận được telemetry từ xe (0.0 Hz - kiểm tra nguồn xe / Bluetooth).")
                return
            if self.phase == "ramp":
                xs, ys = self.curves[key]
                xs.append(self.pwm)
                ys.append(mean_abs)
                self.lines[key].set_data(xs, ys)
                self.ax.relim()
                self.ax.autoscale_view()
                self.canvas.draw_idle()
                if moving:
                    # Ma sát tĩnh: PWM lúc bắt đầu quay. Tiếp theo giảm PWM để tìm PWM lúc bánh dừng (ma sát động, quan trọng khi xe đang chạy)
                    self.results[key] = self.pwm
                    self.res_lbl[key].config(text="%.0f" % self.pwm, fg=a.C_GREEN)
                    self.phase = "down"
                    self.pwm = max(0.0, self.pwm - self.p_step)
                    self.step_t0 = now
                    self.t_pc = now - a.start_ts
                    self.status("Đang dò bánh %s, PWM %s (giảm dần để tìm điểm dừng) ..." % ("TRÁI" if wheel == "L" else "PHẢI", "+" if sign > 0 else "−"))
                elif self.pwm >= self.p_max:
                    self.results[key] = None
                    self.res_lbl[key].config(text="> %.0f" % self.p_max, fg=a.C_RED)
                    if not self._next_test(now):
                        return
                else:
                    self.pwm += self.p_step
                    self.step_t0 = now
                    self.t_pc = now - a.start_ts
            else:   # phase == "down"
                if not moving or self.pwm <= 0.0:
                    stop = self.pwm + self.p_step if not moving else 0.0     # PWM nhỏ nhất bánh còn quay
                    self.results_stop[key] = stop
                    self.res_stop_lbl[key].config(text="dừng: %.0f" % stop, fg=a.C_YELLOW)
                    if not self._next_test(now):
                        return
                else:
                    self.pwm = max(0.0, self.pwm - self.p_step)
                    self.step_t0 = now
                    self.t_pc = now - a.start_ts
        self.win.after(50, self._tick)

    # ----------------------------------------------------------------- kết quả
    def recommend(self):
        """Vùng chết bù cho từng bánh, từng chiều. Bánh thường xuyên đang quay khi cân bằng nên dùng ma sát ĐỘNG
        (PWM nhỏ nhất bánh còn quay khi giảm dần); nếu không đo được thì dùng ma sát tĩnh. Hệ số 0.85."""
        out = {}
        for k in self.TESTS:
            v = self.results_stop.get(k)
            if v is None or v <= 0:
                v = self.results.get(k)
            out[k] = round(0.85 * v / 10.0) * 10.0 if v else None
        return out

    def finish(self):
        self.running = False
        for _ in range(3):
            self.app._send_test_pwm(0.0, 0.0)
        self.btn_start.config(state=tk.NORMAL)
        rec = self.recommend()
        fmt = lambda v: "%.0f" % v if v is not None else "?"
        lines = []
        for w, name in (("L", "TRÁI"), ("R", "PHẢI")):
            lines.append("Bánh %s: _POS ≈ %s | _NEG ≈ %s   (quay: +%s/−%s, dừng: +%s/−%s)" % (
                name, fmt(rec[(w, 1)]), fmt(rec[(w, -1)]),
                fmt(self.results.get((w, 1))), fmt(self.results.get((w, -1))),
                fmt(self.results_stop.get((w, 1))), fmt(self.results_stop.get((w, -1)))))
        vl = [v for v in (rec[("L", 1)], rec[("L", -1)]) if v is not None]
        vr = [v for v in (rec[("R", 1)], rec[("R", -1)]) if v is not None]
        if vl and vr:
            dl, dr = sum(vl) / len(vl), sum(vr) / len(vr)
            lines.append("Chênh lệch (theo ma sát động): bánh %s cần PWM lớn hơn %.0f." % ("PHẢI" if dr > dl else "TRÁI", abs(dr - dl)))
        lines.append("Lưu ý: ma sát TĨNH (bắt đầu quay) thường lớn hơn ma sát ĐỘNG (dừng); dùng giá trị ĐỘNG để bù khi xe đang cân bằng.")
        self.lbl_rec.config(text="\n".join(lines))
        self.status("Hoàn tất. Đặt các giá trị #define vào managetask.c rồi nạp lại firmware.")
        self.app._log("[DÒ VÙNG CHẾT] " + " | ".join(lines) + "\n")

    def copy_defines(self):
        rec = self.recommend()
        if all(v is None for v in rec.values()):
            messagebox.showinfo("Thông báo", "Chưa có kết quả!", parent=self.win)
            return
        txt = ""
        for w in ("L", "R"):
            for sign, tag in ((1, "POS"), (-1, "NEG")):
                if rec[(w, sign)] is not None:
                    txt += "#define MOTOR_DEADBAND_PWM_%s_%s   %.1ff\n" % (w, tag, rec[(w, sign)])
        self.win.clipboard_clear()
        self.win.clipboard_append(txt)
        self.status("Đã sao chép vào clipboard.")


# =============================================================================
# GIAO DIỆN CHÍNH
# =============================================================================
class RobotControlGUI:
    C_BG, C_PANEL, C_CARD, C_BORDER = "#181825", "#1e1e2e", "#26283b", "#3b3e5b"
    C_TEXT, C_DIM = "#cdd6f4", "#9399b2"
    C_ACCENT, C_GREEN, C_RED, C_YELLOW = "#89b4fa", "#a6e3a1", "#f38ba8", "#f9e2af"
    C_ORANGE, C_PURPLE, C_CYAN = "#fab387", "#cba6f7", "#89dceb"

    # Độ dốc quanh 0 = out_small / ns của từng kênh (khớp Fuzzy_Init trong main.c sau khi tuning)
    FZ_SLOPE_PITCH_E, FZ_SLOPE_PITCH_DE = 0.28 / 0.20, 0.28 / 0.20
    FZ_SLOPE_POS_E, FZ_SLOPE_POS_DE     = 0.28 / 0.16, 0.28 / 0.20
    TILT_MAX_DEG = 2.20           # ku của FuzzyPD_Pos (giới hạn góc bù động, cascade)
    POS_PWM_MAX  = 2520.0         # dự phòng
    MODE_NAMES   = ("Dual PID (Cascade)", "Dual FuzzyPD (Cascade)")

    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("ROBOT CÂN BẰNG STM32F4 - GIÁM SÁT / ĐIỀU KHIỂN / TUNING")
        root.geometry("1480x940")
        root.minsize(1200, 780)
        root.configure(bg=self.C_BG)

        # Kết nối
        self.serial_port: Optional[serial.Serial] = None
        self.is_connected = False
        self.is_simulating = False
        self.rx_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.data_queue: "queue.Queue" = queue.Queue()
        self.config_queue: "queue.Queue" = queue.Queue()
        self.log_queue: "queue.Queue" = queue.Queue()
        self.tx_queue: "queue.Queue" = queue.Queue()

        # Dữ liệu
        self.hist = {k: [] for k in ALL_FIELDS}
        self.cur = {k: 0.0 for k in ALL_FIELDS}
        self.start_ts = time.time()
        self.last_pkt_ts: Optional[float] = None
        self.rx_count = 0
        self.freq_count = 0
        self.freq_time = time.time()
        self.rx_bytes = 0          # tổng byte nhận được từ cổng COM (chẩn đoán mất kết nối dữ liệu)
        self.rx_bad = 0            # số khung sai checksum
        self._diag_bytes = 0
        self._diag_ok = 0
        self._diag_t = time.time()
        self._last_tile_t = 0.0
        self.has_telem = False

        # Lệnh đặt hiện tại
        self.tgt_speed_rpm = 0.0       # RPM, dương = tiến
        self.tgt_yaw = 0.0
        self.tgt_pitch = DEFAULT_PITCH_TARGET
        self.robot_state = "STOP"
        self.ctrl_mode = 1             # 0 = CTRL_MODE_DUAL_PID, 1 = CTRL_MODE_FUZZY (mặc định trên STM32)
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.test_pwm = (0.0, 0.0, 0.0)   # (L, R, thời điểm) cho chế độ mô phỏng dò vùng chết

        self.is_paused = False
        self.window_s = 15.0
        self.is_running = True

        # Joystick / bàn phím
        self.joy_x = 0.0
        self.joy_y = 0.0
        self.joy_mouse = False
        self.keys = set()
        self._joy_was_active = False
        self._last_sent_speed = None
        self._updating = False

        self._setup_styles()
        self._build_ui()
        root.bind_all("<KeyPress>", self._on_key_press, add="+")
        root.bind_all("<KeyRelease>", self._on_key_release, add="+")
        root.protocol("WM_DELETE_WINDOW", self._on_close)

        root.after(20, self._process_incoming)
        root.after(100, self._update_plots)
        root.after(100, self._joystick_tick)

    # ------------------------------------------------------------------ styles
    def _setup_styles(self):
        st = ttk.Style()
        st.theme_use("clam")
        st.configure(".", background=self.C_BG, foreground=self.C_TEXT)
        st.configure("TFrame", background=self.C_BG)
        st.configure("TCombobox", fieldbackground=self.C_CARD, background=self.C_BORDER, foreground="#ffffff")
        st.map("TCombobox", fieldbackground=[("readonly", self.C_CARD)])
        st.configure("TScale", background=self.C_CARD, troughcolor=self.C_BORDER)
        st.configure("TNotebook", background=self.C_BG, borderwidth=0)
        st.configure("TNotebook.Tab", background=self.C_PANEL, foreground=self.C_TEXT,
                     padding=[18, 8], font=("Segoe UI", 10, "bold"))
        st.map("TNotebook.Tab", background=[("selected", self.C_CARD), ("active", self.C_BORDER)],
               foreground=[("selected", "#89dceb"), ("active", "#ffffff")])

    # ------------------------------------------------------------- UI helpers
    def _card(self, parent, title, color=None, **pack):
        f = tk.LabelFrame(parent, text=f" {title} ", bg=self.C_CARD, fg=color or self.C_ACCENT,
                          font=("Segoe UI", 9, "bold"), bd=1, relief="ridge", padx=6, pady=4)
        f.pack(**({"fill": tk.X, "pady": (0, 4)} | pack))
        return f

    def _btn(self, parent, text, cmd, bg=None, fg="#ffffff", side=tk.LEFT, **kw):
        b = tk.Button(parent, text=text, command=cmd, bg=bg or self.C_BORDER, fg=fg, relief="flat",
                      font=("Segoe UI", 8, "bold"), cursor="hand2", pady=3, **kw)
        b.pack(side=side, expand=True, fill=tk.X, padx=1)
        return b

    def _tile(self, parent, r, c, title, default, color):
        box = tk.Frame(parent, bg=self.C_BG, bd=1, relief="solid", padx=5, pady=2)
        box.grid(row=r, column=c, padx=2, pady=2, sticky="nsew")
        tk.Label(box, text=title, fg=self.C_DIM, bg=self.C_BG, font=("Segoe UI", 7, "bold")).pack(anchor="w")
        v = tk.Label(box, text=default, fg=color, bg=self.C_BG, font=("Segoe UI", 10, "bold"))
        v.pack(anchor="e")
        return v

    def _entry(self, parent, width, text, color="#ffffff", **pack):
        e = tk.Entry(parent, width=width, bg=self.C_BG, fg=color, justify="center",
                     insertbackground="#ffffff", font=("Consolas", 9, "bold"))
        e.insert(0, text)
        e.pack(**({"side": tk.LEFT, "padx": 2} | pack))
        return e

    @staticmethod
    def _set_entry(e: tk.Entry, text: str):
        e.delete(0, tk.END)
        e.insert(0, text)

    def _label(self, parent, text, color=None, **pack):
        l = tk.Label(parent, text=text, fg=color or self.C_DIM, bg=self.C_CARD, font=("Segoe UI", 8, "bold"))
        l.pack(**({"side": tk.LEFT} | pack))
        return l

    def _make_fig(self, parent, n, ratios=None):
        fig, axes = plt.subplots(n, 1, figsize=(8, 7.5), sharex=False,
                                 gridspec_kw={"height_ratios": ratios} if ratios else None)
        axes = [axes] if n == 1 else list(axes)
        fig.patch.set_facecolor(self.C_PANEL)
        for ax in axes:
            ax.set_facecolor("#11111b")
            ax.tick_params(colors=self.C_DIM, labelsize=8)
            for sp in ax.spines.values():
                sp.set_color(self.C_BORDER)
            ax.grid(True, linestyle="--", alpha=0.3, color=self.C_BORDER)
        canvas = FigureCanvasTkAgg(fig, master=parent)
        canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        return fig, axes, canvas

    def _legend(self, ax, ncol=3):
        ax.legend(loc="upper right", facecolor=self.C_CARD, edgecolor=self.C_BORDER,
                  labelcolor=self.C_TEXT, fontsize=7.5, ncol=ncol)

    def _ylabel(self, ax, text):
        ax.set_ylabel(text, color=self.C_TEXT, fontsize=9, fontweight="bold")

    def _plot_bar(self, parent, with_window=True):
        bar = tk.Frame(parent, bg=self.C_PANEL, bd=1, relief="ridge", padx=8, pady=4)
        bar.pack(side=tk.TOP, fill=tk.X, pady=(0, 4))
        if with_window:
            tk.Label(bar, text="Khung thời gian:", fg=self.C_DIM, bg=self.C_PANEL,
                     font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=(0, 4))
            cb = ttk.Combobox(bar, width=5, state="readonly", values=["5s", "10s", "15s", "30s", "60s"])
            cb.set("15s")
            cb.pack(side=tk.LEFT, padx=2)
            cb.bind("<<ComboboxSelected>>", lambda e: self._on_window(cb.get()))
        for text, cmd, bg, fg in (("💾 Xuất CSV", self._export_csv, "#313244", self.C_GREEN),
                                  ("🗑 Xóa", self._clear_data, self.C_BORDER, "#ffffff"),
                                  ("⏸ Tạm dừng", self._toggle_pause, self.C_BORDER, "#ffffff")):
            b = tk.Button(bar, text=text, command=cmd, bg=bg, fg=fg, relief="flat", cursor="hand2",
                          font=("Segoe UI", 9, "bold"), width=10)
            b.pack(side=tk.RIGHT, padx=2)
            if "Tạm dừng" in text:
                self.pause_buttons.append(b)
        return bar

    # ------------------------------------------------------------- main build
    def _build_ui(self):
        self.pause_buttons: List[tk.Button] = []
        top = tk.Frame(self.root, bg=self.C_PANEL, bd=1, relief="ridge")
        top.pack(side=tk.TOP, fill=tk.X, padx=10, pady=(8, 4))
        self._build_top_bar(top)

        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=4)
        t1, t2, t3 = ttk.Frame(self.notebook), ttk.Frame(self.notebook), ttk.Frame(self.notebook)
        self.notebook.add(t1, text=" 📊 TAB 1: GIÁM SÁT ")
        self.notebook.add(t2, text=" 🕹️ TAB 2: ĐIỀU KHIỂN XE ")
        self.notebook.add(t3, text=" 🎛️ TAB 3: TUNING BỘ ĐIỀU KHIỂN ")
        self._build_tab_monitor(t1)
        self._build_tab_drive(t2)
        self._build_tab_tuning(t3)

    def _build_top_bar(self, parent):
        inner = tk.Frame(parent, bg=self.C_PANEL)
        inner.pack(fill=tk.BOTH, expand=True, padx=12, pady=6)
        tk.Label(inner, text="🤖 BALANCING ROBOT", font=("Segoe UI", 11, "bold"),
                 fg=self.C_CYAN, bg=self.C_PANEL).pack(side=tk.LEFT, padx=(0, 12))
        tk.Label(inner, text="Cổng COM:", fg=self.C_DIM, bg=self.C_PANEL, font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT)
        self.cb_ports = ttk.Combobox(inner, width=12, state="readonly")
        self.cb_ports.pack(side=tk.LEFT, padx=3)
        tk.Button(inner, text="🔄", bg=self.C_CARD, fg="#ffffff", relief="flat", width=3,
                  command=self._refresh_ports).pack(side=tk.LEFT, padx=2)
        self._refresh_ports()
        tk.Label(inner, text="Baud:", fg=self.C_DIM, bg=self.C_PANEL, font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=(8, 2))
        self.cb_baud = ttk.Combobox(inner, width=8, state="readonly",
                                    values=["115200", "9600", "57600", "230400", "460800", "921600"])
        self.cb_baud.set("115200")
        self.cb_baud.pack(side=tk.LEFT, padx=3)
        self.btn_connect = tk.Button(inner, text="🔌 Kết Nối", bg="#2e7d32", fg="#ffffff", relief="flat",
                                     font=("Segoe UI", 9, "bold"), width=13, cursor="hand2",
                                     command=self._toggle_connection)
        self.btn_connect.pack(side=tk.LEFT, padx=8)
        self.var_demo = tk.BooleanVar(value=False)
        tk.Checkbutton(inner, text="Chế độ mô phỏng (không cần xe)", variable=self.var_demo,
                       command=self._toggle_demo, fg=self.C_YELLOW, bg=self.C_PANEL, selectcolor=self.C_CARD,
                       activebackground=self.C_PANEL, font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT, padx=6)
        self.led = tk.Canvas(inner, width=16, height=16, bg=self.C_PANEL, highlightthickness=0)
        self.led.pack(side=tk.LEFT, padx=(8, 3))
        self.led_dot = self.led.create_oval(2, 2, 14, 14, fill=self.C_RED, outline="")
        self.lbl_status = tk.Label(inner, text="Chưa kết nối", fg=self.C_DIM, bg=self.C_PANEL, font=("Segoe UI", 9))
        self.lbl_status.pack(side=tk.LEFT)
        self.lbl_rate = tk.Label(inner, text="0.0 Hz | 0 pkts", fg=self.C_ACCENT, bg=self.C_PANEL, font=("Segoe UI", 9))
        self.lbl_rate.pack(side=tk.RIGHT, padx=8)

    # =========================================================== TAB 1: MONITOR
    def _build_tab_monitor(self, parent):
        box = tk.Frame(parent, bg=self.C_BG)
        box.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        left = tk.Frame(box, bg=self.C_BG, width=470)
        left.pack(side=tk.LEFT, fill=tk.BOTH, padx=(0, 6))
        left.pack_propagate(False)
        right = tk.Frame(box, bg=self.C_BG)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        c = self._card(left, "⚡ ĐIỀU KHIỂN HỆ THỐNG")
        row = tk.Frame(c, bg=self.C_CARD)
        row.pack(fill=tk.X)
        self._btn(row, "▶ START", self._cmd_start, "#1b5e20")
        self._btn(row, "⏹ STOP", self._cmd_stop, "#b71c1c")
        self._btn(row, "⚖ CALIB", self._cmd_calib, "#e65100")
        self._btn(row, "📍 Reset vị trí", self._reset_position, "#00695c")
        self.lbl_state = tk.Label(c, text="TRẠNG THÁI: DỪNG (STANDBY)", fg=self.C_YELLOW, bg=self.C_CARD,
                                  font=("Segoe UI", 8, "bold"))
        self.lbl_state.pack(fill=tk.X, pady=(3, 0))

        c = self._card(left, "📊 GÓC PITCH / YAW")
        g = tk.Frame(c, bg=self.C_CARD)
        g.pack(fill=tk.X)
        g.columnconfigure(0, weight=1)
        g.columnconfigure(1, weight=1)
        self.t_pitch = self._tile(g, 0, 0, "PITCH: RAW | KALMAN", "0.00° | 0.00°", self.C_CYAN)
        self.t_yaw = self._tile(g, 0, 1, "YAW: RAW | KALMAN", "0.0° | 0.0°", self.C_PURPLE)
        self.t_rate = self._tile(g, 1, 0, "GYRO RATE (PITCH | YAW)", "0.0 | 0.0 °/s", self.C_GREEN)
        self.t_err = self._tile(g, 1, 1, "SAI SỐ |e_Pitch| | |e_Yaw|", "0.00° | 0.0°", self.C_YELLOW)

        c = self._card(left, "⚙ PWM ĐỘNG CƠ & VẬN TỐC")
        g = tk.Frame(c, bg=self.C_CARD)
        g.pack(fill=tk.X)
        g.columnconfigure(0, weight=1)
        g.columnconfigure(1, weight=1)
        self.t_pwm = self._tile(g, 0, 0, "PWM TRÁI | PHẢI (±8400)", "0 | 0", self.C_ORANGE)
        self.t_pwm_pct = self._tile(g, 0, 1, "PWM % TRÁI | PHẢI", "0.0% | 0.0%", self.C_ORANGE)
        self.t_wheel_raw = self._tile(g, 1, 0, "BÁNH RAW L | R (RPM)", "0.0 | 0.0", self.C_RED)
        self.t_wheel_kal = self._tile(g, 1, 1, "BÁNH KALMAN L | R (RPM)", "0.0 | 0.0", self.C_CYAN)
        self.t_speed = self._tile(g, 2, 0, "VẬN TỐC XE (cm/s)", "0.00", self.C_GREEN)
        self.t_pwm_parts = self._tile(g, 2, 1, "OUT: BALANCE | YAW", "0 | 0", self.C_PURPLE)

        c = self._card(left, "📍 VỊ TRÍ HIỆN TẠI (odometry từ encoder)")
        g = tk.Frame(c, bg=self.C_CARD)
        g.pack(fill=tk.X)
        g.columnconfigure(0, weight=1)
        g.columnconfigure(1, weight=1)
        self.t_pos = self._tile(g, 0, 0, "X | Y (cm)", "0.0 | 0.0", self.C_CYAN)
        self.t_dist = self._tile(g, 0, 1, "QUÃNG ĐƯỜNG TỪ GỐC (cm)", "0.0", self.C_YELLOW)

        c = self._card(left, "💬 NHẬT KÝ UART", fill=tk.BOTH, expand=True, pady=0)
        self.txt = ScrolledText(c, bg="#11111b", fg="#a6adc8", font=("Consolas", 8), height=4, bd=0)
        self.txt.pack(fill=tk.BOTH, expand=True)
        row = tk.Frame(c, bg=self.C_CARD)
        row.pack(fill=tk.X, pady=(2, 0))
        tk.Button(row, text="Xóa log", bg=self.C_BORDER, fg="#ffffff", relief="flat",
                  command=lambda: self.txt.delete("1.0", tk.END)).pack(side=tk.RIGHT)

        self._plot_bar(right)
        self.fig1, (a1, a2, a3), self.canvas1 = self._make_fig(right, 3, [1.2, 1.0, 1.1])
        self._ylabel(a1, "Pitch (°)")
        self.l_raw_pitch, = a1.plot([], [], label="Raw Pitch", color=self.C_RED, lw=1.0, alpha=0.8)
        self.l_pitch, = a1.plot([], [], label="Kalman Pitch", color=self.C_CYAN, lw=2.0)
        self.l_tgt_ang, = a1.plot([], [], label="Góc đặt θ_ref", color=self.C_YELLOW, ls="--", lw=1.4)
        self._legend(a1)
        self._ylabel(a2, "Yaw (°)")
        self.l_raw_yaw, = a2.plot([], [], label="Raw Yaw", color=self.C_ORANGE, ls=":", lw=1.2)
        self.l_yaw, = a2.plot([], [], label="Kalman Yaw", color=self.C_PURPLE, lw=2.0)
        self.l_tgt_yaw, = a2.plot([], [], label="Yaw đặt", color=self.C_GREEN, ls="--", lw=1.4)
        self._legend(a2)
        self._ylabel(a3, "PWM (%) & Vận tốc (cm/s)")
        a3.set_xlabel("Thời gian (s)", color=self.C_TEXT, fontsize=9)
        self.l_pwm_L, = a3.plot([], [], label="PWM Trái (%)", color=self.C_ORANGE, lw=1.4)
        self.l_pwm_R, = a3.plot([], [], label="PWM Phải (%)", color=self.C_RED, lw=1.4)
        self.l_spd_cms, = a3.plot([], [], label="Vận tốc (cm/s)", color=self.C_GREEN, lw=1.8)
        self._legend(a3)
        self.axes1 = (a1, a2, a3)
        self.fig1.tight_layout(pad=1.1)

    # ============================================================ TAB 2: DRIVE
    def _build_tab_drive(self, parent):
        box = tk.Frame(parent, bg=self.C_BG)
        box.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        left = tk.Frame(box, bg=self.C_BG, width=480)
        left.pack(side=tk.LEFT, fill=tk.BOTH, padx=(0, 6))
        left.pack_propagate(False)
        right = tk.Frame(box, bg=self.C_BG)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        c = self._card(left, "⚡ HỆ THỐNG")
        row = tk.Frame(c, bg=self.C_CARD)
        row.pack(fill=tk.X)
        self._btn(row, "▶ START", self._cmd_start, "#1b5e20")
        self._btn(row, "⏹ STOP", self._cmd_stop, "#b71c1c")
        self._btn(row, "⏹ PHANH (0)", self._brake, "#e65100")
        self._btn(row, "📍 Reset vị trí", self._reset_position, "#00695c")

        # Joystick
        c = self._card(left, "🕹️ CẦN JOYSTICK / PHÍM WASD", "#a6e3a1")
        top = tk.Frame(c, bg=self.C_CARD)
        top.pack(fill=tk.X)
        self.var_kb = tk.BooleanVar(value=True)
        tk.Checkbutton(top, text="⌨ Dùng WASD / mũi tên (Space = phanh)", variable=self.var_kb,
                       fg=self.C_YELLOW, bg=self.C_CARD, selectcolor=self.C_BG, activebackground=self.C_CARD,
                       font=("Segoe UI", 8, "bold")).pack(side=tk.LEFT)
        self.var_invert_turn = tk.BooleanVar(value=False)
        tk.Checkbutton(top, text="Đảo chiều rẽ", variable=self.var_invert_turn, fg=self.C_DIM, bg=self.C_CARD,
                       selectcolor=self.C_BG, activebackground=self.C_CARD,
                       font=("Segoe UI", 8, "bold")).pack(side=tk.RIGHT)

        mid = tk.Frame(c, bg=self.C_CARD)
        mid.pack(fill=tk.X, pady=3)
        self.joy_size, self.joy_r, self.knob_r = 170, 62, 16
        self.joy_c = self.joy_size // 2
        self.cv_joy = tk.Canvas(mid, width=self.joy_size, height=self.joy_size, bg="#11111b",
                                highlightthickness=1, highlightbackground=self.C_BORDER, cursor="fleur")
        self.cv_joy.pack(side=tk.LEFT, padx=(0, 6))
        cc, r = self.joy_c, self.joy_r
        self.cv_joy.create_oval(cc - r, cc - r, cc + r, cc + r, outline="#45475a", width=2)
        self.cv_joy.create_oval(cc - r // 2, cc - r // 2, cc + r // 2, cc + r // 2, outline="#313244", dash=(3, 3))
        self.cv_joy.create_line(cc, 6, cc, self.joy_size - 6, fill="#313244")
        self.cv_joy.create_line(6, cc, self.joy_size - 6, cc, fill="#313244")
        self.cv_joy.create_text(cc, 9, text="▲ TIẾN", fill=self.C_GREEN, font=("Segoe UI", 7, "bold"))
        self.cv_joy.create_text(cc, self.joy_size - 9, text="▼ LÙI", fill=self.C_RED, font=("Segoe UI", 7, "bold"))
        self.cv_joy.create_text(18, cc - 9, text="◄TRÁI", fill=self.C_PURPLE, font=("Segoe UI", 6, "bold"))
        self.cv_joy.create_text(self.joy_size - 19, cc - 9, text="PHẢI►", fill=self.C_CYAN, font=("Segoe UI", 6, "bold"))
        self.joy_stem = self.cv_joy.create_line(cc, cc, cc, cc, fill=self.C_ACCENT, width=4, capstyle=tk.ROUND)
        self.joy_knob = self.cv_joy.create_oval(cc - self.knob_r, cc - self.knob_r, cc + self.knob_r,
                                                cc + self.knob_r, fill=self.C_CYAN, outline="#ffffff", width=2)
        self.cv_joy.bind("<ButtonPress-1>", self._joy_press)
        self.cv_joy.bind("<B1-Motion>", self._joy_drag)
        self.cv_joy.bind("<ButtonRelease-1>", self._joy_release)

        side = tk.Frame(mid, bg=self.C_CARD)
        side.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.lbl_joy = tk.Label(side, text="Tiến/lùi: +0% | Rẽ: +0%", fg=self.C_CYAN, bg=self.C_CARD,
                                font=("Consolas", 9, "bold"))
        self.lbl_joy.pack(anchor="w")
        row = tk.Frame(side, bg=self.C_CARD)
        row.pack(fill=tk.X, pady=2)
        self._label(row, "Vận tốc tối đa (cm/s):")
        self.ent_joy_vmax = self._entry(row, 5, "6.0", self.C_YELLOW)
        row = tk.Frame(side, bg=self.C_CARD)
        row.pack(fill=tk.X, pady=2)
        self._label(row, "Tốc độ rẽ (°/s):")
        self.ent_joy_turn = self._entry(row, 5, "45", self.C_PURPLE)
        tk.Label(side, text="Thả cần/phím → xe phanh & giữ hướng hiện tại.", fg=self.C_DIM, bg=self.C_CARD,
                 font=("Segoe UI", 7), wraplength=230, justify="left").pack(anchor="w", pady=(4, 0))

        # Lệnh trực tiếp
        c = self._card(left, "🎯 ĐẶT VẬN TỐC & HƯỚNG (YAW)")
        row = tk.Frame(c, bg=self.C_CARD)
        row.pack(fill=tk.X, pady=1)
        self._label(row, "Vận tốc (cm/s):", self.C_YELLOW, ipadx=0)
        self.var_speed = tk.DoubleVar(value=0.0)
        ttk.Scale(row, from_=-ROBOT_MAX_SPEED_CMS, to=ROBOT_MAX_SPEED_CMS, variable=self.var_speed,
                  orient=tk.HORIZONTAL, command=self._on_speed_slide).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=3)
        self.ent_speed = self._entry(row, 5, "0.0", self.C_YELLOW)
        self.ent_speed.bind("<Return>", lambda e: self._send_speed_entry())
        self._btn(row, "Gửi", self._send_speed_entry, "#e65100", width=4)
        row = tk.Frame(c, bg=self.C_CARD)
        row.pack(fill=tk.X, pady=1)
        self._label(row, "Yaw đặt (°):", self.C_PURPLE)
        self.var_yaw = tk.DoubleVar(value=0.0)
        ttk.Scale(row, from_=-180, to=180, variable=self.var_yaw, orient=tk.HORIZONTAL,
                  command=self._on_yaw_slide).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=3)
        self.ent_yaw = self._entry(row, 5, "0.0", self.C_PURPLE)
        self.ent_yaw.bind("<Return>", lambda e: self._send_yaw_entry())
        self._btn(row, "Gửi", self._send_yaw_entry, "#6a1b9a", width=4)

        # Nút bấm cố định
        c = self._card(left, "🧭 NÚT LÁI NHANH", self.C_CYAN)
        cfg = tk.Frame(c, bg=self.C_CARD)
        cfg.pack(fill=tk.X, pady=(0, 3))
        self._label(cfg, "Vận tốc:")
        self.cb_step_v = ttk.Combobox(cfg, state="readonly", width=9,
                                      values=["2 cm/s", "4 cm/s", "6 cm/s", "8 cm/s", "10 cm/s"])
        self.cb_step_v.set("4 cm/s")
        self.cb_step_v.pack(side=tk.LEFT, padx=3)
        self._label(cfg, "Góc xoay:", padx=(8, 0))
        self.cb_step_a = ttk.Combobox(cfg, state="readonly", width=6, values=["15°", "30°", "45°", "90°", "180°"])
        self.cb_step_a.set("45°")
        self.cb_step_a.pack(side=tk.LEFT, padx=3)
        grid = tk.Frame(c, bg=self.C_CARD)
        grid.pack(fill=tk.X)
        for i in range(3):
            grid.columnconfigure(i, weight=1)
        spec = [
            (0, 0, "↖ Tiến-Trái", lambda: self._drive_step(1, -1), self.C_BORDER),
            (0, 1, "⬆ TIẾN", lambda: self._drive_step(1, 0), "#1b5e20"),
            (0, 2, "↗ Tiến-Phải", lambda: self._drive_step(1, 1), self.C_BORDER),
            (1, 0, "↶ XOAY TRÁI", lambda: self._nudge_yaw(-self._step_angle()), "#4a148c"),
            (1, 1, "⏹ PHANH", self._brake, "#e65100"),
            (1, 2, "↷ XOAY PHẢI", lambda: self._nudge_yaw(self._step_angle()), "#0d47a1"),
            (2, 0, "↙ Lùi-Trái", lambda: self._drive_step(-1, -1), self.C_BORDER),
            (2, 1, "⬇ LÙI", lambda: self._drive_step(-1, 0), "#b71c1c"),
            (2, 2, "↘ Lùi-Phải", lambda: self._drive_step(-1, 1), self.C_BORDER),
        ]
        for r_, c_, txt, cmd, bg in spec:
            tk.Button(grid, text=txt, command=cmd, bg=bg, fg="#ffffff", relief="flat", cursor="hand2",
                      font=("Segoe UI", 8, "bold"), pady=5).grid(row=r_, column=c_, padx=1, pady=1, sticky="nsew")
        row = tk.Frame(c, bg=self.C_CARD)
        row.pack(fill=tk.X, pady=(3, 0))
        self._label(row, "Hướng:")
        for d in (0, -90, 90, 180):
            self._btn(row, f"{d:+d}°" if d not in (0, 180) else f"{d}°", lambda v=d: self._set_yaw(float(v)),
                      self.C_BG, self.C_PURPLE)

        c = self._card(left, "📟 TRẠNG THÁI XE", fill=tk.BOTH, expand=True, pady=0)
        g = tk.Frame(c, bg=self.C_CARD)
        g.pack(fill=tk.X)
        g.columnconfigure(0, weight=1)
        g.columnconfigure(1, weight=1)
        self.d_speed = self._tile(g, 0, 0, "VẬN TỐC ĐẶT | ĐO (cm/s)", "0.0 | 0.0", self.C_YELLOW)
        self.d_yaw = self._tile(g, 0, 1, "YAW ĐẶT | ĐO (°)", "0.0 | 0.0", self.C_PURPLE)
        self.d_pos = self._tile(g, 1, 0, "VỊ TRÍ X | Y (cm)", "0.0 | 0.0", self.C_CYAN)
        self.d_pitch = self._tile(g, 1, 1, "PITCH (°)", "0.00", self.C_GREEN)
        self.d_whl_L = self._tile(g, 2, 0, "BÁNH TRÁI: RPM | cm/s (tiến = +)", "0.0 | 0.0", self.C_CYAN)
        self.d_whl_R = self._tile(g, 2, 1, "BÁNH PHẢI: RPM | cm/s (tiến = +)", "0.0 | 0.0", self.C_ORANGE)

        # Bên phải: bản đồ + đồ thị
        self._plot_bar(right)
        self.fig2, (m, s, w, y), self.canvas2 = self._make_fig(right, 4, [2.0, 1.0, 1.1, 0.9])
        self.ax_map = m
        m.set_aspect("equal", adjustable="box")
        m.set_title("BẢN ĐỒ 2D: QUỸ ĐẠO XE (odometry)", color=self.C_YELLOW, fontsize=9, fontweight="bold")
        m.set_xlabel("X (cm)", color=self.C_TEXT, fontsize=8)
        m.set_ylabel("Y (cm)", color=self.C_TEXT, fontsize=8)
        self.l_traj, = m.plot([], [], color=self.C_CYAN, lw=1.5, label="Quỹ đạo")
        self.l_robot, = m.plot([], [], "o", color=self.C_GREEN, ms=8, label="Xe")
        self.l_head, = m.plot([], [], "-", color=self.C_RED, lw=2.5, label="Hướng")
        self.l_body, = m.plot([], [], "-", color="#ffffff", lw=2)
        m.plot([0], [0], "x", color=self.C_YELLOW, ms=8, label="Gốc")
        self._legend(m, 4)
        self._ylabel(s, "Vận tốc (cm/s)")
        self.l_v_cmd, = s.plot([], [], label="Đặt", color=self.C_YELLOW, ls="--", lw=1.4)
        self.l_v_meas, = s.plot([], [], label="Đo", color=self.C_GREEN, lw=1.8)
        self._legend(s, 2)
        self._ylabel(w, "Tốc độ bánh (RPM)")
        self.l_wL_raw, = w.plot([], [], label="Trái (raw)", color=self.C_CYAN, ls=":", lw=1.0, alpha=0.55)
        self.l_wR_raw, = w.plot([], [], label="Phải (raw)", color=self.C_ORANGE, ls=":", lw=1.0, alpha=0.55)
        self.l_wL, = w.plot([], [], label="Trái (Kalman)", color=self.C_CYAN, lw=1.8)
        self.l_wR, = w.plot([], [], label="Phải (Kalman)", color=self.C_ORANGE, lw=1.8)
        w.axhline(0.0, color=self.C_BORDER, lw=0.8)
        self._legend(w, 4)
        self._ylabel(y, "Yaw (°)")
        y.set_xlabel("Thời gian (s)", color=self.C_TEXT, fontsize=8)
        self.l_y_cmd, = y.plot([], [], label="Đặt", color=self.C_YELLOW, ls="--", lw=1.4)
        self.l_y_meas, = y.plot([], [], label="Đo (Kalman)", color=self.C_PURPLE, lw=1.8)
        self._legend(y, 2)
        self.axes2 = (m, s, w, y)
        self.fig2.tight_layout(pad=1.1)

    # =========================================================== TAB 3: TUNING
    def _build_tab_tuning(self, parent):
        box = tk.Frame(parent, bg=self.C_BG)
        box.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        left = tk.Frame(box, bg=self.C_BG, width=500)
        left.pack(side=tk.LEFT, fill=tk.BOTH, padx=(0, 6))
        left.pack_propagate(False)
        right = tk.Frame(box, bg=self.C_BG)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        c = self._card(left, "⚡ ĐIỀU KHIỂN & BỘ ĐK")
        row = tk.Frame(c, bg=self.C_CARD)
        row.pack(fill=tk.X)
        self._btn(row, "▶ START", self._cmd_start, "#1b5e20")
        self._btn(row, "⏹ STOP", self._cmd_stop, "#b71c1c")
        self._btn(row, "⚖ CALIB", self._cmd_calib, "#e65100")
        self._btn(row, "🔄 ĐỌC CẤU HÌNH STM32", self._request_config, "#1565c0")
        self._btn(row, "🔧 DÒ VÙNG CHẾT", self._open_deadzone_dialog, "#6a1b9a")
        row = tk.Frame(c, bg=self.C_CARD)
        row.pack(fill=tk.X, pady=(4, 0))
        self._label(row, "Bộ điều khiển:")
        self.cb_mode = ttk.Combobox(row, state="readonly", width=38,
                                    values=["0: Dual PID Cascade (PID_Vel → PD_Pitch)",
                                            "1: Dual FuzzyPD Cascade (FuzzyPD_Pos → FuzzyPD_Pitch)"])
        self.cb_mode.current(1)
        self.cb_mode.pack(side=tk.LEFT, padx=3, expand=True, fill=tk.X)
        self.cb_mode.bind("<<ComboboxSelected>>", lambda e: self._send_mode())
        self._btn(row, "Nạp", self._send_mode, "#1565c0", width=4)

        # Góc cân bằng
        c = self._card(left, "🎯 GÓC CÂN BẰNG (PITCH ĐẶT)")
        row = tk.Frame(c, bg=self.C_CARD)
        row.pack(fill=tk.X)
        self.var_pitch = tk.DoubleVar(value=DEFAULT_PITCH_TARGET)
        sc = ttk.Scale(row, from_=-15, to=15, variable=self.var_pitch, orient=tk.HORIZONTAL,
                       command=lambda v: self._set_entry(self.ent_pitch, f"{float(v):.2f}"))
        sc.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=3)
        sc.bind("<ButtonRelease-1>", lambda e: self._send_pitch())
        self.ent_pitch = self._entry(row, 6, f"{DEFAULT_PITCH_TARGET:.2f}", self.C_CYAN)
        self.ent_pitch.bind("<Return>", lambda e: self._send_pitch())
        self._btn(row, "Gửi", self._send_pitch, "#1565c0", width=4)
        for d in (-0.1, 0.1):
            self._btn(row, f"{d:+.1f}°", lambda dd=d: self._nudge_pitch(dd), width=5)

        # PID giữ hướng Yaw
        c = self._card(left, "🎛️ PID GIỮ HƯỚNG YAW (lệnh 0x08)", self.C_CYAN)
        self.pid_entries = {}
        spec = [
            ("kp_yaw", "Kp Yaw (PWM/°)", "25.2", self.C_PURPLE, 1.0),
            ("ki_yaw", "Ki Yaw", "4.20", self.C_CYAN, 0.2),
            ("kd_yaw", "Kd Yaw (gyro, PWM/(°/s))", "3.78", self.C_GREEN, 0.2),
        ]
        for key, text, default, col, step in spec:
            row = tk.Frame(c, bg=self.C_CARD)
            row.pack(fill=tk.X, pady=1)
            self._label(row, text, self.C_TEXT, anchor="w")
            e = self._entry(row, 8, default, col, side=tk.RIGHT)
            tk.Button(row, text="+", bg=self.C_BORDER, fg="#fff", relief="flat", width=2,
                      command=lambda k=key, s=step: self._nudge_pid(k, s)).pack(side=tk.RIGHT)
            tk.Button(row, text="-", bg=self.C_BORDER, fg="#fff", relief="flat", width=2,
                      command=lambda k=key, s=step: self._nudge_pid(k, -s)).pack(side=tk.RIGHT)
            e.bind("<Return>", lambda ev: self._send_pid())
            self.pid_entries[key] = e
        row = tk.Frame(c, bg=self.C_CARD)
        row.pack(fill=tk.X, pady=(3, 0))
        self._btn(row, "💾 NẠP PID YAW (0x08)", self._send_pid, "#2e7d32")

        # Fuzzy
        c = self._card(left, "🧠 FUZZY (lệnh 0x0A, áp cho chế độ đang chạy)", self.C_YELLOW)
        g = tk.Frame(c, bg=self.C_CARD)
        g.pack(fill=tk.X)
        for i, t in enumerate(("", "ke", "kde", "ku")):
            tk.Label(g, text=t, fg=self.C_DIM, bg=self.C_CARD, font=("Segoe UI", 8, "bold")).grid(row=0, column=i)
        self.fz = []
        self.lbl_fz = []
        for r_, (name, defs, col) in enumerate((("FuzzyPD_Pitch (ku: PWM)", ("0.086957", "0.003125", "7560.0"), self.C_CYAN),
                                                ("FuzzyPD_Pos (ku: °)", ("0.025", "0.060", "2.40"), self.C_YELLOW)), 1):
            lb = tk.Label(g, text=name, fg=col, bg=self.C_CARD, font=("Segoe UI", 8, "bold"))
            lb.grid(row=r_, column=0, sticky="w")
            self.lbl_fz.append(lb)
            es = []
            for c_, d in enumerate(defs, 1):
                e = tk.Entry(g, width=10, bg=self.C_BG, fg=col, justify="center", insertbackground="#fff",
                             font=("Consolas", 9, "bold"))
                e.insert(0, d)
                e.grid(row=r_, column=c_, padx=3, pady=2)
                e.bind("<KeyRelease>", lambda ev: self._update_fz_equiv())
                e.bind("<Return>", lambda ev: self._send_fuzzy())
                es.append(e)
            self.fz.append(es)
        self.lbl_equiv = tk.Label(c, text="", fg=self.C_DIM, bg=self.C_CARD, font=("Consolas", 8),
                                  justify="left", anchor="w")
        self.lbl_equiv.pack(fill=tk.X, pady=(3, 2))
        row = tk.Frame(c, bg=self.C_CARD)
        row.pack(fill=tk.X)
        self._btn(row, "💾 NẠP FUZZY", self._send_fuzzy, "#2e7d32")
        self._btn(row, "⛔ TẮT V.NGOÀI (ku_Pos=0)", self._disable_outer, "#8c2f39")

        c = self._card(left, "📟 SỐ LIỆU TỨC THỜI", fill=tk.BOTH, expand=True, pady=0)
        g = tk.Frame(c, bg=self.C_CARD)
        g.pack(fill=tk.X)
        g.columnconfigure(0, weight=1)
        g.columnconfigure(1, weight=1)
        self.f_vel = self._tile(g, 0, 0, "VẬN TỐC ĐẶT | ĐO (RPM)", "0.0 | 0.0", self.C_YELLOW)
        self.f_tilt = self._tile(g, 0, 1, "GÓC ĐẶT (°) | RA V.VỊ TRÍ", "0.00 | 0.00", self.C_ORANGE)
        self.f_pd = self._tile(g, 1, 0, "PITCH: e | de (norm)", "0.00 | 0.00", self.C_CYAN)
        self.f_pi = self._tile(g, 1, 1, "POS: e | de (norm)", "0.00 | 0.00", self.C_YELLOW)
        self.f_sat = self._tile(g, 2, 0, "% BÃO HÒA (3s)", "Pitch 0% | Pos 0%", self.C_GREEN)
        self.f_out = self._tile(g, 2, 1, "OUT CÂN BẰNG (PWM)", "0", self.C_CYAN)

        # Đồ thị
        self._plot_bar(right)
        self.fig3, (p, v, pd, pi), self.canvas3 = self._make_fig(right, 4, [1.2, 1.2, 0.9, 0.9])
        self._ylabel(p, "Pitch (°)")
        self.l3_pitch, = p.plot([], [], label="Kalman Pitch", color=self.C_CYAN, lw=1.9)
        self.l3_tgt, = p.plot([], [], label="Góc đặt θ_ref", color=self.C_YELLOW, ls="--", lw=1.4)
        self.l3_base = p.axhline(self.tgt_pitch, color=self.C_GREEN, ls=":", lw=1.0)
        self._legend(p, 2)
        self._ylabel(v, "Vận tốc (RPM, tiến = +)")
        self.l3_vcmd, = v.plot([], [], label="Đặt", color=self.C_YELLOW, ls="--", lw=1.4)
        self.l3_vmeas, = v.plot([], [], label="Đo (Kalman TB)", color=self.C_CYAN, lw=1.8)
        self._legend(v, 2)
        for ax, ttl in ((pd, "Pitch e/de"), (pi, "Pos e/de")):
            self._ylabel(ax, ttl)
            ax.set_ylim(-1.15, 1.15)
            for lv in (1.0, -1.0):
                ax.axhline(lv, color=self.C_RED, lw=0.9, alpha=0.8)
            for lv in (0.25, -0.25):
                ax.axhline(lv, color=self.C_BORDER, ls=":", lw=0.9)
        self.l3_pd_e, = pd.plot([], [], label="e_norm", color=self.C_CYAN, lw=1.5)
        self.l3_pd_de, = pd.plot([], [], label="de_norm", color=self.C_GREEN, lw=1.0, alpha=0.85)
        self.l3_pi_e, = pi.plot([], [], label="e_norm", color=self.C_YELLOW, lw=1.5)
        self.l3_pi_de, = pi.plot([], [], label="de_norm", color=self.C_PURPLE, lw=1.0, alpha=0.85)
        pi.set_xlabel("Thời gian (s)", color=self.C_TEXT, fontsize=8)
        self._legend(pd, 2)
        self._legend(pi, 2)
        self.axes3 = (p, v, pd, pi)
        self.fig3.tight_layout(pad=1.1)
        self._update_fz_equiv()

    # ============================================================= SERIAL COMM
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.cb_ports["values"] = ports
        if ports and self.cb_ports.get() not in ports:
            self.cb_ports.current(0)
        elif not ports:
            self.cb_ports.set("")

    def _set_status(self, text, color, led):
        self.lbl_status.config(text=text, fg=color)
        self.led.itemconfig(self.led_dot, fill=led)

    def _toggle_connection(self):
        if self.is_connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.cb_ports.get()
        if not port:
            messagebox.showwarning("Cảnh báo", "Hãy chọn cổng COM trước!")
            return
        if self.is_simulating:
            self.var_demo.set(False)
            self._toggle_demo()
        try:
            baud = int(self.cb_baud.get())
            self.serial_port = serial.Serial(port=port, baudrate=baud, timeout=0.015, write_timeout=0.1)
            try:
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
            except Exception:
                pass
            while not self.tx_queue.empty():
                self.tx_queue.get_nowait()
            self.is_connected = True
            self.stop_event.clear()
            self.rx_thread = threading.Thread(target=self._reader_thread, args=(self.serial_port,), daemon=True)
            self.rx_thread.start()
            self.btn_connect.config(text="❌ Ngắt Kết Nối", bg="#b71c1c")
            self._set_status(f"Đã kết nối {port} @ {baud}", self.C_GREEN, self.C_GREEN)
            self._log(f"[HỆ THỐNG] Mở cổng {port} @ {baud}. Đang đọc cấu hình từ STM32...\n")
            self.root.after(200, self._request_config)
        except Exception as e:
            self.is_connected = False
            self.serial_port = None
            messagebox.showerror("Lỗi kết nối", f"Không mở được {port}:\n{e}")

    def _disconnect(self):
        self.is_connected = False
        self.stop_event.set()
        ser, self.serial_port = self.serial_port, None
        if ser is not None:
            try:
                if hasattr(ser, "cancel_read"):
                    ser.cancel_read()
            except Exception:
                pass
        self.btn_connect.config(text="🔌 Kết Nối", bg="#2e7d32")
        self._set_status("Chưa kết nối", self.C_DIM, self.C_RED)
        self._log("[HỆ THỐNG] Đã ngắt kết nối.\n")

    def _reader_thread(self, ser: serial.Serial):
        parser = PacketParser()
        last_ts = 0.0
        try:
            while not self.stop_event.is_set() and ser.is_open:
                while not self.tx_queue.empty():
                    try:
                        ser.write(self.tx_queue.get_nowait())
                    except queue.Empty:
                        break
                    except Exception as e:
                        self.log_queue.put(f"[TX LỖI] {e}\n")
                        break
                try:
                    n = ser.in_waiting
                    if n > 0:
                        raw = ser.read(min(4096, n))
                    else:
                        b0 = ser.read(1)
                        if not b0:
                            continue
                        n2 = ser.in_waiting
                        raw = b0 + (ser.read(min(4096, n2)) if n2 > 0 else b"")
                except Exception as e:
                    if not self.stop_event.is_set():
                        self.log_queue.put(f"[RX LỖI] {e}\n")
                    break
                if not raw:
                    continue
                self.rx_bytes += len(raw)
                pkts = parser.feed(raw)
                self.rx_bad = parser.n_bad
                if parser.mode_changed:
                    self.log_queue.put("[GIAO THỨC] Firmware dùng checksum %s -> app đã chuyển sang chế độ này (cả nhận và gửi lệnh).\n"
                                       % parser.mode_changed.upper())
                    parser.mode_changed = None
                now = time.time()
                tel = [p for p in pkts if p[0] == TELEM_ID_ROBOT]
                k = len(tel)
                idx = 0
                for mid, payload in pkts:
                    if mid == TELEM_ID_ROBOT:
                        # Gói về dồn cục (Bluetooth): rải đều timestamp 20ms
                        ts = max(last_ts + 0.004, now - (k - 1 - idx) * 0.020)
                        last_ts = ts
                        idx += 1
                        self.data_queue.put((ts, struct.unpack("<22f", payload)))
                    elif mid == TELEM_ID_CONFIG:
                        if len(payload) == CONFIG_LEN_EXT:
                            (kp_bal, kd_bal, kd_yaw_w, kp_yaw, kd_yaw_imu,
                             tang, tyaw, mode, kp_spd, ki_spd,
                             pdke, pdkde, pdku, pike, pikde, piku) = struct.unpack("<16f", payload)
                            self.config_queue.put((kp_yaw, ki_spd, kd_yaw_imu, tang, tyaw, mode,
                                                   pdke, pdkde, pdku, pike, pikde, piku))
                        else:
                            self.config_queue.put(struct.unpack("<12f", payload))
        finally:
            try:
                ser.close()
            except Exception:
                pass
            if self.is_connected and not self.stop_event.is_set():
                self.log_queue.put("[HỆ THỐNG] Mất kết nối cổng COM.\n")

    def _open_deadzone_dialog(self):
        if getattr(self, "_dz_dialog", None) is not None:
            try:
                if self._dz_dialog.win.winfo_exists():
                    self._dz_dialog.win.lift()
                    return
            except Exception:
                pass
        self._dz_dialog = DeadzoneDialog(self)

    def _send_test_pwm(self, pwm_l: float, pwm_r: float):
        """Lệnh 0x0B: đặt PWM trực tiếp từng bánh (chỉ khi robot STOP). Firmware tự tắt nếu 0.5 s không có lệnh mới."""
        if self.is_simulating:
            self.test_pwm = (pwm_l, pwm_r, time.time())
        elif self.is_connected:
            self.tx_queue.put(build_packet(CMD_TEST_PWM, struct.pack("<2f", pwm_l, pwm_r)))

    def _send(self, data: bytes, desc: str = "", silent: bool = False):
        if self.is_simulating:
            if desc and not silent:
                self._log(f"[DEMO TX] {desc}\n")
            return
        if not self.is_connected:
            if not silent:
                messagebox.showwarning("Chưa kết nối", "Hãy kết nối cổng COM (hoặc bật chế độ mô phỏng)!")
            return
        self.tx_queue.put(data)
        if desc and not silent:
            self._log(f"[TX] {desc}\n")

    # ================================================================ SIMULATOR
    def _toggle_demo(self):
        if self.var_demo.get():
            if self.is_connected:
                self._disconnect()
            self.is_simulating = True
            self.stop_event.clear()
            threading.Thread(target=self._sim_loop, daemon=True).start()
            self._set_status("🎮 Đang mô phỏng", self.C_YELLOW, self.C_YELLOW)
            self._log("[MÔ PHỎNG] Bắt đầu.\n")
            self.root.after(200, self._request_config)
        else:
            self.is_simulating = False
            self.stop_event.set()
            self._set_status("Chưa kết nối", self.C_DIM, self.C_RED)
            self._log("[MÔ PHỎNG] Đã dừng.\n")

    def _sim_loop(self):
        dt = 0.02
        yaw = spd = 0.0
        pitch = DEFAULT_PITCH_TARGET
        prev_v = 0.0
        swL = swR = 0.0
        mv = [False, False]
        t0 = time.time()
        while self.is_simulating and not self.stop_event.is_set():
            t = time.time() - t0
            run = self.robot_state == "RUN"
            v_cmd = self.tgt_speed_rpm if run else 0.0
            spd += (v_cmd - spd) * 0.15
            err = self.tgt_yaw - yaw
            yaw_rate = max(-90.0, min(90.0, 3.0 * err)) if run else 0.0
            yaw += yaw_rate * dt
            pitch = self.tgt_pitch + (0.4 * math.sin(t * 3.0) - 0.25 * spd / ROBOT_MAX_SPEED_RPM if run else 2.5 * math.sin(t * 0.8))
            kL, kR = -(spd - yaw_rate * 0.01), -(spd + yaw_rate * 0.01)
            if not run:
                # Mô phỏng vùng chết có trễ (ma sát tĩnh > động): bắt đầu quay khi |PWM| >= Zs, dừng khi |PWM| < Zk.
                # Bánh trái Zs = 280, bánh phải Zs = 620 (hộp số cứng), Zk = 250 cả hai. Tốc độ tiến = âm.
                tp = self.test_pwm
                live = (time.time() - tp[2]) < 0.5

                def _w(i, pw, zs, zk):
                    if not live:
                        mv[i] = False
                        return 0.0
                    if mv[i]:
                        if abs(pw) < zk:
                            mv[i] = False
                    elif abs(pw) >= zs:
                        mv[i] = True
                    return -math.copysign(8.0 + (abs(pw) - zk) * 0.05, pw) if mv[i] else 0.0
                swL += (_w(0, tp[0], 280.0, 250.0) - swL) * 0.3
                swR += (_w(1, tp[1], 620.0, 250.0) - swR) * 0.3
                kL, kR = swL, swR
            rL = round(kL / 2.27) * 2.27 + (random.uniform(-2, 2) if run else 0.0)
            rR = round(kR / 2.27) * 2.27 + (random.uniform(-2, 2) if run else 0.0)
            v_meas = 0.5 * (kL + kR)
            prev_v = v_meas
            out_f = max(-7560, min(7560, (pitch - self.tgt_pitch) * 440 + 11.5 * 5 * math.cos(t * 3))) if run else 0.0
            out_y = max(-840, min(840, err * 25.2)) if run else 0.0
            tilt = -0.25 * spd / ROBOT_MAX_SPEED_RPM * 2.5 if run else 0.0
            sat = lambda x: max(-1.0, min(1.0, x))
            f = (pitch + random.uniform(-1.2, 1.2), pitch, 0.0,
                 yaw + random.uniform(-0.4, 0.4), yaw, yaw_rate,
                 rL, rR, kL, kR,
                 out_f - out_y, out_f + out_y, self.tgt_pitch - tilt,
                 out_f, out_y, tilt,
                 -v_cmd, 0.0, sat((pitch - self.tgt_pitch) / 11.3) if run else 0.0, 0.0,
                 0.0, sat(-(v_meas + v_cmd) * WHEEL_RPM_TO_CMS / 25.1) if run else 0.0)
            self.data_queue.put((time.time(), f))
            time.sleep(dt)

    # ============================================================ SEND COMMANDS
    def _cmd_start(self):
        self._send(build_packet(CMD_START), "START (0x01)")
        self.robot_state = "RUN"
        self.lbl_state.config(text="TRẠNG THÁI: ĐANG CÂN BẰNG (RUN)", fg=self.C_GREEN)

    def _cmd_stop(self):
        self._send_speed_rpm(0.0, silent=True)
        self._send(build_packet(CMD_STOP), "STOP (0x02)")
        self.robot_state = "STOP"
        self.lbl_state.config(text="TRẠNG THÁI: DỪNG (STANDBY)", fg=self.C_RED)

    def _cmd_calib(self):
        if not messagebox.askokcancel("Calib", "Giữ xe đứng yên ở tư thế cân bằng rồi bấm OK."):
            return
        self._send(build_packet(CMD_CALIB), "AUTO CALIB (0x03)")
        self.robot_state = "STOP"
        self.lbl_state.config(text="TRẠNG THÁI: ĐANG CALIB (GIỮ XE ĐỨNG YÊN...)", fg=self.C_ORANGE)
        self.root.after(8000, self._request_config)

    def _request_config(self):
        if self.is_connected or self.is_simulating:
            self._send(build_packet(CMD_REQ_CONFIG), "REQUEST CONFIG (0x09)")
            if self.is_simulating:
                self.config_queue.put((25.2, 4.2, 3.78, self.tgt_pitch, self.tgt_yaw, float(self.ctrl_mode),
                                       1.0 / 11.5, 1.0 / 320.0, 7560.0, 0.022, 0.042, 2.20))

    def _send_speed_rpm(self, rpm: float, silent=False):
        rpm = max(-ROBOT_MAX_SPEED_RPM, min(ROBOT_MAX_SPEED_RPM, rpm))
        self.tgt_speed_rpm = rpm
        self._last_sent_speed = rpm
        self._send(build_packet(CMD_SET_SPEED, struct.pack("<2f", rpm, rpm)),
                   f"SPEED = {rpm:+.1f} RPM ({rpm * WHEEL_RPM_TO_CMS:+.2f} cm/s)", silent)

    def _send_yaw(self, yaw: float, silent=False):
        self.tgt_yaw = yaw
        self._send(build_packet(CMD_SET_YAW, struct.pack("<f", yaw)), f"YAW đặt = {yaw:.1f}°", silent)

    def _set_yaw(self, yaw: float):
        self._set_entry(self.ent_yaw, f"{yaw:.1f}")
        self._updating = True
        self.var_yaw.set(max(-180, min(180, yaw)))
        self._updating = False
        self._send_yaw(yaw)

    def _nudge_yaw(self, delta: float):
        self._set_yaw(self.tgt_yaw + delta)

    def _brake(self):
        self.joy_x = self.joy_y = 0.0
        self.keys.clear()
        self.joy_mouse = False
        self._draw_knob()
        self._set_speed_ui(0.0)
        self._send_speed_rpm(0.0)
        self._send_yaw(self.cur["yaw"])
        self._set_entry(self.ent_yaw, f"{self.tgt_yaw:.1f}")

    def _set_speed_ui(self, cms: float):
        self._updating = True
        self.var_speed.set(cms)
        self._updating = False
        self._set_entry(self.ent_speed, f"{cms:.1f}")

    def _on_speed_slide(self, v):
        if not self._updating:
            self._set_entry(self.ent_speed, f"{float(v):.1f}")
            self._send_speed_rpm(float(v) / WHEEL_RPM_TO_CMS, silent=True)

    def _on_yaw_slide(self, v):
        if not self._updating:
            self._set_entry(self.ent_yaw, f"{float(v):.1f}")

    def _send_speed_entry(self):
        try:
            cms = max(-ROBOT_MAX_SPEED_CMS, min(ROBOT_MAX_SPEED_CMS, float(self.ent_speed.get())))
        except ValueError:
            messagebox.showerror("Lỗi", "Vận tốc phải là số!")
            return
        self._set_speed_ui(cms)
        self._send_speed_rpm(cms / WHEEL_RPM_TO_CMS)

    def _send_yaw_entry(self):
        try:
            self._set_yaw(float(self.ent_yaw.get()))
        except ValueError:
            messagebox.showerror("Lỗi", "Yaw phải là số!")

    def _step_speed(self) -> float:
        return float(self.cb_step_v.get().split()[0])

    def _step_angle(self) -> float:
        return float(self.cb_step_a.get().replace("°", ""))

    def _drive_step(self, direction: int, turn: int):
        """direction: +1 tiến / -1 lùi; turn: -1 trái / 0 thẳng / +1 phải (đổi hướng theo góc xoay)."""
        if turn:
            self._nudge_yaw(turn * self._step_angle())
        cms = direction * self._step_speed()
        self._set_speed_ui(cms)
        self._send_speed_rpm(cms / WHEEL_RPM_TO_CMS)

    def _reset_position(self):
        self._send(build_packet(CMD_SET_YAW), "RESET YAW/ODOMETRY (0x06 len=0)")
        self.pos_x = self.pos_y = 0.0
        self.tgt_yaw = 0.0
        self._set_entry(self.ent_yaw, "0.0")
        for k in ("x", "y"):
            self.hist[k] = [0.0] * len(self.hist[k])

    def _send_mode(self):
        idx = max(0, self.cb_mode.current())
        self._set_mode_ui(idx)
        self._send(build_packet(CMD_SET_MODE, struct.pack("<B", idx)), f"MODE = {idx} ({self.MODE_NAMES[idx]})")
        if self.is_simulating:
            self._request_config()

    def _set_mode_ui(self, idx: int):
        self.ctrl_mode = 1 if idx == 1 else 0
        self.lbl_fz[1].config(text="FuzzyPD_Pos (ku: °)")
        self._update_fz_equiv()

    def _send_pitch(self):
        try:
            v = float(self.ent_pitch.get())
        except ValueError:
            messagebox.showerror("Lỗi", "Góc Pitch phải là số!")
            return
        v = max(-15.0, min(15.0, v))
        self.tgt_pitch = v
        self.var_pitch.set(v)
        self._set_entry(self.ent_pitch, f"{v:.2f}")
        self._send(build_packet(CMD_SET_ANGLE, struct.pack("<f", v)), f"PITCH đặt = {v:.2f}°")

    def _nudge_pitch(self, d):
        try:
            v = float(self.ent_pitch.get()) + d
        except ValueError:
            v = self.tgt_pitch + d
        self._set_entry(self.ent_pitch, f"{v:.2f}")
        self._send_pitch()

    def _nudge_pid(self, key, delta):
        e = self.pid_entries[key]
        try:
            v = float(e.get()) + delta
        except ValueError:
            v = delta
        self._set_entry(e, f"{max(0.0, v):.2f}")
        self._send_pid()

    def _send_pid(self):
        try:
            v = [float(self.pid_entries[k].get()) for k in ("kp_yaw", "ki_yaw", "kd_yaw")]
        except ValueError:
            messagebox.showerror("Lỗi", "Các hệ số PID phải là số!")
            return
        self._send(build_packet(CMD_SET_PID, struct.pack("<3f", *v)),
                   "PID Yaw: Kp=%.2f Ki=%.2f Kd=%.2f" % tuple(v))

    def _read_fuzzy(self):
        return [float(e.get().strip().replace(",", ".")) for row in self.fz for e in row]

    def _update_fz_equiv(self):
        try:
            a, b, c, d, e, f = self._read_fuzzy()
        except ValueError:
            self.lbl_equiv.config(text="(giá trị nhập chưa hợp lệ)", fg=self.C_RED)
            return
        pos_line = (f"  Pos (cascade): Kp ≈ {d * f * self.FZ_SLOPE_POS_E:6.3f} °/cm   "
                    f"Kd ≈ {e * f * self.FZ_SLOPE_POS_DE:6.3f} °/(cm/s)\n")
        self.lbl_equiv.config(fg=self.C_DIM, text=(
            "Tương đương gần đúng (|e|,|de| < 0.25):\n"
            f"  Pitch: Kp ≈ {a * c * self.FZ_SLOPE_PITCH_E:7.1f} PWM/°   Kd ≈ {b * c * self.FZ_SLOPE_PITCH_DE:6.2f} PWM/(°/s)\n"
            + pos_line +
            f"  Bão hòa khi |e_pitch| ≥ {1 / a if a else 0:.1f}°, |rate| ≥ {1 / b if b else 0:.0f}°/s, "
            f"|x_err| ≥ {1 / d if d else 0:.1f} cm, |v_err| ≥ {1 / e if e else 0:.1f} cm/s"))

    def _send_fuzzy(self):
        try:
            v = self._read_fuzzy()
        except ValueError:
            messagebox.showerror("Lỗi", "Hệ số Fuzzy phải là số!")
            return
        if any(x <= 0 for x in v[:5]) or v[5] < 0:
            messagebox.showerror("Lỗi", "ke, kde, ku phải > 0 (riêng ku_Pos được phép = 0 để tắt vòng ngoài).")
            return
        self._send(build_packet(CMD_SET_FUZZY, struct.pack("<6f", *v)),
                   "FUZZY: Pitch(%.5g,%.5g,%.5g) Pos(%.5g,%.5g,%.5g)" % tuple(v))

    def _disable_outer(self):
        self._set_entry(self.fz[1][2], "0")
        self._update_fz_equiv()
        self._send_fuzzy()

    # ============================================== JOYSTICK / BÀN PHÍM / TICK
    def _joy_from_pixel(self, ex, ey):
        dx, dy = ex - self.joy_c, ey - self.joy_c
        d = math.hypot(dx, dy)
        if d > self.joy_r:
            dx, dy = dx * self.joy_r / d, dy * self.joy_r / d
        self.joy_x = dx / self.joy_r
        self.joy_y = -dy / self.joy_r
        self._draw_knob()

    def _joy_press(self, e):
        self.joy_mouse = True
        self._joy_from_pixel(e.x, e.y)

    def _joy_drag(self, e):
        if self.joy_mouse:
            self._joy_from_pixel(e.x, e.y)

    def _joy_release(self, e=None):
        self.joy_mouse = False
        if not self.keys:
            self.joy_x = self.joy_y = 0.0
            self._draw_knob()

    def _draw_knob(self):
        c, r = self.joy_c, self.knob_r
        px, py = c + self.joy_x * self.joy_r, c - self.joy_y * self.joy_r
        active = abs(self.joy_x) > 0.04 or abs(self.joy_y) > 0.04
        col = self.C_GREEN if active else self.C_CYAN
        self.cv_joy.coords(self.joy_stem, c, c, px, py)
        self.cv_joy.itemconfig(self.joy_stem, fill=col)
        self.cv_joy.coords(self.joy_knob, px - r, py - r, px + r, py + r)
        self.cv_joy.itemconfig(self.joy_knob, fill=col)
        self.lbl_joy.config(text=f"Tiến/lùi: {self.joy_y * 100:+.0f}% | Rẽ: {self.joy_x * 100:+.0f}%", fg=col)

    def _typing(self) -> bool:
        return isinstance(self.root.focus_get(), (tk.Entry, ttk.Entry, ttk.Combobox, ScrolledText))

    def _on_key_press(self, ev):
        if not self.var_kb.get() or self._typing():
            return
        ks = ev.keysym.lower()
        if ks == "space":
            self._brake()
        elif ks in ("w", "a", "s", "d", "up", "down", "left", "right"):
            self.keys.add(ks)
            self._joy_from_keys()

    def _on_key_release(self, ev):
        ks = ev.keysym.lower()
        if ks in self.keys:
            self.keys.discard(ks)
            self._joy_from_keys()

    def _joy_from_keys(self):
        if self.joy_mouse:
            return
        y = (("w" in self.keys or "up" in self.keys) - ("s" in self.keys or "down" in self.keys)) * 0.8
        x = (("d" in self.keys or "right" in self.keys) - ("a" in self.keys or "left" in self.keys)) * 0.7
        self.joy_x, self.joy_y = x, y
        self._draw_knob()

    def _joystick_tick(self):
        if not self.is_running:
            return
        dt = 0.1
        active = abs(self.joy_x) > 0.05 or abs(self.joy_y) > 0.05
        if active and (self.is_connected or self.is_simulating):
            try:
                vmax = min(ROBOT_MAX_SPEED_CMS, abs(float(self.ent_joy_vmax.get())))
                turn_rate = abs(float(self.ent_joy_turn.get()))
            except ValueError:
                vmax, turn_rate = 6.0, 45.0
            cms = self.joy_y * vmax
            self._set_speed_ui(cms)
            self._send_speed_rpm(cms / WHEEL_RPM_TO_CMS, silent=True)
            if abs(self.joy_x) > 0.05:
                sign = -1.0 if self.var_invert_turn.get() else 1.0
                self.tgt_yaw += sign * self.joy_x * turn_rate * dt
                self._set_entry(self.ent_yaw, f"{self.tgt_yaw:.1f}")
                self._send_yaw(self.tgt_yaw, silent=True)
            self._joy_was_active = True
        elif self._joy_was_active:
            # Vừa thả cần: phanh và giữ hướng hiện tại
            self._joy_was_active = False
            self._set_speed_ui(0.0)
            self._send_speed_rpm(0.0, silent=True)
            self._send_yaw(self.cur["yaw"], silent=True)
            self._set_entry(self.ent_yaw, f"{self.tgt_yaw:.1f}")
        self.root.after(int(dt * 1000), self._joystick_tick)

    # ============================================================ DATA PIPELINE
    def _apply_config(self, f):
        (kp_yaw, ki_yaw, kd_yaw, tang, tyaw, mode,
         pdke, pdkde, pdku, pike, pikde, piku) = f
        mode_i = 1 if int(round(mode)) == 1 else 0
        self.cb_mode.current(mode_i)
        self._set_mode_ui(mode_i)
        vals = {"kp_yaw": (kp_yaw, ".2f"), "ki_yaw": (ki_yaw, ".2f"), "kd_yaw": (kd_yaw, ".2f")}
        for k, (v, fmt) in vals.items():
            self._set_entry(self.pid_entries[k], format(v, fmt))
        for e, v in zip([e for row in self.fz for e in row], (pdke, pdkde, pdku, pike, pikde, piku)):
            self._set_entry(e, f"{v:.6g}")
        self._update_fz_equiv()
        self.tgt_pitch = tang
        self.var_pitch.set(max(-15, min(15, tang)))
        self._set_entry(self.ent_pitch, f"{tang:.2f}")
        self.tgt_yaw = tyaw
        self._set_entry(self.ent_yaw, f"{tyaw:.1f}")
        self._log(f"[STM32→PC] Cấu hình: mode={self.MODE_NAMES[mode_i]} Yaw(Kp={kp_yaw:.1f},Ki={ki_yaw:.2f},Kd={kd_yaw:.2f}) "
                  f"Pitch({pdke:.4g},{pdkde:.4g},{pdku:.5g}) Pos({pike:.4g},{pikde:.4g},{piku:.5g}) Pitch đặt={tang:.2f}°\n")

    def _process_incoming(self):
        if not self.is_running:
            return
        text = []
        while not self.log_queue.empty() and len(text) < 30:
            text.append(self.log_queue.get_nowait())
        if text:
            self._log("".join(text))
        while not self.config_queue.empty():
            self._apply_config(self.config_queue.get_nowait())

        n = 0
        while not self.data_queue.empty() and n < 250:
            ts, f = self.data_queue.get_nowait()
            n += 1
            d = dict(zip(TELEM_FIELDS, f))
            dt = 0.02 if self.last_pkt_ts is None else max(0.001, min(0.1, ts - self.last_pkt_ts))
            self.last_pkt_ts = ts
            # Vận tốc tịnh tiến (xe tiến => tốc độ encoder âm) & odometry
            v_cms = -0.5 * (d["kal_L"] + d["kal_R"]) * WHEEL_RPM_TO_CMS
            th = math.radians(d["yaw"])
            self.pos_x += v_cms * math.cos(th) * dt
            self.pos_y += v_cms * math.sin(th) * dt
            d.update(x=self.pos_x, y=self.pos_y, speed_cms=v_cms,
                     speed_cmd_cms=self.tgt_speed_rpm * WHEEL_RPM_TO_CMS if self.robot_state == "RUN" else 0.0,
                     tgt_yaw=self.tgt_yaw, tilt=d["out_pos"], t=ts - self.start_ts)
            for k in ALL_FIELDS:
                self.hist[k].append(d[k])
            self.cur = d
            self.has_telem = True
            self.rx_count += 1
            self.freq_count += 1

        if len(self.hist["t"]) > 9000:
            for k in ALL_FIELDS:
                del self.hist[k][:1500]

        now = time.time()
        if n and (now - self._last_tile_t >= 0.08):
            self._last_tile_t = now
            self._update_tiles()
        if now - self.freq_time >= 1.0:
            self.lbl_rate.config(text=f"{self.freq_count / (now - self.freq_time):.1f} Hz | {self.rx_count} pkts | "
                                      f"RX {self.rx_bytes} B | lỗi CS {self.rx_bad}")
            # Chẩn đoán mỗi 5 s khi đang kết nối thật mà không có gói hợp lệ
            if self.is_connected and now - self._diag_t >= 5.0:
                d_bytes = self.rx_bytes - self._diag_bytes
                d_ok = self.rx_count - self._diag_ok
                if d_ok == 0:
                    if d_bytes == 0:
                        self._log("[CHẨN ĐOÁN] 5 s qua KHÔNG nhận được byte nào: kiểm tra đúng cổng COM/baud (115200), nguồn xe, "
                                  "dây TX/RX (hoặc module Bluetooth), và firmware có đang chạy (LED nhấp nháy).\n")
                    else:
                        self._log(f"[CHẨN ĐOÁN] 5 s qua nhận {d_bytes} byte nhưng KHÔNG có gói hợp lệ (khung sai checksum: {self.rx_bad}). "
                                  "Có thể firmware khác phiên bản giao thức (độ dài gói/checksum) hoặc sai baudrate.\n")
                self._diag_bytes, self._diag_ok, self._diag_t = self.rx_bytes, self.rx_count, now
            self.freq_count = 0
            self.freq_time = now
        self.root.after(20, self._process_incoming)

    @staticmethod
    def _set_lbl(lbl: tk.Label, text: str, fg: Optional[str] = None):
        prev = getattr(lbl, "_last_cfg", None)
        cur_cfg = (text, fg)
        if prev != cur_cfg:
            lbl._last_cfg = cur_cfg
            if fg is not None:
                lbl.config(text=text, fg=fg)
            else:
                lbl.config(text=text)

    def _update_tiles(self):
        c = self.cur
        tab = self.notebook.index(self.notebook.select())
        if tab == 0:
            e_p, e_y = abs(c["pitch"] - c["tgt_ang"]), abs(c["yaw"] - self.tgt_yaw)
            self._set_lbl(self.t_pitch, f"{c['raw_pitch']:+.2f}° | {c['pitch']:+.2f}°",
                          self.C_CYAN if e_p < 5 else (self.C_YELLOW if e_p < 12 else self.C_RED))
            self._set_lbl(self.t_yaw, f"{c['raw_yaw']:+.1f}° | {c['yaw']:+.1f}°")
            self._set_lbl(self.t_rate, f"{c['pitch_rate']:+.1f} | {c['yaw_rate']:+.1f} °/s")
            self._set_lbl(self.t_err, f"{e_p:.2f}° | {e_y:.1f}°")
            self._set_lbl(self.t_pwm, f"{c['pwm_L']:+.0f} | {c['pwm_R']:+.0f}")
            self._set_lbl(self.t_pwm_pct, f"{c['pwm_L'] / PWM_MAX * 100:+.1f}% | {c['pwm_R'] / PWM_MAX * 100:+.1f}%")
            self._set_lbl(self.t_wheel_raw, f"{c['raw_L']:+.1f} | {c['raw_R']:+.1f}")
            self._set_lbl(self.t_wheel_kal, f"{c['kal_L']:+.1f} | {c['kal_R']:+.1f}")
            self._set_lbl(self.t_speed, f"{c['speed_cms']:+.2f}")
            self._set_lbl(self.t_pwm_parts, f"{c['out_fuzzy']:+.0f} | {c['out_yaw']:+.0f}")
            self._set_lbl(self.t_pos, f"{self.pos_x:+.1f} | {self.pos_y:+.1f}")
            self._set_lbl(self.t_dist, f"{math.hypot(self.pos_x, self.pos_y):.1f}")
        elif tab == 1:
            self._set_lbl(self.d_speed, f"{c['speed_cmd_cms']:+.1f} | {c['speed_cms']:+.1f}")
            self._set_lbl(self.d_yaw, f"{self.tgt_yaw:+.1f} | {c['yaw']:+.1f}")
            self._set_lbl(self.d_pos, f"{self.pos_x:+.1f} | {self.pos_y:+.1f}")
            self._set_lbl(self.d_pitch, f"{c['pitch']:+.2f}")
            wl, wr = -c["kal_L"], -c["kal_R"]
            self._set_lbl(self.d_whl_L, f"{wl:+.1f} | {wl * WHEEL_RPM_TO_CMS:+.2f}")
            self._set_lbl(self.d_whl_R, f"{wr:+.1f} | {wr * WHEEL_RPM_TO_CMS:+.2f}")
        else:
            sat = lambda v: abs(v) >= 0.999
            self._set_lbl(self.f_vel, f"{-c['vel_cmd']:+.1f} | {-0.5 * (c['kal_L'] + c['kal_R']):+.1f}")
            self._set_lbl(self.f_tilt, f"{c['tgt_ang']:+.2f} | {c['out_pos']:+.2f}°",
                          self.C_RED if abs(c["out_pos"]) >= self.TILT_MAX_DEG - 0.01 else self.C_ORANGE)
            self._set_lbl(self.f_pd, f"{c['pitch_e']:+.2f} | {c['pitch_de']:+.2f}",
                          self.C_RED if sat(c["pitch_e"]) or sat(c["pitch_de"]) else self.C_CYAN)
            self._set_lbl(self.f_pi, f"{c['pos_e']:+.2f} | {c['pos_de']:+.2f}  ({c['pos_err']:+.1f} cm)",
                          self.C_RED if sat(c["pos_e"]) or sat(c["pos_de"]) else self.C_YELLOW)
            self._set_lbl(self.f_out, f"{c['out_fuzzy']:+.0f}",
                          self.C_RED if abs(c["out_fuzzy"]) >= 7559 else self.C_CYAN)
            k = min(150, len(self.hist["pitch_e"]))
            if k:
                pd_s = sum(1 for a, b in zip(self.hist["pitch_e"][-k:], self.hist["pitch_de"][-k:]) if sat(a) or sat(b)) * 100 / k
                pi_s = sum(1 for a, b in zip(self.hist["pos_e"][-k:], self.hist["pos_de"][-k:]) if sat(a) or sat(b)) * 100 / k
                m = max(pd_s, pi_s)
                self._set_lbl(self.f_sat, f"Pitch {pd_s:.0f}% | Pos {pi_s:.0f}%",
                              self.C_RED if m > 30 else (self.C_YELLOW if m > 5 else self.C_GREEN))

    def _log(self, text: str):
        self.txt.insert(tk.END, text)
        try:
            lines = int(self.txt.index("end-1c").split(".")[0])
            if lines > 250:
                self.txt.delete("1.0", f"{lines - 200}.0")
        except Exception:
            pass
        self.txt.see(tk.END)

    # ================================================================== PLOTS
    @staticmethod
    def _ylim(ax, vals, min_span):
        if not vals:
            return
        lo, hi = min(vals), max(vals)
        span = max(min_span, (hi - lo) * 1.3)
        mid = 0.5 * (lo + hi)
        cl, ch = ax.get_ylim()
        if lo < cl + 0.05 * (ch - cl) or hi > ch - 0.05 * (ch - cl) or span < (ch - cl) * 0.5:
            ax.set_ylim(mid - span / 2, mid + span / 2)

    def _update_plots(self):
        if not self.is_running:
            return
        t_all = self.hist["t"]
        if not self.is_paused and len(t_all) > 1:
            t_max = max(self.window_s, math.ceil((t_all[-1] + 0.2) * 2) * 0.5)
            t_min = max(0.0, t_max - self.window_s)
            i0 = bisect.bisect_left(t_all, t_min)
            n_pts = len(t_all) - i0
            step = max(1, n_pts // 450)
            t = t_all[i0::step]
            h = lambda k: self.hist[k][i0::step]
            tab = self.notebook.index(self.notebook.select())

            def setx(axes):
                for ax in axes:
                    cx0, cx1 = ax.get_xlim()
                    if abs(cx0 - t_min) > 1e-3 or abs(cx1 - t_max) > 1e-3:
                        ax.set_xlim(t_min, t_max)

            if tab == 0:
                a1, a2, a3 = self.axes1
                rp, kp, ta = h("raw_pitch"), h("pitch"), h("tgt_ang")
                ry, ky, ty = h("raw_yaw"), h("yaw"), h("tgt_yaw")
                pl = [p / PWM_MAX * 100 for p in h("pwm_L")]
                pr = [p / PWM_MAX * 100 for p in h("pwm_R")]
                sc = h("speed_cms")
                self.l_raw_pitch.set_data(t, rp)
                self.l_pitch.set_data(t, kp)
                self.l_tgt_ang.set_data(t, ta)
                self.l_raw_yaw.set_data(t, ry)
                self.l_yaw.set_data(t, ky)
                self.l_tgt_yaw.set_data(t, ty)
                self.l_pwm_L.set_data(t, pl)
                self.l_pwm_R.set_data(t, pr)
                self.l_spd_cms.set_data(t, sc)
                setx(self.axes1)
                self._ylim(a1, rp + kp + ta, 8)
                self._ylim(a2, ry + ky + ty, 10)
                self._ylim(a3, pl + pr + sc, 25)
                self.canvas1.draw_idle()
            elif tab == 1:
                m, s, w, y = self.axes2
                tx, ty = self.hist["x"][-600::2], self.hist["y"][-600::2]
                self.l_traj.set_data(tx, ty)
                self.l_robot.set_data([self.pos_x], [self.pos_y])
                th = math.radians(self.cur["yaw"])
                ch, sh = math.cos(th), math.sin(th)
                self.l_head.set_data([self.pos_x, self.pos_x + 8 * ch], [self.pos_y, self.pos_y + 8 * sh])
                hw = ROBOT_WHEELBASE_CM / 2
                self.l_body.set_data([self.pos_x - hw * sh, self.pos_x + hw * sh],
                                     [self.pos_y + hw * ch, self.pos_y - hw * ch])
                ext = max(30.0, max((abs(v) for v in tx + ty), default=0) + 15)
                ext = math.ceil(ext / 10) * 10
                cx0, cx1 = m.get_xlim()
                if abs(cx1 - ext) > 1e-2 or abs(cx0 + ext) > 1e-2:
                    m.set_xlim(-ext, ext)
                    m.set_ylim(-ext, ext)
                sc_cmd, sc_meas = h("speed_cmd_cms"), h("speed_cms")
                self.l_v_cmd.set_data(t, sc_cmd)
                self.l_v_meas.set_data(t, sc_meas)
                # Tốc độ từng bánh: đổi dấu để tiến = dương (firmware báo tiến = âm)
                wl_raw = [-v for v in h("raw_L")]
                wr_raw = [-v for v in h("raw_R")]
                wl = [-v for v in h("kal_L")]
                wr = [-v for v in h("kal_R")]
                ty_h, y_h = h("tgt_yaw"), h("yaw")
                self.l_wL_raw.set_data(t, wl_raw)
                self.l_wR_raw.set_data(t, wr_raw)
                self.l_wL.set_data(t, wl)
                self.l_wR.set_data(t, wr)
                self.l_y_cmd.set_data(t, ty_h)
                self.l_y_meas.set_data(t, y_h)
                setx((s, w, y))
                self._ylim(s, sc_cmd + sc_meas, 4)
                self._ylim(w, wl + wr, 10)
                self._ylim(y, ty_h + y_h, 20)
                self.canvas2.draw_idle()
            else:
                p, v, pd, pi = self.axes3
                kp, ta = h("pitch"), h("tgt_ang")
                vcmd = [-x for x in h("vel_cmd")]
                vmeas = [-0.5 * (a + b) for a, b in zip(h("kal_L"), h("kal_R"))]
                self.l3_pitch.set_data(t, kp)
                self.l3_tgt.set_data(t, ta)
                self.l3_base.set_ydata([self.tgt_pitch, self.tgt_pitch])
                self.l3_vcmd.set_data(t, vcmd)
                self.l3_vmeas.set_data(t, vmeas)
                self.l3_pd_e.set_data(t, h("pitch_e"))
                self.l3_pd_de.set_data(t, h("pitch_de"))
                self.l3_pi_e.set_data(t, h("pos_e"))
                self.l3_pi_de.set_data(t, h("pos_de"))
                setx(self.axes3)
                self._ylim(p, kp + ta, 6)
                self._ylim(v, vcmd + vmeas, 10)
                self.canvas3.draw_idle()
        self.root.after(110, self._update_plots)

    def _on_window(self, s: str):
        try:
            self.window_s = float(s.replace("s", ""))
        except ValueError:
            self.window_s = 15.0

    def _toggle_pause(self):
        self.is_paused = not self.is_paused
        for b in self.pause_buttons:
            b.config(text="▶ Tiếp tục" if self.is_paused else "⏸ Tạm dừng",
                     bg="#1565c0" if self.is_paused else self.C_BORDER)

    def _clear_data(self):
        for k in ALL_FIELDS:
            self.hist[k].clear()
        self.start_ts = time.time()
        self.last_pkt_ts = None
        self.pos_x = self.pos_y = 0.0
        for cv in (self.canvas1, self.canvas2, self.canvas3):
            cv.draw_idle()
        self._log("[ĐỒ THỊ] Đã xóa lịch sử dữ liệu & quỹ đạo.\n")

    def _export_csv(self):
        if not self.hist["t"]:
            messagebox.showinfo("Thông báo", "Chưa có dữ liệu để xuất!")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv", filetypes=[("CSV", "*.csv")],
            initialfile=f"robot_log_{datetime.now():%Y%m%d_%H%M%S}.csv", title="Lưu dữ liệu")
        if not path:
            return
        try:
            with open(path, "w", newline="", encoding="utf-8") as fh:
                w = csv.writer(fh)
                w.writerow(ALL_FIELDS)
                for i in range(len(self.hist["t"])):
                    w.writerow([f"{self.hist[k][i]:.4f}" for k in ALL_FIELDS])
            messagebox.showinfo("Thành công", f"Đã xuất {len(self.hist['t'])} mẫu ra:\n{path}")
        except Exception as e:
            messagebox.showerror("Lỗi lưu file", str(e))

    def _on_close(self):
        self.is_running = False
        self.is_simulating = False
        try:
            if self.is_connected:
                self._send_speed_rpm(0.0, silent=True)
                time.sleep(0.05)
        except Exception:
            pass
        self._disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    RobotControlGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
