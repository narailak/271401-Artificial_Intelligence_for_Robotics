"""
HW_01: จำลองระบบควบคุมอุณหภูมิห้องด้วยแอร์แบบ Closed Loop (Real-time Interactive Plot)
========================================================================================
Plant (แอร์):        G(s) = K / (tau*s + 1)
Sensor (feedback):    H(s) = 1

สามารถปรับค่า K, tau และ Setpoint (R0) ได้แบบเรียลไทม์ผ่าน Slider
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from scipy import signal

# ตั้งค่า font ให้รองรับภาษาไทยในกราฟ
plt.rcParams["font.family"] = "sans-serif"
plt.rcParams["font.sans-serif"] = ["Tahoma", "Loma", "Sarabun", "Microsoft Sans Serif", "Arial"]

# ฟังก์ชันคำนวณ step response ของระบบวงปิด
def closed_loop_step_response(K, tau, R0=5.0, t_end=15.0, n=1000):
    num = [K]
    den = [tau, 1 + K]
    system = signal.TransferFunction(num, den)
    t = np.linspace(0, t_end, n)
    _, y_out = signal.step(system, T=t)
    return y_out * R0

# กำหนดค่าเริ่มต้น
K_init = 5.0
tau_init = 2.0
R0_init = 5.0
t_end = 15.0

# คำนวณสัญญาณเริ่มต้น
t_data = np.linspace(0, t_end, 1000)
y_data = closed_loop_step_response(K_init, tau_init, R0_init, t_end=t_end)

# สร้างหน้าต่างกราฟ
fig, ax = plt.subplots(figsize=(9, 6))
plt.subplots_adjust(bottom=0.32)  # เหลือพื้นที่ด้านล่างสำหรับใส่สไลเดอร์และข้อมูลสรุป

# วาดเส้นกราฟเริ่มต้น
line, = ax.plot(t_data, y_data, label="Closed Loop Response (y)", color="steelblue", linewidth=2.5)
setpoint_line = ax.axhline(R0_init, color="red", linestyle=":", linewidth=1.5, label="Setpoint (R0)")
ss_line = ax.axhline(R0_init * K_init / (1 + K_init), color="gray", linestyle="--", linewidth=1.5, label="Steady State (y_ss)")

# ตกแต่งกราฟ
ax.set_title("ระบบควบคุมอุณหภูมิแอร์แบบ Closed Loop (ปรับค่า Real-time)", fontsize=12, fontweight="bold")
ax.set_xlabel("เวลา (วินาที)")
ax.set_ylabel("อุณหภูมิห้องที่เปลี่ยนแปลง")
ax.grid(True, alpha=0.3)
ax.legend(loc="upper right")
ax.set_xlim(0, t_end)
ax.set_ylim(0, R0_init * 1.2)

# กำหนดตำแหน่งแกนสำหรับ Sliders [left, bottom, width, height]
ax_K = plt.axes([0.2, 0.20, 0.65, 0.03])
ax_tau = plt.axes([0.2, 0.14, 0.65, 0.03])
ax_R0 = plt.axes([0.2, 0.08, 0.65, 0.03])

# สร้าง Sliders
slider_K = Slider(ax_K, 'Gain (K)', 1.0, 10.0, valinit=K_init, valfmt='%1.1f', color='teal')
slider_tau = Slider(ax_tau, 'Time Const (tau)', 0.1, 10.0, valinit=tau_init, valfmt='%1.1f', color='orange')
slider_R0 = Slider(ax_R0, 'Setpoint (R0)', 1.0, 10.0, valinit=R0_init, valfmt='%1.1f', color='crimson')

# กล่องข้อความแสดงผลการคำนวณด้านล่าง
info_text = fig.text(0.15, 0.02, '', fontsize=10, color='darkblue', fontweight='bold')

# ฟังก์ชันอัปเดตข้อมูลเมื่อมีการปรับสไลเดอร์
def update(val):
    K = slider_K.val
    tau = slider_tau.val
    R0 = slider_R0.val
    
    # คำนวณค่าผลลัพธ์ใหม่
    y_new = closed_loop_step_response(K, tau, R0, t_end=t_end)
    tau_prime = tau / (1 + K)
    y_ss = R0 * K / (1 + K)
    ss_error = R0 - y_ss
    
    # อัปเดตตำแหน่งเส้นกราฟ
    line.set_ydata(y_new)
    setpoint_line.set_ydata([R0, R0])
    ss_line.set_ydata([y_ss, y_ss])
    
    # อัปเดตขอบเขตแกน Y ตามขนาดของ Setpoint
    ax.set_ylim(0, R0 * 1.2)
    
    # อัปเดตข้อความรายละเอียด
    info_text.set_text(
        f"Steady State (y_ss): {y_ss:.2f} | "
        f"Steady-State Error: {ss_error:.2f} | "
        f"New Time Const (tau'): {tau_prime:.2f}s (เร็วขึ้น {(tau/tau_prime):.1f} เท่า)"
    )
    
    fig.canvas.draw_idle()

# ผูกเหตุการณ์การเปลี่ยนแปลงค่าสไลเดอร์
slider_K.on_changed(update)
slider_tau.on_changed(update)
slider_R0.on_changed(update)

# เรียกใช้ฟังก์ชันอัปเดตเพื่อแสดงค่าเริ่มต้น
update(None)

plt.show()