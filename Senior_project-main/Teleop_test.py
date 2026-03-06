import socket
import struct
import tkinter as tk
import time

# ==========================================
# CONFIGURATION
# ==========================================
ESP32_IP = "myrobot.local"  # หรือ IP 192.168.1.100 ของนาย
UDP_PORT = 8888

MAX_SPEED_MS = 0.3
MAX_OMEGA = 1.2

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

state = {
    'v_req': 0.0, 'w_req': 0.0,
    'btn_dpad': 0, 'btn_cancel': 0,
    'btn_toggle': 0, 'btn_reset': 0
}

# สร้าง Dictionary เก็บสถานะการหน่วงเวลาปล่อยปุ่ม
pending_releases = {}

# ==========================================
# KEYBOARD MAPPINGS (แก้ปัญหาปุ่มกดค้าง)
# ==========================================
def key_press(event):
    key = event.keysym.lower()
    
    # --- สำคัญตรงนี้ ---
    # ถ้ามีการหน่วงเวลาเตรียมจะ "ปล่อยปุ่ม" นี้อยู่ ให้ยกเลิกซะ เพราะผู้ใช้ยังกดค้างอยู่!
    if key in pending_releases:
        root.after_cancel(pending_releases[key])
        del pending_releases[key]

    # บังคับทิศทาง
    if key == 'w': state['v_req'] = MAX_SPEED_MS
    elif key == 's': state['v_req'] = -MAX_SPEED_MS
    elif key == 'a': state['w_req'] = MAX_OMEGA
    elif key == 'd': state['w_req'] = -MAX_OMEGA
    
    # ระยะทาง และโหมด
    elif key == 'up': state['btn_dpad'] = 1
    elif key == 'right': state['btn_dpad'] = 2
    elif key == 'down': state['btn_dpad'] = 3
    elif key == 'left': state['btn_dpad'] = 4
    elif key == 'c': state['btn_cancel'] = 1
    elif key == 'm': state['btn_toggle'] = 1
    elif key == 'r': state['btn_reset'] = 1
    
    update_ui()

def process_release(key):
    # ฟังก์ชันนี้จะทำงานจริงๆ เมื่อผู้ใช้ "ปล่อยปุ่ม" เกิน 50ms แล้ว
    if key in ['w', 's']: state['v_req'] = 0.0
    if key in ['a', 'd']: state['w_req'] = 0.0
    if key in ['up', 'right', 'down', 'left']: state['btn_dpad'] = 0
    if key == 'c': state['btn_cancel'] = 0
    if key == 'm': state['btn_toggle'] = 0
    if key == 'r': state['btn_reset'] = 0
    update_ui()
    
    if key in pending_releases:
        del pending_releases[key]

def key_release(event):
    key = event.keysym.lower()
    # แทนที่จะเคลียร์ค่าเป็น 0 ทันที ให้หน่วงเวลาไป 50ms 
    # (ถ้า Auto-repeat ทำงาน มันจะเรียก key_press มายกเลิกไอ้นี่ได้ทันพอดี)
    pending_releases[key] = root.after(50, lambda: process_release(key))

# ==========================================
# UDP SENDER LOOP
# ==========================================
def send_udp_packet():
    packet = struct.pack('<ffBBBB', 
                         state['v_req'], state['w_req'], 
                         state['btn_dpad'], state['btn_cancel'], 
                         state['btn_toggle'], state['btn_reset'])
    try:
        sock.sendto(packet, (ESP32_IP, UDP_PORT))
    except Exception as e:
        print(f"UDP Error: {e}")
        
    root.after(50, send_udp_packet)

# ==========================================
# GUI SETUP
# ==========================================
root = tk.Tk()
root.title("ESP32 Robot Teleop")
root.geometry("400x300")

tk.Label(root, text="Robot WiFi Controller (ROS2 Prep)", font=('Arial', 14, 'bold')).pack(pady=10)
tk.Label(root, text="W/A/S/D : บังคับทิศทาง (กดค้างได้เลย)").pack()
tk.Label(root, text="Arrow Keys : ตั้งระยะทางวิ่ง (5m, 10m, 15m, 20m)").pack()
tk.Label(root, text="M : สลับโหมด (Auto/Manual) | C : ยกเลิก | R : รีเซ็ตระยะ").pack(pady=10)

status_lbl = tk.Label(root, text="V: 0.0 m/s | W: 0.0 rad/s", font=('Arial', 12, 'bold'), fg="blue")
status_lbl.pack(pady=10)

def update_ui():
    status_lbl.config(text=f"V: {state['v_req']:.2f} m/s | W: {state['w_req']:.2f} rad/s")

root.bind('<KeyPress>', key_press)
root.bind('<KeyRelease>', key_release)

send_udp_packet()
root.mainloop()
