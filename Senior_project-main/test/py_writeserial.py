import serial
import tkinter as tk
from tkinter import ttk
import re  # ใช้สำหรับดึงตัวเลขออกจากข้อความ (Regex)

# --- การตั้งค่า Serial ---
SERIAL_PORT = 'COM5' 
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01) # ลด timeout เพื่อความลื่นไหล
    print(f"Connected to {SERIAL_PORT}")
except:
    print("Error: Could not open serial port")
    exit()

def send_data(event=None):
    """ อ่านค่าจาก Slider และส่งออกทาง Serial """
    v_val = int(v_slider.get())
    w_val = int(w_slider.get())
    cmd = f"{v_val},{w_val}\n"
    ser.write(cmd.encode())
    value_label.config(text=f"Sending -> V: {v_val}, W: {w_val}")

def read_serial():
    """ ฟังก์ชันอ่านค่าจาก Serial และอัปเดต UI (เรียกตัวเองซ้ำทุก 50ms) """
    if ser.in_waiting > 0:
        try:
            # อ่านข้อมูลหนึ่งบรรทัด
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            # ตรวจสอบว่ามีข้อความ RPM หรือไม่
            if "RPM1:" in line:
                # ใช้ Regular Expression ดึงตัวเลขทศนิยม/เต็ม ออกมา
                # หาตัวเลขที่ตามหลัง RPM1: และ RPM2:
                numbers = re.findall(r"[-+]?\d*\.\d+|\d+", line)
                
                if len(numbers) >= 2:
                    rpm1_label.config(text=f"Motor 1: {numbers[0]} RPM", foreground="#00aa00")
                    rpm2_label.config(text=f"Motor 2: {numbers[1]} RPM", foreground="#00aa00")
        except Exception as e:
            print(f"Read error: {e}")

    # ตั้งเวลาให้กลับมาทำงานฟังก์ชันนี้ใหม่ในอีก 50 มิลลิวินาที
    root.after(50, read_serial)

def stop_robot():
    v_slider.set(0)
    w_slider.set(0)
    send_data()

# --- สร้างหน้าต่าง GUI ---
root = tk.Tk()
root.title("BTS7960 Control & Monitoring")
root.geometry("450x450")

main_frame = ttk.Frame(root, padding="20")
main_frame.pack(expand=True, fill="both")

ttk.Label(main_frame, text="Robot Control Dashboard", font=("Helvetica", 16, "bold")).pack(pady=10)

# --- ส่วนควบคุม (Sliders) ---
ttk.Label(main_frame, text="Linear Velocity (V)").pack()
v_slider = ttk.Scale(main_frame, from_=-255, to=255, orient="horizontal", command=send_data)
v_slider.pack(fill="x", padx=20, pady=5)

ttk.Label(main_frame, text="Angular Velocity (W)").pack()
w_slider = ttk.Scale(main_frame, from_=-255, to=255, orient="horizontal", command=send_data)
w_slider.pack(fill="x", padx=20, pady=5)

value_label = ttk.Label(main_frame, text="Sending -> V: 0, W: 0", font=("Courier", 10))
value_label.pack(pady=10)

ttk.Separator(main_frame, orient='horizontal').pack(fill='x', pady=15)

# --- ส่วนแสดงผล Feedback (RPM) ---
ttk.Label(main_frame, text="Real-time Feedback", font=("Helvetica", 12, "bold")).pack()
rpm_frame = ttk.Frame(main_frame)
rpm_frame.pack(pady=10)

rpm1_label = ttk.Label(rpm_frame, text="Motor 1: 0.00 RPM", font=("Courier", 14))
rpm1_label.pack()

rpm2_label = ttk.Label(rpm_frame, text="Motor 2: 0.00 RPM", font=("Courier", 14))
rpm2_label.pack()

# ปุ่มหยุด
stop_btn = ttk.Button(main_frame, text="STOP ALL", command=stop_robot)
stop_btn.pack(pady=20)

def on_closing():
    ser.write(b"0,0\n")
    ser.close()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_closing)

# เริ่มต้นลูปการอ่าน Serial
root.after(100, read_serial)
root.mainloop()