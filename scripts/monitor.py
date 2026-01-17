import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import threading
import time
from collections import deque
import csv
import datetime
import sys

# Windows resolution scaling fix
try:
    from ctypes import windll
    windll.shcore.SetProcessDpiAwareness(1)
except:
    pass

class WingSailMonitor:
    def __init__(self, root):
        self.root = root
        self.root.title("Wing Sail Monitor & Control")
        
        # Default geometry, but window is resizable
        self.root.geometry("1400x900")
        
        # --- Internal State ---
        self.serial_port = None
        self.running = False
        self.reading_thread = None
        self.data_lock = threading.Lock()
        
        # --- Data Buffers ---
        self.max_points = 500
        self.times = deque(maxlen=self.max_points)
        self.yaw_target = deque(maxlen=self.max_points)
        self.yaw_meas = deque(maxlen=self.max_points)
        self.gyro_rate = deque(maxlen=self.max_points)
        self.servo_angle = deque(maxlen=self.max_points)
        self.servo_pwm = deque(maxlen=self.max_points)
        
        # CSV Logging
        self.csv_file = None
        self.csv_writer = None
        
        self.font_style = ("Consolas", 10)
        
        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        self.setup_ui()
        
    def on_closing(self):
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except:
                pass
        if self.csv_file:
            try:
                self.csv_file.close()
            except:
                pass
        self.root.destroy()
        sys.exit(0)

    def setup_ui(self):
        # Configure grid layout for responsiveness
        self.root.columnconfigure(0, weight=0, minsize=300) # Sidebar fixed min width
        self.root.columnconfigure(1, weight=1) # Main Plot Area expands
        self.root.rowconfigure(0, weight=1)

        # --- Sidebar (Controls & Console) ---
        sidebar = ttk.Frame(self.root, padding="10")
        sidebar.grid(row=0, column=0, sticky="nsew")
        
        # 1. Connection Group
        conn_frame = ttk.LabelFrame(sidebar, text="Connection", padding="5")
        conn_frame.pack(fill="x", pady=5)
        
        ttk.Label(conn_frame, text="Port:").pack(anchor="w")
        self.combo_port = ttk.Combobox(conn_frame)
        self.combo_port.pack(fill="x", pady=2)
        self.combo_port.bind("<Button-1>", self.refresh_ports)
        
        ttk.Label(conn_frame, text="Baudrate:").pack(anchor="w")
        self.combo_baud = ttk.Combobox(conn_frame, values=["9600", "115200", "460800", "921600"])
        self.combo_baud.current(1) # Default 115200
        self.combo_baud.pack(fill="x", pady=2)
        
        self.btn_connect = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.btn_connect.pack(fill="x", pady=5)
        
        # 2. Command Group
        cmd_frame = ttk.LabelFrame(sidebar, text="Command", padding="5")
        cmd_frame.pack(fill="x", pady=5)
        
        self.entry_cmd = ttk.Entry(cmd_frame, font=self.font_style)
        self.entry_cmd.pack(fill="x", pady=2)
        self.entry_cmd.bind("<Return>", self.send_command)
        
        self.btn_send = ttk.Button(cmd_frame, text="Send", command=self.send_command)
        self.btn_send.pack(fill="x", pady=2)
        
        # Quick Commands
        btn_grid = ttk.Frame(cmd_frame)
        btn_grid.pack(fill="x", pady=2)
        
        ttk.Button(btn_grid, text="Help (?)", width=10, command=lambda: self.send_cmd_str("?")).grid(row=0, column=0, padx=2, pady=2)
        ttk.Button(btn_grid, text="Manual", width=10, command=lambda: self.send_cmd_str("m 0")).grid(row=0, column=1, padx=2, pady=2)
        ttk.Button(btn_grid, text="Auto", width=10, command=lambda: self.send_cmd_str("m 1")).grid(row=1, column=1, padx=2, pady=2)
        ttk.Button(btn_grid, text="Target 0", width=10, command=lambda: self.send_cmd_str("t 0")).grid(row=1, column=0, padx=2, pady=2)
        
        # 3. Console Display
        console_frame = ttk.LabelFrame(sidebar, text="Log Output", padding="5")
        console_frame.pack(fill="both", expand=True, pady=5)
        
        self.txt_console = scrolledtext.ScrolledText(console_frame, height=10, font=("Consolas", 9), state='disabled')
        self.txt_console.pack(fill="both", expand=True)

        # 4. View Control
        view_frame = ttk.LabelFrame(sidebar, text="View Settings", padding="5")
        view_frame.pack(fill="x", pady=5)
        
        # Points count
        ttk.Label(view_frame, text="Window Points:").pack(anchor="w")
        self.entry_points = ttk.Entry(view_frame)
        self.entry_points.insert(0, "500")
        self.entry_points.pack(fill="x", pady=2)
        ttk.Button(view_frame, text="Apply", command=self.update_window_size).pack(fill="x")
        
        ttk.Button(view_frame, text="Clear Plots", command=self.clear_data).pack(fill="x", pady=5)

        # 5. Capture & Export
        cap_frame = ttk.LabelFrame(sidebar, text="Data & Capture", padding="5")
        cap_frame.pack(fill="x", pady=5)
        
        ttk.Button(cap_frame, text="Save Screenshot", command=self.save_screenshot).pack(fill="x", pady=2)
        ttk.Button(cap_frame, text="Save Console Log", command=self.save_console_log).pack(fill="x", pady=2)
        ttk.Label(cap_frame, text="(CSV Data Auto-saved)", font=("Arial", 8), foreground="gray").pack(pady=2)

        # --- Main Area (Plots) ---
        plot_frame = ttk.Frame(self.root, padding="5")
        plot_frame.grid(row=0, column=1, sticky="nsew")
        
        self.setup_plots(plot_frame)
        
        # Initial port scan
        self.refresh_ports()

    def setup_plots(self, parent):
        plt.style.use('dark_background')
        # Use subplots
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, sharex=True)
        self.fig.subplots_adjust(hspace=0.15, left=0.06, right=0.94, top=0.95, bottom=0.08)
        
        # Plot 1: Yaw (Target vs Measured)
        self.ax1.set_title("Yaw Control")
        self.ln_target, = self.ax1.plot([], [], 'r--', label='Target')
        self.ln_meas, = self.ax1.plot([], [], 'g-', label='Measured', linewidth=1.5)
        self.ax1.set_ylabel("Angle (deg)")
        self.ax1.legend(loc='upper right')
        self.ax1.grid(True, alpha=0.3)
        
        # Plot 2: Gyro Rate
        self.ax2.set_title("Gyro Rate")
        self.ln_rate, = self.ax2.plot([], [], 'y-', label='Rate')
        self.ax2.set_ylabel("Rate (deg/s)")
        self.ax2.grid(True, alpha=0.3)
        
        # Plot 3: Servo
        self.ax3.set_title("Servo Output")
        self.ln_servo_angle, = self.ax3.plot([], [], 'c-', label='Angle')
        self.ax3.set_ylabel("Servo (deg)")
        self.ax3.set_xlabel("Time (s)")
        
        # Twin axis for PWM
        self.ax3_pwm = self.ax3.twinx()
        self.ln_servo_pwm, = self.ax3_pwm.plot([], [], 'm:', label='PWM', linewidth=1, alpha=0.7)
        self.ax3_pwm.set_ylabel("PWM (us)")
        
        lines = [self.ln_servo_angle, self.ln_servo_pwm]
        labels = [l.get_label() for l in lines]
        self.ax3.legend(lines, labels, loc='upper left')
        self.ax3.grid(True, alpha=0.3)
        
        # Important: set tight_layout or process to respect resizing
        # But we used subplots_adjust, so tight_layout might conflict. 
        # Matplotlib within Tkinter usually resizes well with pack/grid.
        
        canvas = FigureCanvasTkAgg(self.fig, master=parent)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)
        
        # Animation - blit=False is required for dynamic axis scaling
        self.anim = FuncAnimation(self.fig, self.update_plots, interval=50, blit=False, cache_frame_data=False)

    def refresh_ports(self, event=None):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.combo_port['values'] = ports
        if ports and not self.combo_port.get():
            self.combo_port.current(0)
            
    def toggle_connection(self):
        if not self.running:
            try:
                port = self.combo_port.get()
                baud = self.combo_baud.get()
                if not port:
                    return
                self.serial_port = serial.Serial(port, int(baud), timeout=0.1)
                self.running = True
                self.btn_connect.config(text="Disconnect")
                
                # Start logging
                fname = f"log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
                self.csv_file = open(fname, "w", newline='')
                self.csv_writer = csv.writer(self.csv_file)
                self.csv_writer.writerow(["Time_PC", "Time_Dev", "Target", "Yaw", "Rate", "Servo_Angle", "Servo_PWM"])
                self.log_to_console(f"Connected to {port}. Logging to {fname}")
                
                # Start thread
                self.reading_thread = threading.Thread(target=self.serial_loop, daemon=True)
                self.reading_thread.start()
                
            except Exception as e:
                messagebox.showerror("Error", str(e))
        else:
            self.running = False
            if self.serial_port:
                try:
                    self.serial_port.close()
                except:
                    pass
            if self.csv_file:
                try:
                    self.csv_file.close()
                except:
                    pass
            self.btn_connect.config(text="Connect")
            self.log_to_console("Disconnected")

    def send_command(self, event=None):
        if not self.running or not self.serial_port:
            return
        cmd = self.entry_cmd.get()
        if cmd:
            self.send_cmd_str(cmd)
            self.entry_cmd.delete(0, 'end')

    def send_cmd_str(self, cmd_str):
        if self.running and self.serial_port:
            try:
                full_cmd = cmd_str + "\n"
                self.serial_port.write(full_cmd.encode('utf-8'))
                self.log_to_console(f"-> {cmd_str}")
            except Exception as e:
                self.log_to_console(f"Send Error: {e}")

    def log_to_console(self, text):
        self.txt_console.config(state='normal')
        self.txt_console.insert('end', text + "\n")
        self.txt_console.see('end')
        self.txt_console.config(state='disabled')

    def update_window_size(self):
        try:
            val = int(self.entry_points.get())
            if 100 <= val <= 10000:
                self.max_points = val
                with self.data_lock:
                    self.times = deque(self.times, maxlen=val)
                    self.yaw_target = deque(self.yaw_target, maxlen=val)
                    self.yaw_meas = deque(self.yaw_meas, maxlen=val)
                    self.gyro_rate = deque(self.gyro_rate, maxlen=val)
                    self.servo_angle = deque(self.servo_angle, maxlen=val)
                    self.servo_pwm = deque(self.servo_pwm, maxlen=val)
            else:
                messagebox.showwarning("Range", "Please enter 100-10000")
        except ValueError:
            pass
            
    def clear_data(self):
        with self.data_lock:
            self.times.clear()
            self.yaw_target.clear()
            self.yaw_meas.clear()
            self.gyro_rate.clear()
            self.servo_angle.clear()
            self.servo_pwm.clear()

    def serial_loop(self):
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8', errors='replace').strip()
                    if not line:
                        continue
                    
                    if line.startswith("TRACK,"):
                        self.parse_track_line(line)
                    else:
                        # Log other output to console window
                        self.root.after(0, lambda l=line: self.log_to_console(f"[DEV] {l}"))
                else:
                    time.sleep(0.001)
                    
            except Exception as e:
                print(f"Serial Error: {e}")
                self.running = False
                self.root.after(0, lambda: self.btn_connect.config(text="Connect"))
                break

    def parse_track_line(self, line):
        try:
            # Format: TRACK,13440,0.00,-0.07,-0.01,90,307
            parts = line.split(',')
            if len(parts) < 7:
                return
                
            t_dev = float(parts[1]) / 1000.0
            target = float(parts[2])
            meas = float(parts[3])
            rate = float(parts[4])
            ang = float(parts[5])
            pwm = float(parts[6])
            
            # Write CSV
            if self.csv_writer:
                self.csv_writer.writerow([time.time(), parts[1], target, meas, rate, ang, pwm])
            
            # Update Buffers
            with self.data_lock:
                self.times.append(t_dev)
                self.yaw_target.append(target)
                self.yaw_meas.append(meas)
                self.gyro_rate.append(rate)
                self.servo_angle.append(ang)
                self.servo_pwm.append(pwm)
                
        except ValueError:
            pass

    def update_plots(self, frame):
        with self.data_lock:
            if not self.times:
                return self.ln_target, self.ln_meas, self.ln_rate, self.ln_servo_angle, self.ln_servo_pwm
            
            t = list(self.times)
            y_tgt = list(self.yaw_target)
            y_meas = list(self.yaw_meas)
            y_rate = list(self.gyro_rate)
            y_ang = list(self.servo_angle)
            y_pwm = list(self.servo_pwm)
            
            self.ln_target.set_data(t, y_tgt)
            self.ln_meas.set_data(t, y_meas)
            self.ln_rate.set_data(t, y_rate)
            self.ln_servo_angle.set_data(t, y_ang)
            self.ln_servo_pwm.set_data(t, y_pwm)
            
            # X-Axis Scaling
            if len(t) > 0:
                t_min, t_max = min(t), max(t)
                # Keep a sliding window if data is too long, or just fit all
                # Here we fit all in buffer
                self.ax1.set_xlim(t_min, t_max + 0.5)
            
            # Manual Y-Axis Scaling for stability
            def set_y_limits(ax, data_list, margin=0.1):
                if not data_list: return
                d_min, d_max = min(data_list), max(data_list)
                span = d_max - d_min
                if span == 0: span = 1.0
                ax.set_ylim(d_min - span*margin, d_max + span*margin)

            # Ax1: Yaw (Target & Measured)
            all_yaw = y_tgt + y_meas
            if all_yaw:
                 set_y_limits(self.ax1, all_yaw)

            # Ax2: Rate
            if y_rate:
                set_y_limits(self.ax2, y_rate)

            # Ax3: Servo Angle
            if y_ang:
                set_y_limits(self.ax3, y_ang)
            
            # Ax3 PWM (Twin)
            if y_pwm:
                set_y_limits(self.ax3_pwm, y_pwm)
                
        return self.ln_target, self.ln_meas, self.ln_rate, self.ln_servo_angle, self.ln_servo_pwm

    def save_screenshot(self):
        try:
            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            fname = f"plot_{timestamp}.png"
            self.fig.savefig(fname, dpi=150)
            self.log_to_console(f"Screenshot saved: {fname}")
            messagebox.showinfo("Success", f"Screenshot saved to:\n{fname}")
        except Exception as e:
            self.log_to_console(f"Screenshot Error: {e}")
            messagebox.showerror("Error", str(e))

    def save_console_log(self):
        try:
            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            fname = f"console_{timestamp}.txt"
            content = self.txt_console.get("1.0", "end-1c")
            with open(fname, "w", encoding="utf-8") as f:
                f.write(content)
            self.log_to_console(f"Log saved: {fname}")
            messagebox.showinfo("Success", f"Console log saved to:\n{fname}")
        except Exception as e:
            self.log_to_console(f"Log Save Error: {e}")
            messagebox.showerror("Error", str(e))

if __name__ == "__main__":
    root = tk.Tk()
    app = WingSailMonitor(root)
    root.mainloop()
