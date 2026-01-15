#!/usr/bin/env python3
"""
Real-time Tracking Plot for Wing Sail Control
Reads serial data and plots target vs measured yaw angle

Usage:
    python plot_tracking.py [COM_PORT] [BAUD_RATE]
    
Example:
    python plot_tracking.py COM10 115200
"""

import serial
import matplotlib.pyplot as plt
import sys
import time
import threading
from collections import deque

# Configuration
DEFAULT_PORT = "COM10"
DEFAULT_BAUD = 115200
MAX_POINTS = 3000000  # Increased: 30000 points = 600s (10 min) at 50Hz

# Data storage (thread-safe with deque)
times = deque(maxlen=MAX_POINTS)
targets = deque(maxlen=MAX_POINTS)
measured = deque(maxlen=MAX_POINTS)
data_lock = threading.Lock()
running = True
start_time = None

def parse_line(line):
    """Parse TRACK,time_ms,target,measured format"""
    try:
        parts = line.strip().split(',')
        if len(parts) >= 4 and parts[0] == "TRACK":
            time_ms = float(parts[1])
            target = float(parts[2])
            meas = float(parts[3])
            return time_ms / 1000.0, target, meas
    except:
        pass
    return None

def serial_reader(port, baud):
    """Background thread to read serial data"""
    global running, start_time
    
    try:
        ser = serial.Serial(port, baud, timeout=0.5)
        print(f"Connected to {port} at {baud} baud")
        print("Waiting for data... (Press ESP32 Reset button)")
        print("-" * 50)
        
        while running:
            try:
                if ser.in_waiting:
                    line = ser.readline().decode('utf-8', errors='ignore')
                    if line.strip():
                        # Only print non-TRACK messages to reduce console spam
                        if not line.startswith("TRACK"):
                            print(f"RX: {line.strip()}")
                        
                        result = parse_line(line)
                        if result:
                            t, tgt, meas = result
                            with data_lock:
                                # Use time directly from ESP32 (already relative)
                                # Reset data if time goes backwards (device reset)
                                if len(times) > 0 and t < times[-1] - 1.0:
                                    print("Device reset detected, clearing data...")
                                    times.clear()
                                    targets.clear()
                                    measured.clear()
                                times.append(t)
                                targets.append(tgt)
                                measured.append(meas)
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f"Read error: {e}")
                break
                
        ser.close()
    except Exception as e:
        print(f"Serial error: {e}")

def main():
    global running
    
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD
    
    print("=" * 50)
    print("Wing Sail Tracking Plot")
    print("=" * 50)
    print(f"Port: {port}, Baud: {baud}")
    print("Close the plot window to stop and save data")
    print("=" * 50)
    
    # Start serial reader thread
    reader_thread = threading.Thread(target=serial_reader, args=(port, baud), daemon=True)
    reader_thread.start()
    
    # Wait a moment for connection
    time.sleep(1)
    
    # Setup plot with interactive mode
    plt.ion()
    fig, ax = plt.subplots(figsize=(12, 6))
    fig.canvas.manager.set_window_title('Wing Sail Tracking Control')
    
    target_line, = ax.plot([], [], 'b-', label='Target (°)', linewidth=2)
    measured_line, = ax.plot([], [], 'r-', label='Measured (°)', linewidth=1.5)
    
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Yaw Angle (°)')
    ax.set_title('Wing Sail Tracking Control')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(-35, 35)
    ax.set_xlim(0, 10)
    
    error_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, 
                         verticalalignment='top', fontsize=10, color='green')
    
    try:
        last_print_time = 0
        while plt.fignum_exists(fig.number):
            with data_lock:
                if len(times) > 0:
                    t_list = list(times)
                    tgt_list = list(targets)
                    meas_list = list(measured)
                else:
                    t_list, tgt_list, meas_list = [], [], []
            
            if len(t_list) > 0:
                target_line.set_data(t_list, tgt_list)
                measured_line.set_data(t_list, meas_list)
                
                # Update axis limits - auto-expand with time
                max_time = t_list[-1]
                min_time = t_list[0]
                ax.set_xlim(min_time, max_time + 5)  # Always show 5s ahead
                
                # Print status every 5 seconds
                if max_time - last_print_time >= 5:
                    print(f"[{max_time:.1f}s] Points: {len(t_list)}, Target: {tgt_list[-1]:.1f}°, Measured: {meas_list[-1]:.1f}°")
                    last_print_time = max_time
                
                # Update error text
                error = abs(tgt_list[-1] - meas_list[-1])
                error_text.set_text(f'Error: {error:.2f}° | Time: {max_time:.1f}s | Points: {len(t_list)}')
            
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        pass
    finally:
        running = False
        
        # Save data
        if len(times) > 0:
            filename = f"tracking_data_{int(time.time())}.csv"
            with open(filename, 'w') as f:
                f.write("time_s,target_deg,measured_deg,error_deg\n")
                with data_lock:
                    for t, tgt, meas in zip(times, targets, measured):
                        f.write(f"{t:.3f},{tgt:.2f},{meas:.2f},{tgt-meas:.2f}\n")
            print(f"\nData saved to {filename}")
        
        print("Done.")

if __name__ == "__main__":
    main()
