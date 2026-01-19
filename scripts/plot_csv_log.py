import csv
import matplotlib.pyplot as plt
import sys
import os
import tkinter as tk
from tkinter import filedialog

# Initialize Tkinter root for file dialog
root = tk.Tk()
root.withdraw()

def plot_csv(filename):
    if not filename:
        print("No file selected.")
        return

    print(f"Loading {filename}...")
    
    times_pc = []
    times_dev = []
    targets = []
    yaws = []
    rates = []
    servo_angles = []
    servo_pwms = []

    try:
        with open(filename, 'r', newline='') as csvfile:
            reader = csv.reader(csvfile)
            header = next(reader) # Skip header
            # expected: Time_PC, Time_Dev, Target, Yaw, Rate, Servo_Angle, Servo_PWM
            
            start_time = None

            for row in reader:
                if len(row) < 7:
                    continue
                
                try:
                    # Parse data using the monitor.py format
                    t_pc = float(row[0])
                    t_dev = float(row[1]) 
                    target = float(row[2])
                    yaw = float(row[3])
                    rate = float(row[4])
                    s_angle = float(row[5])
                    s_pwm = float(row[6])

                    if start_time is None:
                        start_time = t_pc
                    
                    times_pc.append(t_pc - start_time) # Relative time for plotting
                    times_dev.append(t_dev)
                    targets.append(target)
                    yaws.append(yaw)
                    rates.append(rate)
                    servo_angles.append(s_angle)
                    servo_pwms.append(s_pwm)
                except ValueError:
                    continue
                    
    except Exception as e:
        print(f"Error reading file: {e}")
        return

    # Create plots
    plt.style.use('dark_background')
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(12, 10))
    fig.canvas.manager.set_window_title(f'Log Analysis: {os.path.basename(filename)}')

    # Plot 1: Angles
    # Calculate tolerance bands
    targets_upper = [t + 3 for t in targets]
    targets_lower = [t - 3 for t in targets]
    
    ax1.plot(times_pc, targets, 'r--', label='Target Yaw', linewidth=1.5)
    ax1.plot(times_pc, targets_upper, 'r:', linewidth=1, alpha=0.5, label='±3° Range')
    ax1.plot(times_pc, targets_lower, 'r:', linewidth=1, alpha=0.5)
    ax1.fill_between(times_pc, targets_lower, targets_upper, color='red', alpha=0.1)
    
    ax1.plot(times_pc, yaws, 'g-', label='Measured Yaw', linewidth=1.5)
    ax1.set_ylabel('Angle (deg)')
    ax1.set_title('Target vs Measured Yaw')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)

    # Plot 2: Gyro Rate
    ax2.plot(times_pc, rates, 'y-', label='Gyro Rate', linewidth=1)
    ax2.set_ylabel('Rate (deg/s)')
    ax2.set_title('Gyro Rate')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)

    # Plot 3: Servo Output
    ax3.plot(times_pc, servo_angles, 'c-', label='Servo Angle', linewidth=1.5)
    ax3.set_ylabel('Servo Angle (deg)', color='cyan')
    ax3.tick_params(axis='y', labelcolor='cyan')
    ax3.set_xlabel('Time (s) [Relative]')
    ax3.grid(True, alpha=0.3)
    
    # Twin axis for PWM
    ax3_pwm = ax3.twinx()
    ax3_pwm.plot(times_pc, servo_pwms, 'm:', label='Servo PWM', linewidth=1, alpha=0.7)
    ax3_pwm.set_ylabel('PWM (us)', color='magenta')
    ax3_pwm.tick_params(axis='y', labelcolor='magenta')
    
    # Combined legend for ax3
    lines, labels = ax3.get_legend_handles_labels()
    lines2, labels2 = ax3_pwm.get_legend_handles_labels()
    ax3.legend(lines + lines2, labels + labels2, loc='upper left')
    ax3.set_title('Servo Output')

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Check if file provided in argument
    if len(sys.argv) > 1:
        f_path = sys.argv[1]
    else:
        # Open file dialog
        print("Please select a CSV log file...")
        f_path = filedialog.askopenfilename(
            filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")]
        )
    
    plot_csv(f_path)
