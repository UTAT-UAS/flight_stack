#!/usr/bin/env python3
"""
Plot flight data from log file and ULG file
"""
import re
import matplotlib.pyplot as plt
import numpy as np
from pyulog import ULog

# Configuration: Choose run (30A or 50A)
run_type = "30A"  # or "30A"

if run_type == "30A":
    log_file = '/home/uas/workspace/logs/0517 Cyclone Logs/script/23_48_21.log'
    ulg_file = '/home/uas/workspace/logs/0517 Cyclone Logs/23_47_51.ulg'
else:  # 50A https://logs.px4.io/plot_app?log=84ac0e04-0884-4bf2-a237-903f2c864550
    log_file = '/home/uas/workspace/logs/0517 Cyclone Logs/script/00_31_11.log'
    ulg_file = '/home/uas/workspace/logs/0517 Cyclone Logs/00_30_39.ulg'

# Parse the log file
pathtime = []
current_draw = []
cruise_target_velocity = []
minjerk_velocity = []
ah_error = []

with open(log_file, 'r') as f:
    for line in f:
        try:
            # Extract Pathtime
            pt_match = re.search(r'Pathtime: ([\d.]+)', line)

            if pt_match:
                pathtime.append(float(pt_match.group(1)))
            
            # Extract Current Draw
            cd_match = re.search(r'Current Draw: ([\d.]+)', line)
            if cd_match:
                current_draw.append(float(cd_match.group(1)))
            
            # Extract Cruise Speed target
            cs_match = re.search(r'Cruise Speed target: ([\d.]+)', line)
            if cs_match:
                cruise_target_velocity.append(float(cs_match.group(1)))
            
            # Extract MinJerk Speed target
            mj_match = re.search(r'MinJerk Speed target: ([\d.]+)', line)
            if mj_match:
                minjerk_velocity.append(float(mj_match.group(1)))
            
            # Extract Ah Error
            ah_match = re.search(r'Ah Error: ([-\d.]+)', line)
            if ah_match:
                ah_error.append(float(ah_match.group(1)))
        except Exception as e:
            print(f"Error parsing line: {e}")
            continue

# Convert to numpy arrays
pathtime = np.array(pathtime)
current_draw = np.array(current_draw)
cruise_target_velocity = np.array(cruise_target_velocity)
minjerk_velocity = np.array(minjerk_velocity)
ah_error = np.array(ah_error)

# Parse the ULG file
print(f"Loading ULG file: {ulg_file}")
try:
    ulog = ULog(ulg_file)
    
    # Find the correct dataset name dynamically
    traj_dataset_name = None
    for d in ulog.data_list:
        if d.name == 'trajectory_setpoint' or d.name.startswith('trajectory_setpoint_'):
            traj_dataset_name = d.name
            break
            
    if traj_dataset_name is not None:
        traj_data = ulog.get_dataset(traj_dataset_name)
        
        # PX4 stores array fields dynamically (velocity[0] = x, velocity[1] = y)
        vx_setpoint = np.array(traj_data.data['velocity[0]'])
        vy_setpoint = np.array(traj_data.data['velocity[1]'])
        ulg_time = np.array(traj_data.data['timestamp']) / 1e6  # Convert from microseconds to seconds
        
        # Calculate magnitude
        v_mag = np.sqrt(vx_setpoint**2 + vy_setpoint**2)
        print(f"ULG data loaded: {len(vx_setpoint)} velocity setpoint samples from '{traj_dataset_name}'")
    else:
        print("Warning: trajectory_setpoint not found in ULG file")
        vx_setpoint = None
        vy_setpoint = None
        v_mag = None
except Exception as e:
    print(f"Error loading ULG file: {e}")
    vx_setpoint = None
    vy_setpoint = None
    v_mag = None

# Create time axis (using index)
time_axis = np.arange(len(pathtime))

# Determine which control mode is active (min between minjerk and cruise)
active_mode = np.minimum(minjerk_velocity, cruise_target_velocity)
is_minjerk_active = minjerk_velocity <= cruise_target_velocity

# Create subplots (7 plots if ULG data available, 5 otherwise)
num_plots = 7 if vx_setpoint is not None else 5
fig, axes = plt.subplots(num_plots, 1, figsize=(14, 14))

# Plot 1: Pathtime
axes[0].plot(time_axis, pathtime, 'b-', linewidth=1)
axes[0].set_ylabel('Pathtime (s)')
axes[0].set_title('Flight Path Timeline')
axes[0].grid(True, alpha=0.3)

# Plot 2: Speed targets with control mode coloring
ax_speed = axes[1]
# Plot both speeds as lines
ax_speed.plot(time_axis, minjerk_velocity, 'g--', linewidth=1.5, label='MinJerk Target', alpha=0.7)
ax_speed.plot(time_axis, cruise_target_velocity, 'r--', linewidth=1.5, label='Cruise Target', alpha=0.7)

# Color regions based on active control mode
for i in range(len(time_axis) - 1):
    if is_minjerk_active[i]:
        ax_speed.axvspan(i, i + 1, alpha=0.1, color='green')
    else:
        ax_speed.axvspan(i, i + 1, alpha=0.1, color='red')

# Plot the active mode as a thick line
ax_speed.plot(time_axis, active_mode, 'k-', linewidth=2, label='Active Control', alpha=0.9)

ax_speed.set_ylabel('Velocity (m/s)')
ax_speed.set_title('Speed Targets with Active Control Mode (Green=MinJerk, Red=Cruise)')
ax_speed.legend(loc='best')
ax_speed.grid(True, alpha=0.3)

# Plot 3: Current Draw
axes[2].plot(time_axis, current_draw, 'orange', linewidth=1)
axes[2].set_ylabel('Current Draw (A)')
axes[2].set_title('Current Draw')
axes[2].grid(True, alpha=0.3)

# Plot 4: Ah Error
axes[3].plot(time_axis, ah_error, 'purple', linewidth=1)
axes[3].set_ylabel('Ah Error (Ah)')
axes[3].set_xlabel('Sample Index')
axes[3].set_title('Amp-Hour Error')
axes[3].grid(True, alpha=0.3)

# Plot 5: Control mode transitions (bonus)
ax_mode = axes[4]
ax_mode.fill_between(time_axis, 0, 1, where=is_minjerk_active, alpha=0.3, color='green', label='MinJerk Active')
ax_mode.fill_between(time_axis, 0, 1, where=~is_minjerk_active, alpha=0.3, color='red', label='Cruise Active')
ax_mode.set_ylabel('Control Mode')
ax_mode.set_ylim([-0.1, 1.1])
ax_mode.set_yticks([])
ax_mode.set_xlabel('Sample Index')
ax_mode.set_title('Active Control Mode Timeline')
ax_mode.legend(loc='upper right')
ax_mode.grid(True, alpha=0.3, axis='x')

# Plot 6 & 7: Velocity setpoint components (if available)
if vx_setpoint is not None:
    # Create time axis for ULG data (in sample index, matching Python log scale)
    ulg_time_axis = np.linspace(0, len(pathtime), len(vx_setpoint))
    
    # Plot 6: Velocity X component
    ax_vx = axes[5]
    ax_vx.plot(ulg_time_axis, vx_setpoint, 'b-', linewidth=1, label='VX Setpoint')
    ax_vx.set_ylabel('VX (m/s)')
    ax_vx.set_title('Velocity X Setpoint from ULG')
    ax_vx.legend(loc='best')
    ax_vx.grid(True, alpha=0.3)
    
    # Plot 7: Velocity Y component
    ax_vy = axes[6]
    ax_vy.plot(ulg_time_axis, vy_setpoint, 'c-', linewidth=1, label='VY Setpoint')
    ax_vy.set_ylabel('VY (m/s)')
    ax_vy.set_xlabel('Sample Index')
    ax_vy.set_title('Velocity Y Setpoint from ULG')
    ax_vy.legend(loc='best')
    ax_vy.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('/home/uas/workspace/flight_log_plots.png', dpi=150, bbox_inches='tight')
print(f"Plotted {len(pathtime)} data points")
print(f"Plot saved to /home/uas/workspace/flight_log_plots.png")

# Print statistics
print(f"\nStatistics:")
print(f"Pathtime: min={pathtime.min():.3f}s, max={pathtime.max():.3f}s, mean={pathtime.mean():.3f}s")
print(f"Current Draw: min={current_draw.min():.2f}A, max={current_draw.max():.2f}A, mean={current_draw.mean():.2f}A")
print(f"Cruise Velocity: min={cruise_target_velocity.min():.2f}m/s, max={cruise_target_velocity.max():.2f}m/s, mean={cruise_target_velocity.mean():.2f}m/s")
print(f"MinJerk Velocity: min={minjerk_velocity.min():.3f}m/s, max={minjerk_velocity.max():.3f}m/s, mean={minjerk_velocity.mean():.3f}m/s")
print(f"Ah Error: min={ah_error.min():.3f}Ah, max={ah_error.max():.3f}Ah, mean={ah_error.mean():.3f}Ah")

if vx_setpoint is not None:
    print(f"\nULG Setpoint Statistics:")
    print(f"VX Setpoint: min={vx_setpoint.min():.3f}m/s, max={vx_setpoint.max():.3f}m/s, mean={vx_setpoint.mean():.3f}m/s")
    print(f"VY Setpoint: min={vy_setpoint.min():.3f}m/s, max={vy_setpoint.max():.3f}m/s, mean={vy_setpoint.mean():.3f}m/s")
    print(f"Velocity Magnitude: min={v_mag.min():.3f}m/s, max={v_mag.max():.3f}m/s, mean={v_mag.mean():.3f}m/s")

plt.show()