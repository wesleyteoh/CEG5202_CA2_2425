import serial
import csv
import time
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.dates as mdates
import re
from datetime import datetime, timedelta

SERIAL_PORT = '/dev/cu.usbmodem103'  
# SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
CSV_FILENAME = 'STM32_data.csv'
ALERT_LOG_FILENAME = 'alert_log.txt'

sensors = ['temperature', 'humidity', 'pressure', 'magnetometer', 'accelerometer', 'gyroscope']

data_lines = []
alert_events = []
alert_markers = []

# accepted and lost counts
sensor_stats = {sensor: {'accepted': 0, 'lost': 0} for sensor in sensors}

def record_data():
    global data_lines, sensor_stats
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return

    with open(CSV_FILENAME, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(['Timestamp', 'Sensor', 'Value1', 'Value2', 'Value3'])
        print(f"Logging data to {CSV_FILENAME}. Press Ctrl+C to exit.")
        try:
            while True:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    # Check for alert messages
                    if line.startswith("** Alert:"):
                        data_lines.append(line)
                        continue

                    lower_line = line.lower()
                    # Check for FIFO full messages
                    if "fifo full" in lower_line:
                        for sensor in sensors:
                            if sensor in lower_line:
                                sensor_stats[sensor]["lost"] += 1
                                print(f"Lost event for {sensor}")
                                break
                        continue

                    if ',' in line:
                        fields = line.split(',')
                        if len(fields) >= 3:
                            sensor_name = fields[0].strip()
                            device_timestamp = fields[1].strip()  # STM32 tick timestamp
                            rest = [x.strip() for x in fields[2:]]
                            new_fields = [device_timestamp, sensor_name] + rest
                            csvwriter.writerow(new_fields)
                            sensor_lower = sensor_name.lower()
                            if sensor_lower in sensor_stats:
                                sensor_stats[sensor_lower]["accepted"] += 1
                            data_lines.append(",".join(new_fields))
                        else:
                            data_lines.append(line)
                    else:
                        data_lines.append(line)
        except KeyboardInterrupt:
            print("Logging stopped.")
        finally:
            ser.close()

start_time = datetime.now()
transmission_mode = "FULL BUFFER MODE"  # Default mode

def parse_line(line):
    global transmission_mode
    lower_line = line.lower()

    # Checks for tx mode updates
    if "transmission mode toggled to:" in lower_line:
        if "random mode" in lower_line:
            transmission_mode = "RANDOM MODE"
        elif "full buffer mode" in lower_line:
            transmission_mode = "FULL BUFFER MODE"
        elif "predictive mode" in lower_line:
            transmission_mode = "PREDICTIVE MODE"
        print(f"[INFO] {line.strip()}")
        return {'type': 'mode_update', 'mode': transmission_mode}

    if line.strip().startswith("** Alert:"):
        timestamp_match = re.search(r"\*\* Alert:\s*(\d+)", line)
        if timestamp_match:
            board_timestamp = float(timestamp_match.group(1))
            alert_timestamp = start_time + timedelta(milliseconds=board_timestamp)
        else:
            alert_timestamp = datetime.now()  # fallback if no timestamp found

        with open(ALERT_LOG_FILENAME, "a") as alert_log:
            alert_log.write(f"{alert_timestamp.isoformat()} - {line}\n")

        sensor_delay_match = re.search(r"Sensor delay:\s*(\d+)\s*ms", line)
        response_delay_match = re.search(r"Response delay:\s*(\d+)\s*ms", line)
        sensor_delay = int(sensor_delay_match.group(1)) if sensor_delay_match else None
        response_delay = int(response_delay_match.group(1)) if response_delay_match else None

        alert_type = None
        sensor_value = None
        violated_axes = None
        if "high temperature" in lower_line:
            alert_type = "temperature"
            match = re.search(r"high temperature alert:\s*([0-9]*\.?[0-9]+)", lower_line)
            if match:
                sensor_value = float(match.group(1))
        elif "low humidity" in lower_line:
            alert_type = "humidity"
            match = re.search(r"low humidity alert:\s*([0-9]*\.?[0-9]+)", lower_line)
            if match:
                sensor_value = float(match.group(1))
        elif "high humidity" in lower_line:
            alert_type = "humidity"
            match = re.search(r"high humidity alert:\s*([0-9]*\.?[0-9]+)", lower_line)
            if match:
                sensor_value = float(match.group(1))
        elif "vibration" in lower_line:
            alert_type = "vibration"
            accel_match = re.search(r"accel:\s*([-\d\.]+)\s*,\s*([-\d\.]+)\s*,\s*([-\d\.]+)", lower_line)
            if accel_match:
                sensor_value = (float(accel_match.group(1)),
                                float(accel_match.group(2)),
                                float(accel_match.group(3)))
            else:
                sensor_value = None
            violated_axes_match = re.search(r"violated:\s*([a-z ]+)", lower_line)
            if violated_axes_match:
                violated_axes = violated_axes_match.group(1).strip()
            else:
                violated_axes = None

        return {
            'type': 'alert',
            'timestamp': alert_timestamp,
            'alert_type': alert_type,
            'sensor_delay': sensor_delay,
            'response_delay': response_delay,
            'sensor_value': sensor_value,
            'violated_axes': violated_axes,
            'message': line.strip()
        }
    else:
        # Expected format: sensor,timestamp,value1,value2,value3
        parts = line.split(',')
        try:
            raw_timestamp = parts[0].strip()
            timestamp = start_time + timedelta(milliseconds=float(raw_timestamp))
            sensor = parts[1].strip().lower()
            values = []
            for x in parts[2:]:
                x = x.strip()
                cleaned = re.sub(r'[^0-9\.\-]', '', x)
                try:
                    values.append(float(cleaned))
                except ValueError:
                    values.append(None)
            return {'type': 'data', 'timestamp': timestamp, 'sensor': sensor, 'values': values}
        except (ValueError, IndexError):
            return {'type': 'event', 'message': line.strip()}

# Initialize sensor data and time dictionaries
# For 3-axis sensors, store data as dicts with keys "x", "y", and "z"
sensor_data = {}
sensor_time = {}
for sensor in sensors:
    if sensor in ["accelerometer", "gyroscope"]:
        sensor_data[sensor] = {"x": [], "y": [], "z": []}
        sensor_time[sensor] = []
    else:
        sensor_data[sensor] = []
        sensor_time[sensor] = []

# ===== Matplotlib setup =====
fig, axs = plt.subplots(len(sensors), 1, sharex=True, figsize=(15, 8))
fig.suptitle("Real-time Sensor Data")
lines = {}
loss_text = {}

TIME_WINDOW = 100  # in seconds

for i, sensor in enumerate(sensors):
    ax = axs[i]
    ax.set_ylabel(sensor.capitalize())
    ax.grid(True)
    if sensor in ["accelerometer", "gyroscope"]:
        line_x, = ax.plot([], [], marker='o', linestyle='-', markersize=4, label="X")
        line_y, = ax.plot([], [], marker='o', linestyle='-', markersize=4, label="Y")
        line_z, = ax.plot([], [], marker='o', linestyle='-', markersize=4, label="Z")
        lines[sensor] = (line_x, line_y, line_z)
        ax.legend(loc='upper left')
    else:
        line, = ax.plot([], [], marker='o', linestyle='-', markersize=4)
        lines[sensor] = line
    loss_text[sensor] = ax.text(0.95, 0.9, "", transform=ax.transAxes,
                                ha='right', va='top', fontsize=9, color='red')
    ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
    plt.setp(ax.xaxis.get_majorticklabels(), rotation=45, ha='right')

axs[-1].set_xlabel("Time")

def update_plot(frame):
    global data_lines, sensor_data, sensor_time, sensor_stats, alert_events, alert_markers

    for marker in alert_markers:
        marker.remove()
    alert_markers = []

    current_data = []
    while data_lines:
        current_data.append(data_lines.pop(0))
    
    for line in current_data:
        parsed = parse_line(line)
        if parsed['type'] == 'data':
            if parsed['sensor'] == "accelerometer":
                if parsed['values'] and len(parsed['values']) >= 3:
                    sensor_data["accelerometer"]["x"].append(parsed["values"][0])
                    sensor_data["accelerometer"]["y"].append(parsed["values"][1])
                    sensor_data["accelerometer"]["z"].append(parsed["values"][2])
                    sensor_time["accelerometer"].append(parsed["timestamp"])
            elif parsed['sensor'] == "gyroscope":
                if parsed['values'] and len(parsed['values']) >= 3:
                    sensor_data["gyroscope"]["x"].append(parsed["values"][0])
                    sensor_data["gyroscope"]["y"].append(parsed["values"][1])
                    sensor_data["gyroscope"]["z"].append(parsed["values"][2])
                    sensor_time["gyroscope"].append(parsed["timestamp"])
            elif parsed['sensor'] in sensors:
                if parsed['values'] and parsed['values'][0] is not None:
                    sensor_data[parsed['sensor']].append(parsed["values"][0])
                    sensor_time[parsed['sensor']].append(parsed["timestamp"])
        elif parsed['type'] == 'alert':
            alert_events.append(parsed)
            ts_str = parsed['timestamp'].strftime('%H:%M:%S')
            print(f"[ALERT] {ts_str}: {parsed['message']} "
                  f"(Sensor delay: {parsed['sensor_delay']} ms, Response delay: {parsed['response_delay']} ms)")
        elif parsed['type'] == 'event':
            print(f"[EVENT] {parsed['message']}")

    # Remove old alert events
    cutoff = datetime.now() - timedelta(seconds=TIME_WINDOW)
    alert_events[:] = [event for event in alert_events if event['timestamp'] >= cutoff]

    # Update sensor plots
    for sensor in sensors:
        ax = axs[sensors.index(sensor)]
        if sensor in ["accelerometer", "gyroscope"]:
            line_x, line_y, line_z = lines[sensor]
            line_x.set_data(sensor_time[sensor], sensor_data[sensor]["x"])
            line_y.set_data(sensor_time[sensor], sensor_data[sensor]["y"])
            line_z.set_data(sensor_time[sensor], sensor_data[sensor]["z"])
            all_data = sensor_data[sensor]["x"] + sensor_data[sensor]["y"] + sensor_data[sensor]["z"]
            if all_data:
                y_pad = (max(all_data) - min(all_data)) * 0.1 or 1
                ax.set_ylim(min(all_data) - y_pad, max(all_data) + y_pad)
            if sensor_time[sensor]:
                latest_ts = sensor_time[sensor][-1]
                x_min = latest_ts - timedelta(seconds=TIME_WINDOW)
                ax.set_xlim(x_min, latest_ts)
        else:
            lines[sensor].set_data(sensor_time[sensor], sensor_data[sensor])
            if sensor_data[sensor]:
                y_data = sensor_data[sensor]
                y_pad = (max(y_data) - min(y_data)) * 0.1 or 1
                ax.set_ylim(min(y_data) - y_pad, max(y_data) + y_pad)
            if sensor_time[sensor]:
                latest_ts = sensor_time[sensor][-1]
                x_min = latest_ts - timedelta(seconds=TIME_WINDOW)
                ax.set_xlim(x_min, latest_ts)
        
        stats = sensor_stats[sensor]
        total = stats['accepted'] + stats['lost'] or 1
        loss_text[sensor].set_text(f"Loss: {stats['lost'] / total * 100:.1f}%")
        
    # Mark vibration alerts on the accelerometer plot
    for event in alert_events:
        if event.get('alert_type') == 'vibration':
            ax_acc = axs[sensors.index('accelerometer')]
            if event.get('violated_axes'):
                axes_violated = event['violated_axes'].split()
                for axis in axes_violated:
                    if axis == 'x':
                        y_val = sensor_data["accelerometer"]["x"][-1] if sensor_data["accelerometer"]["x"] else 0
                        marker = ax_acc.plot(event['timestamp'], y_val, 'ro', markersize=8)[0]
                        alert_markers.append(marker)
                    if axis == 'y':
                        y_val = sensor_data["accelerometer"]["y"][-1] if sensor_data["accelerometer"]["y"] else 0
                        marker = ax_acc.plot(event['timestamp'], y_val, 'ro', markersize=8)[0]
                        alert_markers.append(marker)
                    if axis == 'z':
                        y_val = sensor_data["accelerometer"]["z"][-1] if sensor_data["accelerometer"]["z"] else 0
                        marker = ax_acc.plot(event['timestamp'], y_val, 'ro', markersize=8)[0]
                        alert_markers.append(marker)
            else:
                y_val = sensor_data["accelerometer"]["z"][-1] if sensor_data["accelerometer"]["z"] else 0
                marker = ax_acc.plot(event['timestamp'], y_val, 'ro', markersize=8)[0]
                alert_markers.append(marker)
        
    # Return all plotted objects
    all_lines = []
    for sensor_val in lines.values():
        if isinstance(sensor_val, tuple):
            all_lines.extend(sensor_val)
        else:
            all_lines.append(sensor_val)
    return all_lines + list(loss_text.values()) + alert_markers

ani = animation.FuncAnimation(fig, update_plot, interval=1000, blit=False)

record_thread = threading.Thread(target=record_data, daemon=True)
record_thread.start()

plt.tight_layout()
plt.show()