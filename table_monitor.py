#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, Int32
import threading
import os

data = {
    'encoder_left': None,
    'encoder_right': None,
    'omega_est': None,
    'rpm_left': None,
    'rpm_right': None,
    'theta': None,
    'v_est': None,
    'vel_left': None,
    'vel_right': None,
    'cpu_temp': None  # New: Raspberry Pi CPU temperature
}

lock = threading.Lock()

def make_callback(topic_name):
    def callback(msg):
        with lock:
            data[topic_name] = msg.data
    return callback

def read_cpu_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            raw = f.read().strip()
            return float(raw) / 1000.0
    except:
        return None

def print_table():
    rate = rospy.Rate(5)  # 5 Hz
    while not rospy.is_shutdown():
        with lock:
            # Update CPU temperature
            data['cpu_temp'] = read_cpu_temp()

            os.system('clear')  # Use 'cls' for Windows
            print(f"{'Topic':<15} | {'Value':>10}")
            print("-" * 28)
            for key in data:
                val = data[key]
                val_str = f"{val:.2f}" if val is not None else "..."
                print(f"{key:<15} | {val_str:>10}")
        rate.sleep()

def main():
    rospy.init_node('live_table_monitor', anonymous=True)

    ros_topics = [k for k in data if k != 'cpu_temp']  # Exclude cpu_temp

    for topic in ros_topics:
        rospy.Subscriber(f"/{topic}", Float32, make_callback(topic))

    thread = threading.Thread(target=print_table)
    thread.start()

    rospy.spin()

if __name__ == '__main__':
    main()
