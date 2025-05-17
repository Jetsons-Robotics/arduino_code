#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, Int32
import threading
import os

# Dictionary to store the latest values
data = {
    # 'encoder_left': None,
    # 'encoder_right': None,
    'omega_est': None,
    'rpm_left': None,
    'rpm_right': None,
    'theta': None,
    'v_est': None,
    # 'vel_left': None,
    # 'vel_right': None
}

# Lock for thread-safe updates
lock = threading.Lock()

def make_callback(topic_name):
    def callback(msg):
        with lock:
            data[topic_name] = msg.data
    return callback

def print_table():
    rate = rospy.Rate(5)  # 5 Hz update rate
    while not rospy.is_shutdown():
        with lock:
            os.system('clear')  # or 'cls' on Windows
            print(f"{'Topic':<15} | {'Value':>10}")
            print("-" * 28)
            for key in data:
                val = data[key]
                val_str = f"{val:.2f}" if val is not None else "..."
                print(f"{key:<15} | {val_str:>10}")
        rate.sleep()

def main():
    rospy.init_node('live_table_monitor', anonymous=True)

    # List of topics and assumed message types
    topics = list(data.keys())

    for topic in topics:
        # Assuming all are Float32; if any are Int32, you can handle that here
        rospy.Subscriber(f"/{topic}", Float32, make_callback(topic))

    # Start printing thread
    thread = threading.Thread(target=print_table)
    thread.start()

    rospy.spin()

if __name__ == '__main__':
    main()

