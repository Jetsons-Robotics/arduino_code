#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from flask import Flask, jsonify
from flask_cors import CORS
from threading import Lock, Thread
import subprocess
import os
import signal

app = Flask(__name__)
CORS(app)

data = {
    'v_est': None,
    'theta': None,
    'w_est': None,
    'cpu_temp': None,
    'camera_node': None
}
lock = Lock()

launch_process = None  # Handle for roslaunch process

def make_callback(key):
    def cb(msg):
        with lock:
            data[key] = msg.data
    return cb

def read_cpu_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            return float(f.read()) / 1000.0
    except:
        return None

@app.route('/data/<key>')
def get_data(key):
    with lock:
        if key == 'cpu_temp':
            value = read_cpu_temp()
        else:
            value = data.get(key)
        return jsonify({'value': value if value is not None else 0.0})

@app.route('/start_robot', methods=['GET'])
def start_robot():
    global launch_process
    if launch_process is None:
        try:
            launch_process = subprocess.Popen(
                ['roslaunch', 'cleaning_robot', 'cleaning_robot.launch'],
                preexec_fn=os.setsid
            )
            return jsonify({'status': 'started'})
        except Exception as e:
            return jsonify({'status': 'error', 'message': str(e)})
    else:
        return jsonify({'status': 'already running'})

@app.route('/stop_robot', methods=['GET'])
def stop_robot():
    global launch_process
    if launch_process is not None:
        try:
            os.killpg(os.getpgid(launch_process.pid), signal.SIGTERM)
            launch_process = None
            return jsonify({'status': 'stopped'})
        except Exception as e:
            return jsonify({'status': 'error', 'message': str(e)})
    else:
        return jsonify({'status': 'not running'})

def ros_listen():
    # ROS subscribers (ROS node already initialized in main thread)
    for key in data:
        if key != 'cpu_temp':
            rospy.Subscriber(f'/{key}', Float32, make_callback(key))
    rospy.spin()

def start_roscore():
    try:
        # Check if ROS Master is already running
        subprocess.check_output(['rosnode', 'list'], stderr=subprocess.DEVNULL)
        print("roscore is already running.")
        return None
    except subprocess.CalledProcessError:
        print("Starting roscore...")
        roscore_process = subprocess.Popen(
            ['roscore'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )
        import time
        time.sleep(2)  # Give time for roscore to initialize
        return roscore_process

# Start roscore if not already running
roscore_process = start_roscore()

# Optionally stop it on exit
import atexit
if roscore_process:
    atexit.register(lambda: os.killpg(os.getpgid(roscore_process.pid), signal.SIGTERM))


if __name__ == '__main__':
    roscore_process = start_roscore()  # <-- Start roscore here

    if roscore_process:
        atexit.register(lambda: os.killpg(os.getpgid(roscore_process.pid), signal.SIGTERM))

    # Initialize ROS node
    rospy.init_node('online_monitor', anonymous=True)

    # Start ROS subscriber thread
    ros_thread = Thread(target=ros_listen)
    ros_thread.daemon = True
    ros_thread.start()

    # Start Flask server (main thread)
    app.run(host='0.0.0.0', port=5000)
