#!/usr/bin/env python
import rospy
import os
from cv_bridge import CvBridge
import cv2
import json
import tkinter as tk
from tkinter import ttk
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image as ImageMsg
from PIL import Image, ImageTk
from typing import *
import re


NUM_OF_THRUSTERS = 5
NUM_OF_SERVOS = 2
THRUSTER_TOPIC = '/AUVInternalSystem/DevPC/arbitrarlySetThrusters'
SERVOS_TOPIC = '/AUVInternalSystem/DevPC/arbitrarlySetServos'
GLOBAL_POSITION_TOPIC = '/AUVInternalSystem/DevPC/arbitrarlySetGlobalPosition'
THRUST_ALLOCATION_TOPIC = '/AUVInternalSystem/DevPC/arbitrarlySetThrustForce'
AUV_SYSTEM_DIR = re.search(".*AUVSystem", os.getcwd())[0]
AUV_CONFIG_DIR = os.path.join(AUV_SYSTEM_DIR, 'auvConfig', 'auvConfig.json')


class MainGuiTabs:
    def __init__(self, root):
        self.tab_root = ttk.Notebook(root)
        self.tab_root.pack(expand=True, fill='both')

    def add_tabs(self, tabs):
        for name, gui_element in tabs.items():
            self.tab_root.add(gui_element, text=name)


class CameraFrame:
    def __init__(self, root) -> None:
        self.root = root
        self.stereo_camera = False
        self.br = CvBridge()

        self.my_frame: tk.Frame = tk.Frame(self.root)
        self.my_frame.pack(fill='both', expand=True)

        self.left_camera = tk.Label(self.my_frame, text="LEFT", bg='green')
        self.left_camera.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.right_camera = tk.Label(self.my_frame, text="RIGHT", bg='red')
        self.right_camera.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.img_msg = None
        self.new_image = False
        self.img_size = None

    def update(self):
        if self.new_image:
            if self.img_size is None:
                img_width = self.left_camera.winfo_width()
                img_hight = self.left_camera.winfo_height()
                self.img_size = (img_width, img_hight)
            cv_img = self.br.imgmsg_to_cv2(self.img_msg)
            b, g, r = cv2.split(cv_img)
            img = cv2.resize(cv2.merge((r, g, b)), self.img_size)
            pil_img = Image.fromarray(img)
            tk_img = ImageTk.PhotoImage(image=pil_img)

            self.left_camera.configure(image=tk_img)
            self.left_camera.image = tk_img

            self.right_camera.configure(image=tk_img)
            self.right_camera.image = tk_img

            self.new_image = False

    def update_img_msg(self, img_msg):
        self.img_msg = img_msg
        self.new_image = True


class ControlsFrame:
    def __init__(self, root) -> None:
        self.root = root
        temp = tk.Frame(self.root)
        temp.pack(side=tk.BOTTOM)

        self.individual_controls = {}
        self.individual_controls_frame = tk.Frame(temp)
        self.individual_controls_frame.pack(side=tk.LEFT)

        thruster_names = [f"Thruster #{n}" for n in range(NUM_OF_THRUSTERS)]
        servos_names = [f"Servo #{n}" for n in range(NUM_OF_SERVOS)]

        self.input_controls(
            thruster_names, self.individual_controls, self.individual_controls_frame)
        self.input_controls(
            servos_names, self.individual_controls, self.individual_controls_frame)
        self.send_thruster_btn = tk.Button(
            self.individual_controls_frame, text="Send Thrusters")
        self.send_thruster_btn.pack(side=tk.LEFT)
        self.send_servos_btn = tk.Button(
            self.individual_controls_frame, text="Send Servos")
        self.send_servos_btn.pack(side=tk.LEFT)

        self.global_twist_set = {}
        self.global_twist_set_frame = tk.Frame(temp)
        self.global_twist_set_frame.pack(side=tk.LEFT)
        self.input_controls(['X pos', 'Y pos', 'Z pos'],
                            self.global_twist_set, self.global_twist_set_frame)
        self.input_controls(['Pitch', 'Yaw', 'Roll'],
                            self.global_twist_set, self.global_twist_set_frame)
        self.send_globa_pos_btn = tk.Button(
            self.global_twist_set_frame, text="Send Global Twist")
        self.send_globa_pos_btn.pack(fill=tk.BOTH)

        self.global_thrust_controls = {}
        self.global_thrust_alloc_frame = tk.Frame(temp)
        self.global_thrust_alloc_frame.pack(side=tk.LEFT)
        self.input_controls(['X Thrust', 'Y Thrust', 'Z Thrust', "X Torque", "Y Torque", "Z Torque"],
                            self.global_thrust_controls, self.global_thrust_alloc_frame)
        self.send_thrust_alloc_btn = tk.Button(
            self.global_thrust_alloc_frame, text="Send Thrust Alloc")
        self.send_thrust_alloc_btn.pack(fill=tk.BOTH)

    def input_controls(self, names, lookup, root):
        for c in names:
            name = f"Set {c}: "
            control_frame = tk.Frame(root)
            control_frame.pack(fill=tk.X)

            control_label = tk.Label(control_frame, text=name)
            control_label.pack(side=tk.LEFT)
            entry_value = tk.DoubleVar()
            entry_value.set(0.0)
            value_label = tk.Label(control_frame, text=entry_value.get())
            value_label.pack(side=tk.RIGHT, padx=10)
            entry_control = ttk.Entry(control_frame)
            entry_control.pack(side=tk.RIGHT)

            lookup[name] = {
                'variable': entry_value,
                'frame': control_frame,
                'name_label': control_label,
                'entry_control': entry_control,
                'value_label': value_label
            }

    def update(self):
        for lookup in [self.individual_controls, self.global_twist_set, self.global_thrust_controls]:
            for entry in lookup.values():
                entry_val = entry['entry_control'].get()
                if entry_val == "":
                    pass
                else:
                    try:
                        entry['variable'].set(float(entry_val))
                        entry['value_label'].configure(
                            text=f"{entry['variable'].get():.2f}")
                    except ValueError:
                        pass


class AuvConfigSettings:
    def __init__(self, root):
        config_data = AuvConfigSettings.load_config()

        self.textvar = tk.StringVar()
        self.entry: tk.Text = tk.Text(root)
        self.entry.insert(tk.END, json.dumps(config_data, indent=4))
        self.entry.pack(fill=tk.BOTH, expand=True)

    def update(config):
        with open(AUV_CONFIG_DIR, 'w') as file:
            json.dump(config, file, indent=4)

    def load_config():
        with open(AUV_CONFIG_DIR, 'r') as file:
            return json.load(file)


class RosHandler:
    def __init__(self, camera_frame, controls_frame):
        self.camera_frame: CameraFrame = camera_frame
        self.controls_frame: ControlsFrame = controls_frame

        rospy.init_node('haller_gui', anonymous=True)
        rospy.Subscriber('sim_image', ImageMsg,
                         self.camera_frame.update_img_msg)

        self.thruster_Sender = rospy.Publisher(
            THRUSTER_TOPIC, Float32MultiArray, queue_size=10)
        self.controls_frame.send_thruster_btn.configure(
            command=self.send_to_thrusters)
        self.servos_Sender = rospy.Publisher(
            SERVOS_TOPIC, Float32MultiArray, queue_size=10)
        self.controls_frame.send_servos_btn.configure(
            command=self.send_to_servos)
        self.global_position_Sender = rospy.Publisher(
            GLOBAL_POSITION_TOPIC, Twist, queue_size=10)
        self.controls_frame.send_globa_pos_btn.configure(
            command=self.send_global_pos)
        self.thrust_alloc_Sender = rospy.Publisher(
            THRUST_ALLOCATION_TOPIC, Twist, queue_size=10)
        self.controls_frame.send_thrust_alloc_btn.configure(
            command=self.send_thrust_alloc)

        self.thr_dim = MultiArrayDimension("dim", 5, 5)
        self.thr_lay = MultiArrayLayout([self.thr_dim], 0)
        self.srv_dim = MultiArrayDimension("dim", 2, 2)
        self.srv_lay = MultiArrayLayout([self.srv_dim], 0)
        self.alloc_dim = MultiArrayDimension("dim", 6, 6)
        self.alloc_lay = MultiArrayLayout([self.alloc_dim], 0)

    def send_to_thrusters(self):
        values = [float(thruster['variable'].get()) for name,
                  thruster in self.controls_frame.individual_controls.items() if 'Thruster' in name]
        thruster_msg = Float32MultiArray()
        thruster_msg.layout = self.thr_lay
        thruster_msg.data = values
        self.thruster_Sender.publish(thruster_msg)

    def send_to_servos(self):
        values = [servo['variable'].get(
        ) for name, servo in self.controls_frame.individual_controls.items()if 'Servo' in name]
        servo_msg = Float32MultiArray()
        servo_msg.layout = self.srv_lay
        servo_msg.data = values
        self.servos_Sender.publish(servo_msg)

    def send_global_pos(self):
        values = [coord['variable'].get(
        ) for name, coord in self.controls_frame.global_twist_set.items() if 'pos' in name]
        values_ang = [float(angle['variable'].get(
        )) for name, angle in self.controls_frame.global_twist_set.items() if 'pos' not in name]

        assert len(values) == 3 and len(values_ang) == 3
        global_pos_msg = Twist()
        global_pos_msg.linear = Vector3(values[0], values[1], values[2])
        global_pos_msg.angular = Vector3(
            values_ang[0], values_ang[1], values_ang[2])

        self.global_position_Sender.publish(global_pos_msg)

    def send_thrust_alloc(self):
        values = [float(thruster['variable'].get()) for name,
                  thruster in self.controls_frame.global_thrust_controls.items()]
        thrust_alloc_msg = Twist()
        thrust_alloc_msg.linear = Vector3(values[0], values[1], values[2])
        thrust_alloc_msg.angular = Vector3(values[3], values[4], values[5])
        self.thrust_alloc_Sender.publish(thrust_alloc_msg)


class HallerGui:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title('Haller GUI')
        self.root.geometry("1920x1080")

        self.tabs = MainGuiTabs(self.root)

        self.controls = ttk.Frame(self.tabs.tab_root)
        self.controls.pack(fill='both', expand=True)
        self.settings = ttk.Frame(self.tabs.tab_root)
        self.settings.pack(fill='both', expand=True)

        tabs = {
            "Controls": self.controls,
            "Settings": self.settings
        }
        self.tabs.add_tabs(tabs)

        self.camera_frame = CameraFrame(self.controls)
        self.controls_frame = ControlsFrame(self.controls)
        self.config = AuvConfigSettings(self.settings)

        self.ros_handler = RosHandler(self.camera_frame, self.controls_frame)

    def run(self):
        while True:
            self.controls_frame.update()
            self.camera_frame.update()

            self.root.update_idletasks()
            self.root.update()


if __name__ == "__main__":
    gui = HallerGui()
    gui.run()
