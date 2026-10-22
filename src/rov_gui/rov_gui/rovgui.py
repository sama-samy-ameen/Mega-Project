#!/usr/bin/env python3
"""
ROV control station GUI.
Run from the folder that contains rov_gui.ui:   python3 rov_gui.py

RosGUI    - the ROS node: one subscription per topic, it only emits Qt signals.
RosWorker - spins the node in its own QThread.
Dashboard - the window: one set_* method per feature.
"""
import sys
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from PyQt5 import uic
from PyQt5.QtCore import QObject, QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QListWidgetItem, QMainWindow, QSizePolicy

from sensor_msgs.msg import Image                       # camera placeholder
from std_msgs.msg import Float32, Float32MultiArray     # IMU / depth / PID placeholders
from rov_interfaces.msg import ActuatorsCommands, AutonomousCommand, ThrustersSpeeds

PWM_REST = 1500      # T200: 1200..1800 us, 1500 = stopped
PWM_SPAN = 300       # +-300 us = 100 %
PID_DEFAULTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # first values of the PID boxes [yaw P, I, D, pitch P, I, D]
TIME_STYLE = "color: {}; font-family: 'Consolas','DejaVu Sans Mono',monospace; font-size: 30pt; font-weight: bold;"


class RosGUI(Node, QObject):
    connected_changed = pyqtSignal(bool)
    thrusters_changed = pyqtSignal(list)           # 8 PWM values, in the order of the bars
    grippers_changed = pyqtSignal(list)            # 6 bools (p1..p6), True = open
    mode_changed = pyqtSignal(bool)                # True = autonomous
    state_changed = pyqtSignal(str)                # autonomous state text
    orientation_changed = pyqtSignal(float, float, float)   # roll, pitch, yaw in degrees
    depth_changed = pyqtSignal(float)              # metres
    frame_changed = pyqtSignal(int, QImage)        # camera 0..4 (4 = OAK-D), picture

    def __init__(self):
        Node.__init__(self, "rov_gui")
        QObject.__init__(self)

        def sub(msg_type, topic, callback):
            # a message from any topic also means "connected"
            def wrapper(msg):
                self.connected_changed.emit(True)
                callback(msg)
            self.create_subscription(msg_type, topic, wrapper, 10)

        # ---- your topics ----
        sub(ThrustersSpeeds, "/motion", self.motion_callback)
        sub(ActuatorsCommands, "/actuators", self.actuators_callback)
        sub(AutonomousCommand, "/autonomous_command", self.autonomous_callback)

        # ---- placeholders: no topic / message yet, change them when you have them ----
        sub(Float32MultiArray, "imu_roll_pitch_yaw", lambda m: self.orientation_changed.emit(*m.data[:3]))
        sub(Float32, "depth", lambda m: self.depth_changed.emit(m.data))
        for i, topic in enumerate(("camera_1", "camera_2", "camera_3", "camera_4", "oakd")):
            sub(Image, topic, lambda m, i=i: self.camera_callback(i, m))

        # ---- publisher: the PID boxes ----
        self.gui_pub = self.create_publisher(Float32MultiArray, "gui_control", 10)

    def motion_callback(self, m):
        # same order as the bars in the window: TFL TFR BFL BFR TBL TBR BBL BBR
        self.thrusters_changed.emit([m.tfl, m.tfr, m.bfl, m.bfr, m.tbl, m.tbr, m.bbl, m.bbr])

    def actuators_callback(self, m):
        self.grippers_changed.emit([m.p1, m.p2, m.p3, m.p4, m.p5, m.p6])     # motor and command are ignored

    def autonomous_callback(self, m):
        self.mode_changed.emit(m.autonomous_mode)
        self.state_changed.emit("AUTONOMOUS MODE" if m.autonomous_mode else "MANUAL MODE")

    def camera_callback(self, index, msg):
        # TODO placeholder: reads a sensor_msgs/Image (rgb8). Change it for your camera message.
        if msg.width > 0 and msg.height > 0 and len(msg.data) >= msg.step * msg.height:
            image = QImage(bytes(msg.data), msg.width, msg.height, msg.step, QImage.Format_RGB888)
            self.frame_changed.emit(index, image.copy())

    def publish_pid(self, values):
        # TODO placeholder message type: Float32MultiArray = [yaw P, I, D, pitch P, I, D] on "gui_control"
        msg = Float32MultiArray()
        msg.data = [float(v) for v in values]
        self.gui_pub.publish(msg)


class RosWorker(QObject):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        try:
            rclpy.spin(self.node)
        except ExternalShutdownException:
            pass


class Dashboard(QMainWindow):
    pid_changed = pyqtSignal(list)    # [yaw P, I, D, pitch P, I, D]

    def __init__(self, node):
        super().__init__()
        ui_path = os.path.join(get_package_share_directory("rov_gui"), "rov.ui")
        uic.loadUi(ui_path, self)

        self.thr_bars = [getattr(self, f"bar_thr{i}") for i in range(1, 9)]
        self.thr_labels = [getattr(self, f"lbl_thr{i}_value") for i in range(1, 9)]
        self.grip_extend = [getattr(self, f"lbl_gripper{n}_extend") for n in range(1, 7)]
        self.grip_open = [getattr(self, f"lbl_gripper{n}_open") for n in range(1, 7)]
        self.pid_spins = [getattr(self, f"spin_{c}_{k}") for c in ("yaw", "pitch") for k in "pid"]
        self.steps = []                   # the autonomous states received so far
        self.connected = None

        # cameras: the picture is scaled in code (keeps its aspect ratio); NO SIGNAL after 2 s without a frame
        self.cam_labels = [self.lbl_cam1, self.lbl_cam2, self.lbl_cam3, self.lbl_cam4, self.lbl_oakd]
        self.cam_timers = []
        for label in self.cam_labels:
            label.setScaledContents(False)
            label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
            timer = QTimer(self)
            timer.setSingleShot(True)
            timer.timeout.connect(lambda l=label: l.setText("NO SIGNAL"))
            self.cam_timers.append(timer)

        # connection: ON when any topic delivers a message, OFF after 3 s of silence
        self.conn_timer = QTimer(self)
        self.conn_timer.setSingleShot(True)
        self.conn_timer.timeout.connect(lambda: self.set_connected(False))
        self.set_connected(False)

        # trial + mission countdowns: both run here in the GUI, the buttons control both
        self.time_edits = {"trial": self.time_trial_duration, "mission": self.time_mission_duration}
        self.time_labels = {"trial": self.lbl_trial_time, "mission": self.lbl_mission_time}
        self.time_left = {"trial": 0, "mission": 0}
        self.countdown = QTimer(self)
        self.countdown.timeout.connect(self.on_tick)
        self.btn_mission_start.clicked.connect(self.start_timers)
        self.btn_mission_pause.clicked.connect(self.pause_timers)
        self.btn_mission_reset.clicked.connect(self.reset_timers)
        for edit in self.time_edits.values():
            edit.timeChanged.connect(self.reset_timers)      # a new duration resets both timers
        self.reset_timers()

        # PID boxes -> pid_changed -> node.publish_pid
        self.set_pid_values(PID_DEFAULTS)
        for spin in self.pid_spins:
            spin.valueChanged.connect(self.on_pid_edited)
        self.pid_changed.connect(node.publish_pid)

        self.set_light(False)             # the LED is always off for now

        # ROS node -> signals -> the set_* methods of this window
        node.connected_changed.connect(self.set_connected)
        node.thrusters_changed.connect(self.set_thruster_speeds)
        node.grippers_changed.connect(self.set_grippers)
        node.mode_changed.connect(self.set_mode)
        node.state_changed.connect(self.set_state)
        node.orientation_changed.connect(self.set_orientation)
        node.depth_changed.connect(self.set_depth)
        node.frame_changed.connect(self.show_frame)

        # spin the node in its own thread
        self.thread = QThread()
        self.worker = RosWorker(node)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.thread.start()

    # ------------------------------------------------------------------
    # one method per feature
    # ------------------------------------------------------------------
    def set_connected(self, ok):
        if ok:
            self.conn_timer.start(3000)
        if ok == self.connected:
            return
        self.connected = ok
        self.lbl_connection.setText("\u25CF CONNECTED" if ok else "\u25CF DISCONNECTED")
        self.lbl_connection.setStyleSheet(
            f"color: {'#3ddc84' if ok else '#ff5c5c'}; font-size: 11pt; font-weight: bold;")

    def set_thruster_speeds(self, pwm_values):        # 8 PWM values, 1200..1800, 1500 = stopped
        for bar, label, pwm in zip(self.thr_bars, self.thr_labels, pwm_values):
            percent = max(-100.0, min(100.0, (pwm - PWM_REST) / PWM_SPAN * 100.0))
            bar.setValue(int(percent))
            label.setText(f"{percent:.0f}")

    def set_grippers(self, states):                   # 6 bools: True = open (extended + open), False = closed (retracted + closed)
        for n, is_open in enumerate(states):
            self.badge(self.grip_extend[n], "EXTENDED" if is_open else "RETRACTED", is_open)
            self.badge(self.grip_open[n], "OPEN" if is_open else "CLOSED", is_open)

    def set_mode(self, autonomous):                   # True = autonomous, False = manual
        self.lbl_mode_indicator.setStyleSheet(f"color: {'#3ddc84' if autonomous else '#3a4656'}; font-size: 24pt;")
        self.lbl_mode_state.setText("AUTONOMOUS" if autonomous else "MANUAL")

    def set_state(self, text):                        # autonomous state: "searching", "positioning", "item found" ...
        if not text:
            return
        self.lbl_mission_name.setText(text)
        if not self.steps or self.steps[-1] != text:  # a new state adds a line to the list
            self.steps.append(text)
            self.refresh_steps()

    def set_light(self, on):                          # the LED indicator
        self.lbl_light_indicator.setStyleSheet(f"color: {'#ffd54a' if on else '#3a4656'}; font-size: 24pt;")
        self.lbl_light_state.setText("ON" if on else "OFF")

    def set_motion_state(self, text):                 # not connected for now
        self.lbl_motion_state.setText(text)

    def set_orientation(self, roll, pitch, yaw):      # degrees
        self.lbl_roll.setText(f"{roll:.1f}\u00B0")
        self.lbl_pitch.setText(f"{pitch:.1f}\u00B0")
        self.lbl_yaw.setText(f"{yaw:.1f}\u00B0")

    def set_depth(self, metres):
        self.lbl_depth.setText(f"{metres:.2f} m")

    def set_pid_values(self, values):                 # [yaw P, I, D, pitch P, I, D], does not send anything
        for spin, value in zip(self.pid_spins, values):
            spin.blockSignals(True)
            spin.setValue(value)
            spin.blockSignals(False)

    def show_frame(self, index, image):               # index 0..4 (4 = OAK-D)
        label = self.cam_labels[index]
        label.setPixmap(QPixmap.fromImage(image).scaled(label.size(), Qt.KeepAspectRatio))
        self.cam_timers[index].start(2000)

    # ------------------------------------------------------------------
    # trial + mission countdowns
    # ------------------------------------------------------------------
    def duration(self, key):
        t = self.time_edits[key].time()
        return t.minute() * 60 + t.second()

    def start_timers(self):
        if all(left <= 0 for left in self.time_left.values()):    # both finished: start again from the durations
            self.load_durations()
        self.countdown.start(1000)

    def pause_timers(self):
        self.countdown.stop()

    def reset_timers(self):
        self.countdown.stop()
        self.load_durations()

    def load_durations(self):
        for key in self.time_left:
            self.time_left[key] = self.duration(key)
        self.show_times()

    def on_tick(self):
        for key in self.time_left:
            if self.time_left[key] > 0:
                self.time_left[key] -= 1
        self.show_times()
        if all(left <= 0 for left in self.time_left.values()):
            self.countdown.stop()

    def show_times(self):                             # red when a time is over
        for key, label in self.time_labels.items():
            left = self.time_left[key]
            label.setText(f"{left // 60:02d}:{left % 60:02d}")
            label.setStyleSheet(TIME_STYLE.format("#ff5c5c" if left == 0 else "#ffffff"))

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def badge(self, label, text, active):
        label.setText(text)
        label.setStyleSheet("color: #ffffff; font-weight: bold; border-radius: 4px; padding: 2px 6px; "
                            f"background-color: {'#1f7a4d' if active else '#3a4656'};")

    def refresh_steps(self):                          # newest state highlighted, older ones done
        self.list_steps.clear()
        last = len(self.steps) - 1
        for i, text in enumerate(self.steps):
            mark = "\u25B6" if i == last else "\u2714"
            item = QListWidgetItem(f"{mark}  {text}")
            item.setForeground(QColor("#ffd54a" if i == last else "#6b7a8c"))
            self.list_steps.addItem(item)
        self.list_steps.scrollToBottom()

    def on_pid_edited(self):
        self.pid_changed.emit([spin.value() for spin in self.pid_spins])

    def closeEvent(self, event):
        rclpy.shutdown()          # makes rclpy.spin() return
        self.thread.quit()
        self.thread.wait()
        super().closeEvent(event)


def main(args=None):
    rclpy.init()
    node = RosGUI()
    app = QApplication(sys.argv)
    window = Dashboard(node)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()