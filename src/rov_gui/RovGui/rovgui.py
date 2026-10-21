#!/usr/bin/env python3
# Run using: ros2 run rov_gui rovgui (تأكد أن ملف rov.ui موجود في مجلد التشغيل)
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from PyQt5 import uic
from PyQt5.QtCore import QObject, QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QMainWindow, QSizePolicy

from rov_interfaces.msg import Guireader   # <-- حزمة الرسائل واسم الرسالة


class RosGUI(Node, QObject):
    joystick = pyqtSignal(object)         # الرسالة كاملة
    frame = pyqtSignal(int, QImage)       # رقم الكاميرا والصورة

    def __init__(self):
        Node.__init__(self, "rov_dashboard")
        QObject.__init__(self)

        self.sub = self.create_subscription(Guireader, "joystick", self.topic_callback, 10)

    def topic_callback(self, msg):
        for i, name in enumerate(("camera_1", "camera_2", "camera_3", "camera_4", "oakd")):
            data = getattr(msg, f"{name}_image")
            w = getattr(msg, f"{name}_width")
            h = getattr(msg, f"{name}_height")
            if w * h > 0 and len(data) == w * h * 3:      # تخطي الصور الفارغة أو غير التامة
                self.frame.emit(i, QImage(bytes(data), w, h, 3 * w, QImage.Format_RGB888).copy())
        self.joystick.emit(msg)


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
    def __init__(self, node):
        super().__init__()
        uic.loadUi("rov.ui", self)

        self.thr_bars = [getattr(self, f"bar_thr{i}") for i in range(1, 9)]
        self.thr_labels = [getattr(self, f"lbl_thr{i}_value") for i in range(1, 9)]
        self.lbl_imu_temp.setText("--")   # لحين وصول أول رسالة

        self.cam_labels = [self.lbl_cam1, self.lbl_cam2, self.lbl_cam3, self.lbl_cam4, self.lbl_oakd]
        for label in self.cam_labels:
            label.setScaledContents(False)
            label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)

        # عدم استلام بيانات لمدة ثانيتين -> فصل الاتصال ولا يوجد إشارة
        self.watchdog = QTimer(self)
        self.watchdog.setSingleShot(True)
        self.watchdog.timeout.connect(self.on_timeout)
        self.on_timeout()

        # ربط الإشارات مع النافذة
        node.joystick.connect(self.on_joystick)
        node.frame.connect(self.show_frame)

        # تشغيل العقدة في خيط (Thread) منفصل
        self.thread = QThread()
        self.worker = RosWorker(node)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.thread.start()

    def on_joystick(self, msg):
        self.set_connected(msg.connected)
        self.watchdog.start(2000)

        for bar, label, value in zip(self.thr_bars, self.thr_labels, msg.thruster_output_percent):
            bar.setValue(int(value))
            label.setText(f"{value:.0f} %")

        self.lbl_light_indicator.setStyleSheet(f"color: {'#ffd54a' if msg.light_on else '#3a4656'}; font-size: 28pt;")
        self.lbl_light_state.setText("ON" if msg.light_on else "OFF")

        self.lbl_mode_indicator.setStyleSheet(f"color: {'#3ddc84' if msg.operating_mode else '#3a4656'}; font-size: 28pt;")
        self.lbl_mode_state.setText("AUTONOMOUS" if msg.operating_mode else "MANUAL")

        for n in (1, 2):
            getattr(self, f"bar_arm{n}_extension").setValue(int(msg.arm_extension_percent[n - 1]))
            getattr(self, f"bar_arm{n}_gripper").setValue(int(msg.gripper_opening_percent[n - 1]))

        for row, values in zip(("acc", "gyr", "mag"), (msg.accel_m_s2, msg.gyro_deg_s, msg.mag_ut)):
            for axis, value in zip("xyz", values):
                getattr(self, f"lbl_imu_{row}_{axis}").setText(f"{value:.2f}")
        self.lbl_imu_temp.setText(f"{msg.temperature_c:.1f} \u00B0C")

    def show_frame(self, index, image):
        label = self.cam_labels[index]
        label.setPixmap(QPixmap.fromImage(image).scaled(label.size(), Qt.KeepAspectRatio))

    def set_connected(self, ok):
        self.lbl_connection.setText("\u25CF CONNECTED" if ok else "\u25CF DISCONNECTED")
        self.lbl_connection.setStyleSheet(
            f"color: {'#3ddc84' if ok else '#ff5c5c'}; font-size: 11pt; font-weight: bold;")

    def on_timeout(self):
        self.set_connected(False)
        for label in self.cam_labels:
            label.setText("NO SIGNAL")

    def closeEvent(self, event):
        rclpy.shutdown()
        self.thread.quit()
        self.thread.wait()
        super().closeEvent(event)


def main(args=None):
    rclpy.init(args=args)
    node = RosGUI()
    app = QApplication(sys.argv)
    window = Dashboard(node)
    window.show()
    
    exit_code = app.exec_()
    
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()
        
    sys.exit(exit_code)


if __name__ == "__main__":
    main()