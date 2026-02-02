
import nt4 as nt4
# 注意，nt4和PyQt5有冲突，nt4必须最先import,否则程序闪退！！！
# 注意，nt4和PyQt5有冲突，nt4必须最先import,否则程序闪退！！！
# 注意，nt4和PyQt5有冲突，nt4必须最先import,否则程序闪退！！！
import sys
from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import QTimer

import field as field
import console as console


class MyWindow(QtWidgets.QMainWindow):
    
    def __init__(self):
        self.fieldCanvas = None

        super(MyWindow, self).__init__()
        # 加载 UI 文件
        uic.loadUi('./ui/frame.ui', self) 
        
        # 现在你可以直接通过 objectName 访问 UI 里的控件
        # 例如：self.myButton.clicked.connect(self.handleClick)

        self.fieldCanvas = field.FieldCanvas(self)
        self.widget_up.layout().addWidget(self.fieldCanvas)

        self.console_ui = console.ConsoleWidget()
        self.widget_down.layout().addWidget(self.console_ui)

        self.splitter_set = True
        # 主 splitter: 左30% 右70%
        w = self.splitter_h.width()
        self.splitter_h.setSizes([int(w * 0.7), int(w * 0.3)])
        self.splitter_h.setStretchFactor(0, 7)
        self.splitter_h.setStretchFactor(1, 3)
        # 左侧 splitter: 上60% 下40%
        # h = self.splitter_v.height()
        # self.splitter_v.setSizes([int(h * 0.7), int(h * 0.3)])

        self.splitter_v.setStretchFactor(0, 5)  # 第0个部件占5份
        self.splitter_v.setStretchFactor(1, 5)  # 第1个部件占5份

        self.show()

        QTimer.singleShot(0, lambda: self.splitter_v.refresh())
        # QTimer.singleShot(0, lambda: self.splitter_v.setSizes([100, 1]))

        # TODO: 这里上下左右分割比例的计算是有问题的。

        self.timer = QTimer(self)
        # self.timer.timeout.connect(self.update)  # 每次超时调用 update()
        self.timer.start(100)  # 每100毫秒触发一次（即10Hz）
        self.timer.timeout.connect(self.my_custom_update)


    def update_robot_info(self):
        robotPosData = nt4.default_nt4.get_robot_pose()
        if nt4.default_nt4.is_connected() == False or robotPosData is None:
            self.label_robotPos.setText("Pos X:--, Y:--")
            self.label_robotRot.setText("Rot deg:--")
        else:
            x, y = robotPosData.x, robotPosData.y
            rot = robotPosData.rotation()
            deg = rot.degrees()
            self.label_robotPos.setText(f'Pos X:{x:0.2f}, Y:{y:0.2f}')
            self.label_robotRot.setText(f"Rot deg:{deg:0.2f}")

            self.fieldCanvas.update_robot_pose(x, y, deg)

    def update_connect_status(self):
        if nt4.default_nt4.is_connected():
            self.label_status.setText("Connected")
            self.label_status.setStyleSheet("Color:green")
        else:
            self.label_status.setText("Connecting")
            self.label_status.setStyleSheet("Color:red")
    def my_custom_update(self):
        self.update_connect_status()
        self.update_robot_info()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec())