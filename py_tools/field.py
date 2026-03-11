import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.patches as patches
import matplotlib.transforms as transforms

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QGroupBox, QLineEdit)
from PyQt5.QtCore import QTimer

# ---------------------------------------------------------
# 1. 场地画布类 (封装你之前的绘图逻辑)
# ---------------------------------------------------------
class FieldCanvas(FigureCanvas):
    def __init__(self, parent=None, width=12, height=8, dpi=100):
        self.to_m = 0.0254
        self.field_l, self.field_w = 651.2 * self.to_m, 317.7 * self.to_m
        self.rob_size = 0.686

        # 创建 Figure
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.ax = self.fig.add_subplot(111)
        super().__init__(self.fig)
        
        self.robot_rect = None
        self.robot_arrow = None
        self.draw_static_field()

    def draw_static_field(self):
        self.ax.clear()
        # 绘制背景和边界
        self.ax.add_patch(patches.Rectangle((0, 0), self.field_l, self.field_w, color='green', alpha=0.1))
        self.ax.plot([0, self.field_l, self.field_l, 0, 0], [0, 0, self.field_w, self.field_w, 0], color='black', lw=2)
        
        # 调用你之前的对称逻辑绘制两侧设施 (这里简化演示)
        self._draw_side(True)  # Blue
        self._draw_side(False) # Red
        
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-0.5, self.field_l + 0.5)
        self.ax.set_ylim(-0.5, self.field_w + 0.5)

        # 中立区燃料堆
        fuel_w, fuel_d = 206.0 * self.to_m, 72.0 * self.to_m
        self.ax.add_patch(patches.Rectangle((self.field_l/2 - fuel_d/2, self.field_w/2 - fuel_w/2), 
                                         fuel_d, fuel_w, color='gold', alpha=0.2))

    def _draw_side(self, is_blue):
        x_base = 0 if is_blue else self.field_l
        direction = 1 if is_blue else -1
        color = 'blue' if is_blue else 'red'
        
        # 尺寸参数
        outpost_w = 71.0 * self.to_m
        tower_w = 49.25 * self.to_m
        ds_w = (self.field_w - outpost_w - tower_w) / 3
        
        # --- 对角线对称逻辑修正 ---
        # 蓝方从下往上排：OUTPOST -> DS3 -> TOWER -> DS2 -> DS1
        # 红方从上往下排：OUTPOST -> DS3 -> TOWER -> DS2 -> DS1 (即 Y 坐标反转)
        y_order = [
            ('OUTPOST', outpost_w, color),
            ('DS 3', ds_w, 'gray'),
            ('TOWER', tower_w, color),
            ('DS 2', ds_w, 'gray'),
            ('DS 1', ds_w, 'gray')
        ]

        current_y = 0 if is_blue else self.field_w
        
        for name, width, fill_color in y_order:
            # 如果是红方，向上叠加变向向下减去宽度
            draw_y = current_y if is_blue else current_y - width
            
            # 绘制墙体
            self.ax.add_patch(patches.Rectangle((x_base - (0.05 if not is_blue else 0), draw_y), 0.05, width, color='black'))
            self.ax.text(x_base + 0.3*direction, draw_y + width/2, name, rotation=90, 
                         va='center', ha='center', fontsize=7, color=fill_color if fill_color != 'gray' else 'black')
            
            # 塔楼结构
            if name == 'TOWER':
                t_depth, t_width = 45.18 * self.to_m, 39.0 * self.to_m
                self.ax.add_patch(patches.Rectangle((x_base, draw_y + (width - t_width)/2), 
                                                 t_depth * direction, t_width, facecolor=color, alpha=0.2))
            
            # 更新下一个组件的起始 Y
            if is_blue: current_y += width
            else: current_y -= width

        # --- 仓库 (DEPOT) 对称修正 ---
        # 蓝方在最上方，红方在最下方
        depot_w, depot_d = 42.0 * self.to_m, 27.0 * self.to_m
        depot_y = (self.field_w - depot_w) if is_blue else 0
        self.ax.add_patch(patches.Rectangle((x_base, depot_y), 
                                         depot_d * direction, depot_w, edgecolor=color, facecolor='none', hatch='///'))

        # --- 中心障碍区 (HUB, BUMP, TRENCH) ---
        hub_x = 158.6 * self.to_m if is_blue else self.field_l - 158.6 * self.to_m
        hub_dim = 47.0 * self.to_m
        bump_w, bump_d = 73.0 * self.to_m, 44.4 * self.to_m
        trench_w, trench_d = 65.65 * self.to_m, 47.0 * self.to_m
        
        self.ax.add_patch(patches.Rectangle((hub_x - hub_dim/2, (self.field_w - hub_dim)/2), hub_dim, hub_dim, color='black', alpha=0.3))
        self.ax.add_patch(patches.Rectangle((hub_x - bump_d/2, (self.field_w + hub_dim)/2), bump_d, bump_w, color=color, alpha=0.4))
        self.ax.add_patch(patches.Rectangle((hub_x - bump_d/2, (self.field_w - hub_dim)/2 - bump_w), bump_d, bump_w, color=color, alpha=0.4))
        self.ax.add_patch(patches.Rectangle((hub_x - trench_d/2, self.field_w - trench_w), trench_d, trench_w, color=color, alpha=0.15))
        self.ax.add_patch(patches.Rectangle((hub_x - trench_d/2, 0), trench_d, trench_w, color=color, alpha=0.15))

    # def update_robot_pose(self, x, y, yaw):
    #     if self.robot_rect is None:
    #         self.robot_rect = patches.Rectangle((-self.rob_size/2, -self.rob_size/2), 
    #                                           self.rob_size, self.rob_size, color='green', alpha=0.8, zorder=20)
    #         self.robot_arrow = patches.Arrow(0, 0, 0.5, 0, width=0.2, color='gold', zorder=21)
    #         self.ax.add_patch(self.robot_rect)
    #         self.ax.add_patch(self.robot_arrow)

    #     t = transforms.Affine2D().rotate_deg(yaw).translate(x, y) + self.ax.transData
    #     self.robot_rect.set_transform(t)
    #     self.robot_arrow.set_transform(t)
    #     self.draw_idle()
    def update_robot_pose(self, x, y, yaw):
        # 记录当前位姿，方便其他函数调用
        self._last_rob_x = x
        self._last_rob_y = y
        self._last_rob_yaw = yaw

        if self.robot_rect is None:
            self.robot_rect = patches.Rectangle((-self.rob_size/2, -self.rob_size/2), 
                                              self.rob_size, self.rob_size, color='green', alpha=0.8, zorder=20)
            self.robot_arrow = patches.Arrow(0, 0, 0.5, 0, width=0.2, color='gold', zorder=21)
            self.ax.add_patch(self.robot_rect)
            self.ax.add_patch(self.robot_arrow)

        t = transforms.Affine2D().rotate_deg(yaw).translate(x, y) + self.ax.transData
        self.robot_rect.set_transform(t)
        self.robot_arrow.set_transform(t)
        self.draw_idle()

    def update_shoot_target_pos(self, x, y):
        """在指定坐标描绘一个 X 图形"""
        if not hasattr(self, 'target_cross'):
            # 创建两条线段组成 X，并添加到坐标轴
            self.target_cross, = self.ax.plot([], [], 'rx', markersize=12, markeredgewidth=2, zorder=30)
        
        self.target_cross.set_data([x], [y])
        self.draw_idle()
        
    def upate_aim_dir(self, offset_x, offset_y, turret_degree):
        """
        现在只需要传入炮台相对于底盘的偏移和角度
        机器人位姿自动从类成员变量读取
        """
        # 安全检查：确保已经调用过 update_robot_pose
        if not hasattr(self, '_last_rob_x'):
            return

        ray_length = 4.0
        if not hasattr(self, 'aim_ray'):
            self.aim_ray, = self.ax.plot([], [], color='red', linestyle='--', linewidth=1.5, alpha=0.7, zorder=25)

        # 1. 计算局部坐标（炮台系）
        rad = np.radians(turret_degree)
        local_end_x = offset_x + ray_length * np.cos(rad)
        local_end_y = offset_y + ray_length * np.sin(rad)

        # 2. 使用存储的底盘位姿进行变换
        t = transforms.Affine2D().rotate_deg(self._last_rob_yaw).translate(self._last_rob_x, self._last_rob_y)
        
        start_global = t.transform([offset_x, offset_y])
        end_global = t.transform([local_end_x, local_end_y])

        self.aim_ray.set_data([start_global[0], end_global[0]], [start_global[1], end_global[1]])
        self.draw_idle()

# ---------------------------------------------------------
# 2. 主窗口类 (添加 UI 组件)
# ---------------------------------------------------------
class RobotDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("FRC 2026 'REBUILT' - Engineering Dashboard")
        
        # 主布局
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QHBoxLayout(main_widget)

        # --- 左侧：场地 ---
        self.canvas = FieldCanvas(self)
        layout.addWidget(self.canvas, stretch=4)

        # --- 右侧：控制面板 ---
        panel_layout = QVBoxLayout()
        
        # 状态组
        status_group = QGroupBox("Robot Status")
        status_vbox = QVBoxLayout()
        self.label_pos = QLabel("X: 0.00  Y: 0.00  Yaw: 0.0°")
        status_vbox.addWidget(self.label_pos)
        status_group.setLayout(status_vbox)
        panel_layout.addWidget(status_group)

        # 控制组
        ctrl_group = QGroupBox("Controls")
        ctrl_vbox = QVBoxLayout()
        self.btn_reset = QPushButton("Reset Odometry")
        self.btn_reset.clicked.connect(self.reset_robot)
        ctrl_vbox.addWidget(self.btn_reset)
        ctrl_group.setLayout(ctrl_vbox)
        panel_layout.addWidget(ctrl_group)

        panel_layout.addStretch() # 推到顶部
        layout.addLayout(panel_layout, stretch=1)

        # 定时器：模拟数据更新
        self.timer = QTimer()
        self.timer.timeout.connect(self.simulate_telemetry)
        self.timer.start(50) # 20Hz
        
        self.sim_t = 0

    def simulate_telemetry(self):
        # 模拟机器人动作
        self.sim_t += 0.05
        x = 4.0 + 2 * np.cos(self.sim_t)
        y = 4.0 + 2 * np.sin(self.sim_t)
        yaw = np.degrees(self.sim_t)
        
        self.canvas.update_robot_pose(x, y, yaw)
        self.label_pos.setText(f"X: {x:.2f}  Y: {y:.2f}  Yaw: {yaw:.1f}°")

    def reset_robot(self):
        self.sim_t = 0
        print("Odometry Reset!")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotDashboard()
    window.showMaximized()
    sys.exit(app.exec_())