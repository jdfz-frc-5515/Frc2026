import sys
import socket
import datetime
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPlainTextEdit, 
                             QVBoxLayout, QWidget, QLabel, QHBoxLayout,
                             QPushButton, QListWidget, QListWidgetItem, QCheckBox)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QFont, QTextCursor, QColor
from PyQt5 import uic

# 颜色配置映射
COLOR_MAP = {
    0: "#ffffff",  # 白色 - 默认
    1: "#00ff00",  # 绿色 - 信息
    2: "#ffff00",  # 黄色 - 警告
    3: "#ff0000",  # 红色 - 错误
    4: "#00ffff",  # 青色 - 调试
    5: "#ff00ff",  # 紫色 - 特殊
    6: "#ffa500",  # 橙色 - 事件
    7: "#800080",  # 紫罗兰 - 状态
}

# 通用深灰边框（推荐）
BORDER_COLOR_MAP = {
    0: "#555555",  # 白色按钮 → 深灰边框（否则看不见）
    1: "#006600",  # 绿色 → 深绿边框（可选），但深灰也OK
    2: "#8B6F00",  # 黄色 → 深金/棕（避免纯黑太重）
    3: "#8B0000",  # 红色 → 深红
    4: "#006666",  # 青色 → 深青
    5: "#660066",  # 紫色 → 深紫
    6: "#8B4500",  # 橙色 → 深橙/棕
    7: "#4B004B",  # 紫罗兰 → 深紫罗兰
}

PORT = 7777

# ---------------------------------------------------------
# 后台线程：专门负责监听来自 RoboRIO 的 UDP 数据
# ---------------------------------------------------------
class UdpListener(QThread):
    # 定义一个信号，当收到消息时发送给主界面
    message_received = pyqtSignal(str)

    def run(self):
        # FRC NetConsole 默认端口是 7777
        # 绑定到所有接口
        server_address = ("", PORT)
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 允许端口复用（防止报错 Address already in use）
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            sock.bind(server_address)
            print(f"开始监听 UDP 端口 {PORT}...")
        except Exception as e:
            self.message_received.emit(f"错误: 无法绑定端口 {PORT}. 可能是被占用。\n{e}")
            return

        while True:
            try:
                # 接收数据 (通常 FRC 日志包不会太大，4096 足够)
                data, address = sock.recvfrom(4096)
                # 尝试多种编码方案
                decoded_msg = ""
                for encoding in ['utf-8', 'gbk', 'gb18030']:
                    try:
                        decoded_msg = data.decode(encoding)
                        break 
                    except UnicodeDecodeError:
                        continue
                
                if not decoded_msg:
                    # 如果都失败了，强制转换并替换错误字符
                    decoded_msg = data.decode('utf-8', errors='replace')

                if decoded_msg.strip():
                    self.message_received.emit(decoded_msg.strip())

            except Exception as e:
                print(f"接收错误: {e}")

# ---------------------------------------------------------
# 控制台核心部件（只负责显示信息）
# ---------------------------------------------------------
class ConsoleWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_ui()
        
        # 存储所有消息
        self.all_messages = []  # [(category_id, full_text), ...]
        
        # 默认显示所有类别
        self.on_filter_all()
        
        # 启动后台监听线程
        self.listener = UdpListener()
        self.listener.message_received.connect(self.handle_message)
        self.listener.start()

    def load_console_hud(self):
        ui = uic.loadUi('./ui/console.ui')
        for i in range(8):
            button = getattr(ui, f'filter_{i}')
            button.setCheckable(True)  # 启用切换功能
            QPushButton
            # 设置基础样式（未选中状态）
            button.setStyleSheet(f"""
                QPushButton {{
                    background-color: {COLOR_MAP[i]};
                    color: black;
                    border: none;
                    padding: 2px;  /* 较小的内边距 */
                    border-radius: 4px;
                    min-width: 20px;   /* ← 关键：设置最小宽度 */
                    min-height: 20px;  /* 可选：固定最小高度，防止变化 */
                }}
                QPushButton:checked {{
                    border: 2px solid {BORDER_COLOR_MAP[i]};  /* 选中时的黑色边框 */
                    border-radius: 4px;
                    padding: 0px;
                }}
            """)
            button.setChecked(True)
            button.clicked.connect(lambda checked, lvl=i: self.on_filter_click(checked, lvl))

        ui.filter_all.clicked.connect(lambda _: self.on_filter_all())
        ui.filter_none.clicked.connect(lambda _: self.on_filter_none())
        return ui
    
    def on_filter_click(self, checked, lvl):
        self.refresh_filter()

    def on_filter_all(self):
        self.refresh_filter()
        for i in range(8):
            button = getattr(self.hud, f'filter_{i}')
            button.setChecked(True)
        self.refresh_filter()

    def on_filter_none(self):
        for i in range(8):
            button = getattr(self.hud, f'filter_{i}')
            button.setChecked(False)
        self.refresh_filter()

    def refresh_filter(self):
        lst = []
        for i in range(8):
            button = getattr(self.hud, f'filter_{i}')
            if (button.isChecked()):
                lst.append(i)
        self.set_visible_categories(lst)

    def setup_ui(self):
        """设置UI布局"""
        layout = QVBoxLayout(self)
        
        self.hud = self.load_console_hud()
        layout.addWidget(self.hud)

        # 顶部标题
        self.hud.status_label.setText(f"正在监听 RoboRIO Output (UDP {PORT})...")
        # self.status_label = QLabel(f"正在监听 RoboRIO Output (UDP {PORT})...")
        # self.status_label.setStyleSheet("color: #888; font-weight: bold;")
        # layout.addWidget(self.status_label)

        # 日志显示区域 (使用 QPlainTextEdit 性能更好)
        self.text_area = QPlainTextEdit()
        self.text_area.setReadOnly(True)
        # 设置黑色背景，白色字体（黑客风格），增大字体方便看
        self.text_area.setStyleSheet("""
            QPlainTextEdit {
                background-color: #1e1e1e;
                color: #ffffff;
                font-family: 'Consolas', 'Microsoft YaHei', monospace;
                font-size: 14px;
                border: 1px solid #333;
            }
        """)
        # 设置最大行数，防止内存无限增长
        self.text_area.setMaximumBlockCount(5000)
        layout.addWidget(self.text_area)

    def parse_message_with_category(self, text):
        """
        解析消息，提取类别标识符
        格式: "<--标识符-->正文"
        """
        if text.startswith("<--") and "-->" in text:
            end_marker_idx = text.find("-->")
            try:
                # 提取标识符
                category_str = text[3:end_marker_idx]
                category_id = int(category_str)
                # 提取正文
                content = text[end_marker_idx+3:].lstrip()
                return category_id, content
            except ValueError:
                # 如果标识符不是数字，则返回默认值
                return 0, text
        else:
            # 如果没有标识符，返回默认类别
            return 0, text

    def handle_message(self, text):
        """处理接收到的消息"""
        category_id, content = self.parse_message_with_category(text)
        
        # 生成带时间戳的完整文本
        timestamp = datetime.datetime.now().strftime("[%H:%M:%S.%f]")[:-3] # 毫秒只保留3位
        full_text = f"{timestamp} [{category_id}] -> {content}"
        
        # 存储消息
        self.all_messages.append((category_id, full_text))
        
        # 如果该类别被过滤掉，则不显示
        if category_id not in self.visible_categories:
            return
            
        # 添加带颜色的文本
        self.append_colored_log(full_text, category_id)

    def append_colored_log(self, text, category_id):
        """追加带颜色的日志"""
        # 获取颜色
        color = COLOR_MAP.get(category_id, "#ffffff")
        
        # 获取滚动条状态
        scrollbar = self.text_area.verticalScrollBar()
        is_at_bottom = (scrollbar.value() >= (scrollbar.maximum() - 10))
        
        # 获取文本光标
        cursor = self.text_area.textCursor()
        cursor.movePosition(QTextCursor.End)
        
        # 设置颜色格式
        fmt = cursor.charFormat()
        fmt.setForeground(QColor(color))
        cursor.setCharFormat(fmt)
        
        # 插入文本
        cursor.insertText(text + "\n")
        
        # 根据滚动位置决定是否滚到底部
        if is_at_bottom:
            scrollbar.setValue(scrollbar.maximum())

    def set_visible_categories(self, categories):
        """设置可见的消息类别"""
        self.visible_categories = set(categories)
        self.refresh_display()  # 刷新显示
        
    def get_visible_categories(self):
        """获取当前可见的消息类别"""
        return list(self.visible_categories)
        
    def refresh_display(self):
        """刷新显示，根据当前的可见类别重新渲染所有消息"""
        # 清空文本区域
        self.text_area.clear()
        
        # 重新显示符合当前过滤条件的消息
        for category_id, full_text in self.all_messages:
            if category_id in self.visible_categories:
                # 重新应用颜色
                color = COLOR_MAP.get(category_id, "#ffffff")
                
                # 获取文本光标
                cursor = self.text_area.textCursor()
                cursor.movePosition(QTextCursor.End)
                
                # 设置颜色格式
                fmt = cursor.charFormat()
                fmt.setForeground(QColor(color))
                cursor.setCharFormat(fmt)
                
                # 插入文本
                cursor.insertText(full_text + "\n")

    def clear_logs(self):
        """清空日志"""
        self.all_messages = []
        self.text_area.clear()


# ---------------------------------------------------------
# 主窗口（包含控制UI）
# ---------------------------------------------------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("FRC Console with Filtering")
        self.resize(1200, 800)
        
        # 创建中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QHBoxLayout(central_widget)
        
        # 左侧控制面板
        control_panel = QWidget()
        control_panel.setMaximumWidth(200)
        control_layout = QVBoxLayout(control_panel)
        
        # 分类过滤器
        filter_label = QLabel("显示类别:")
        control_layout.addWidget(filter_label)
        
        # 创建类别过滤列表
        self.category_list = QListWidget()
        control_layout.addWidget(self.category_list)
        
        # 添加类别到过滤器
        for category_id, color in COLOR_MAP.items():
            item = QListWidgetItem(f"ID: {category_id}")
            item.setData(Qt.UserRole, category_id)
            item.setCheckState(Qt.Checked)  # 默认全部选中
            self.category_list.addItem(item)
        
        # 连接类别选择变化信号
        self.category_list.itemChanged.connect(self.update_category_filter)
        
        # 清空按钮
        clear_button = QPushButton("清空日志")
        clear_button.clicked.connect(self.clear_logs)
        control_layout.addWidget(clear_button)
        
        # 添加到主布局
        main_layout.addWidget(control_panel)
        
        # 右侧主内容区
        content_layout = QVBoxLayout()
        
        # 添加标签
        label = QLabel("FRC Console Widget Demo")
        label.setStyleSheet("font-size: 16px; font-weight: bold; margin: 10px;")
        content_layout.addWidget(label)
        
        # 创建并添加控制台部件
        self.console_widget = ConsoleWidget()
        content_layout.addWidget(self.console_widget)
        
        main_layout.addLayout(content_layout)

    def update_category_filter(self, item):
        """更新可见类别"""
        # 收集所有选中的类别
        visible_categories = []
        for i in range(self.category_list.count()):
            list_item = self.category_list.item(i)
            if list_item.checkState() == Qt.Checked:
                visible_categories.append(list_item.data(Qt.UserRole))
        
        # 更新控制台部件的可见类别
        self.console_widget.set_visible_categories(visible_categories)

    def clear_logs(self):
        """清空日志"""
        self.console_widget.clear_logs()


# ---------------------------------------------------------
# 程序入口
# ---------------------------------------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
