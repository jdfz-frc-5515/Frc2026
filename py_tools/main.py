import sys
import socket
import datetime
from PyQt5.QtWidgets import (QApplication, QMainWindow, QPlainTextEdit, 
                             QVBoxLayout, QWidget, QLabel)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QFont, QTextCursor

# ---------------------------------------------------------
# 后台线程：专门负责监听来自 RoboRIO 的 UDP 数据
# ---------------------------------------------------------
class UdpListener(QThread):
    # 定义一个信号，当收到消息时发送给主界面
    message_received = pyqtSignal(str)

    def run(self):
        # FRC NetConsole 默认端口是 6666
        PORT = 6666
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
                
                # 关键点：解码。RoboRIO 默认通常是 UTF-8。
                # errors='replace' 防止乱码导致程序崩溃，会用  代替无法识别的字
                decoded_msg = data.decode('utf-8', errors='replace').strip()
                
                if decoded_msg:
                    self.message_received.emit(decoded_msg)
            except Exception as e:
                print(f"接收错误: {e}")

# ---------------------------------------------------------
# 主界面
# ---------------------------------------------------------
class FRCCosnoleWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("FRC Custom NetConsole - Python")
        self.resize(800, 600)
        
        # --- UI 布局设置 ---
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # 顶部标题
        self.status_label = QLabel("正在监听 RoboRIO Output (UDP 6666)...")
        self.status_label.setStyleSheet("color: #888; font-weight: bold;")
        layout.addWidget(self.status_label)

        # 日志显示区域 (使用 QPlainTextEdit 性能更好)
        self.text_area = QPlainTextEdit()
        self.text_area.setReadOnly(True)
        # 设置黑色背景，绿色字体（黑客风格），增大字体方便看
        self.text_area.setStyleSheet("""
            QPlainTextEdit {
                background-color: #1e1e1e;
                color: #00ff00;
                font-family: 'Consolas', 'Microsoft YaHei', monospace;
                font-size: 14px;
                border: 1px solid #333;
            }
        """)
        # 设置最大行数，防止内存无限增长
        self.text_area.setMaximumBlockCount(5000)
        layout.addWidget(self.text_area)

        # --- 启动后台监听线程 ---
        self.listener = UdpListener()
        self.listener.message_received.connect(self.append_log)
        self.listener.start()

    def append_log(self, text):
        """
        接收日志并追加到界面，处理滚动逻辑
        """
        # 1. 生成时间戳
        timestamp = datetime.datetime.now().strftime("[%H:%M:%S.%f]")[:-3] # 毫秒只保留3位
        formatted_text = f"{timestamp} -> {text}"

        # 2. 获取滚动条
        scrollbar = self.text_area.verticalScrollBar()
        
        # 3. 智能滚动逻辑判断
        # 如果当前滚动条距离底部非常近（比如偏差在10像素内），我们认为用户想看最新的
        # 否则，用户可能滚上去看历史了，我们就不自动滚动
        is_at_bottom = (scrollbar.value() >= (scrollbar.maximum() - 10))

        # 4. 追加文本 (PyQt会自动处理 Unicode 中文)
        self.text_area.appendPlainText(formatted_text)

        # 5. 根据之前的判断决定是否滚到底部
        if is_at_bottom:
            scrollbar.setValue(scrollbar.maximum())

# ---------------------------------------------------------
# 程序入口
# ---------------------------------------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = FRCCosnoleWindow()
    window.show()
    sys.exit(app.exec())