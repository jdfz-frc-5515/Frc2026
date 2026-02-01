import ntcore
import time
import threading
from typing import Any, Callable
from wpimath.geometry import Pose2d

ROBOT_POS_KEY = "MyPose"
TEST_KEY = 'MyPoseIntArr'

class NT4Manager:
    def __init__(self, identity: str = "__5515PythonClient__", server_ip: str = None, team_number: int = 5515):
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.identity = identity
        
        # 严格定义优先级列表
        self.targets = []
        if server_ip:
            self.targets.append(("Static IP", server_ip))
        if team_number:
            self.targets.append(("Team Number", team_number))
        self.targets.append(("Localhost", "127.0.0.1"))
        
        self.publishers = {}
        self.subscribers = {}
        self._stop_thread = False

        # 启动状态监听器 (用于控制台提示)
        self.inst.addConnectionListener(True, self._connection_event_callback)
        
        # 启动后台优先级轮询线程
        self.thread = threading.Thread(target=self._connection_monitor, daemon=True)
        self.thread.start()

        self._start_()

    def _connection_event_callback(self, event):
        if event.is_(ntcore.EventFlags.kConnected):
            print(f"\n[NT4] >>> 成功建立连接! 远程地址: {event.data.remote_id}")
        elif event.is_(ntcore.EventFlags.kDisconnected):
            print("\n[NT4] >>> 连接已断开，重新启动优先级寻址...")

    def _connection_monitor(self):
        """核心逻辑：优先级轮询状态机"""
        time.sleep(0.5)
        while not self._stop_thread:
            if not self.inst.isConnected():
                # 按照优先级列表尝试每一个地址
                for label, value in self.targets:
                    if self.inst.isConnected():
                        break
                    
                    print(f"[NT4] 尝试寻址 -> {label}: {value} ...", end="\r")
                    
                    self.inst.stopClient()
                    self.inst.startClient4(self.identity)
                    
                    if label == "Team Number":
                        self.inst.setServerTeam(value)
                    else:
                        self.inst.setServer(value)
                    
                    # 给每个优先级留出等待时间（赛场网络建议 2-3 秒）
                    start_wait = time.time()
                    while time.time() - start_wait < 2.5:
                        if self.inst.isConnected():
                            break
                        time.sleep(0.1)
            
            # 如果已连接，心跳检查频率降低
            time.sleep(1.0)

    def publish(self, path: str, value: Any):
        if path not in self.publishers:
            type_str = self._get_type_string(value)
            topic = self.inst.getTopic(path)
            self.publishers[path] = topic.genericPublish(type_str)
        
        # 核心修复点：将原生 Python 类型包装成 ntcore.Value
        nt_value = self._make_nt_value(value)
        self.publishers[path].set(nt_value)

    def _make_nt_value(self, value: Any) -> ntcore.Value:
        """将 Python 类型转换为 NT4 需要的 Value 对象"""
        if isinstance(value, bool):
            return ntcore.Value.makeBoolean(value)
        if isinstance(value, int):
            return ntcore.Value.makeInteger(value)
        if isinstance(value, float):
            return ntcore.Value.makeDouble(value)
        if isinstance(value, str):
            return ntcore.Value.makeString(value)
        
        # 数组处理
        if isinstance(value, (list, tuple)):
            if not value: return ntcore.Value.makeDoubleArray([])
            first = value[0]
            if isinstance(first, float): return ntcore.Value.makeDoubleArray(value)
            if isinstance(first, int): return ntcore.Value.makeIntegerArray(value)
            if isinstance(first, bool): return ntcore.Value.makeBooleanArray(value)
            if isinstance(first, str): return ntcore.Value.makeStringArray(value)
            
        return ntcore.Value.makeString(str(value)) # 保底方案

    def _get_type_string(self, value: Any) -> str:
        if isinstance(value, (list, tuple)):
            if not value: return "double[]"
            first = value[0]
            if isinstance(first, float): return "double[]"
            if isinstance(first, int): return "int[]"
            if isinstance(first, bool): return "boolean[]"
            return "string[]"
        if isinstance(value, float): return "double"
        if isinstance(value, int): return "int"
        if isinstance(value, bool): return "boolean"
        return "string"


    def subscribe_struct(self, path: str, struct_type: Any, callback: Callable = None):
        """
        专门用于订阅 Struct 类型数据，如 Pose2d, SwerveModuleState
        :param path: Topic 名称 (如 "MyPose")
        :param struct_type: wpimath 中的类 (如 Pose2d)
        """
        if path not in self.subscribers:
            topic = self.inst.getStructTopic(path, struct_type)
            # 必须提供一个该类型的实例作为默认值
            sub = topic.subscribe(struct_type())
            self.subscribers[path] = sub
            
            if callback:
                # 注意：Struct 的回调返回的是解析后的对象
                self.inst.addListener(
                    sub, 
                    ntcore.EventFlags.kValueAll, 
                    lambda event: callback(path, event.data.value.value())
                )

    def get(self, path: str, default_value: Any = None) -> Any:
        if path not in self.subscribers:
            return default_value

        raw_val = self.subscribers[path].get()
        
        if raw_val is None:
            return default_value

        # 检查是否为 ntcore.Value (GenericSubscriber)
        if isinstance(raw_val, ntcore.Value):
            data = raw_val.value()
            return data if data is not None else default_value
            
        # 否则就是 Struct 类型直接返回的对象
        return raw_val

    # 建议同时修改回调逻辑，确保回调传出的也是解包后的值
    def subscribe(self, path: str, default_value: Any = None, callback: Callable = None):
        if path not in self.subscribers:
            type_str = self._get_type_string(default_value)
            sub = self.inst.getTopic(path).genericSubscribe(type_str)
            self.subscribers[path] = sub
            
            if callback:
                def wrapped_callback(event):
                    # 从事件中提取解包后的值
                    val_obj = event.data.value
                    callback(path, val_obj.value() if val_obj else None)

                self.inst.addListener(sub, ntcore.EventFlags.kValueAll, wrapped_callback)

    def is_connected(self) -> bool:
        return self.inst.isConnected()

    def close(self):
        self._stop_thread = True
        self.inst.stopClient()

    ############################################# 以下是应用层
    def _start_(self):
        self.subscribe_struct(ROBOT_POS_KEY, Pose2d)
        self.subscribe(TEST_KEY, [0])
    def get_robot_pose(self):
        return self.get(ROBOT_POS_KEY)
    
    def get_test_val(self):
        return self.get(TEST_KEY)
    

default_nt4 = NT4Manager()

def test():
    # 运行主逻辑
    try:
        while True:
            pos = default_nt4.get_robot_pose()
            if pos is not None:
                print(f'pos: {pos}')
            tv = default_nt4.get_test_val()
            if tv is not None:
                print(f'tv: {tv}')
            time.sleep(1)
    except KeyboardInterrupt:
        default_nt4.close()

if __name__ == '__main__':
    test()


