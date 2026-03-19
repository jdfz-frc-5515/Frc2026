import math

def calc_turret_angle(robot_x, robot_y, robot_heading_deg, target_x, target_y, offset_x, offset_y):
    """
    计算炮台相对于机器人前端的瞄准角度（单位：度）
    """
    # 1. 将角度转换为弧度
    heading_rad = math.radians(robot_heading_deg)
    
    # 2. 计算炮台在场地坐标系中的世界位置 (考虑机器人旋转)
    # 旋转矩阵计算偏移
    turret_world_x = robot_x + (offset_x * math.cos(heading_rad) - offset_y * math.sin(heading_rad))
    turret_world_y = robot_y + (offset_x * math.sin(heading_rad) + offset_y * math.cos(heading_rad))
    
    # 3. 计算从炮台到目标的向量
    delta_x = target_x - turret_world_x
    delta_y = target_y - turret_world_y
    
    # 4. 计算目标在世界坐标系下的绝对角度
    angle_to_target_world_rad = math.atan2(delta_y, delta_x)
    angle_to_target_world_deg = math.degrees(angle_to_target_world_rad)
    
    # 5. 计算炮台相对于机器人基座（Heading）的局部角度
    # 结果范围在 [-180, 180]
    relative_angle = angle_to_target_world_deg - robot_heading_deg
    
    # 标准化到 -180 到 180
    while relative_angle > 180: relative_angle -= 360
    while relative_angle < -180: relative_angle += 360
    
    return relative_angle

# ================= 配置参数 =================
# 炮台相对于机器人中心的偏移 (米)
OFFSET_X = 0.127
OFFSET_Y = 0.174

# new AprilTagCoordinate(9,  12.51,  3.67,  1.12,   1,  0,  0, 1.54), // 0°
# new AprilTagCoordinate(10, 12.51,  4.02,  1.12,   1,  0,  0, 1.54), // 0°

# 目标点坐标 (例如场地 Y 轴中心)
TARGET_POS = (12.868148, 4.021328) 

# 假设机器人当前的基准位置 (在 Y 轴平移)
ROBOT_X = 14.85            # 距离目标的 X 轴距离
ROBOT_Y_CENTER = 4.03479     # 刚好在 Y 轴中心
ROBOT_HEADING = -180.0    # 机器人面朝 X 轴负方向（面向目标）

# 模拟偏差 (米)
X_BIAS = 0.05
Y_BIAS = 0.1

X_OFF_B = 0.00
Y_OFF_B = 0
print(f"ROBOT_HEADING: {ROBOT_HEADING}")
print(f"{'机器人Y坐标':>10} | {'理论角度(deg)':>12} | {'偏移5cm后角度':>12} | {'误差值(deg)':>10}")
print("-" * 60)

# 测试从中心到边缘的多个点
for i in range(0, 41, 5):
    y_pos = ROBOT_Y_CENTER - (i / 10.0) # 从中心向一侧平移 0 到 4 米
    
    # 1. 计算理论正确角度
    angle_true = calc_turret_angle(ROBOT_X, y_pos, ROBOT_HEADING, 
                                   TARGET_POS[0], TARGET_POS[1], OFFSET_X, OFFSET_Y)
    
    # 2. 计算存在 5cm 定位偏差时的角度 (假设定位系统认为它在 y_pos + 0.05)
    angle_with_bias = calc_turret_angle(ROBOT_X + X_BIAS, y_pos + Y_BIAS, ROBOT_HEADING, 
                                        TARGET_POS[0], TARGET_POS[1], OFFSET_X+X_OFF_B, OFFSET_Y+Y_OFF_B)
    
    error = (angle_with_bias - angle_true)
    
    print(f"{y_pos:12.2f} | {angle_true:14.3f} | {angle_with_bias:14.3f} | {error:12.3f}")

to_m = 0.0254
field_l, field_w = 651.2 * to_m, 317.7 * to_m

print(f"field_l: {field_l}, {field_l/2}")
print(f"field_w: {field_w}, {field_w/2}")

print(f"492.88*to_m(0.0254)={492.88*to_m}")
print(f"492.33*to_m(0.0254)={492.33*to_m}")