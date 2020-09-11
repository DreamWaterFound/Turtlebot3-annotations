# 传感器信息类型
########################################
# CONSTANTS -- 定义一些常量/宏
########################################
# Bumper states (states are combined, when multiple bumpers are pressed)
# 碰撞检测器状态
uint8 BUMPER_FORWARD  = 1
uint8 BUMPER_BACKWARD = 2

# Cliff sensor states (states are combined, when multiple cliff sensors are triggered)
# 跌落传感器状态
uint8 CLIFF = 1

# Sonar sensor states (states are combined, when multiple sonar sensors are triggered)
# 超声波传感器状态
uint8 SONAR = 1

# Illumination sensor (states are combined, when multiple illumination sensors are triggered) 
# 光照传感器状态
uint8 ILLUMINATION = 1

# Button states (states are combined, when multiple buttons are pressed)
# OpenCR 板 板载的两个按钮状态
uint8 BUTTON0 = 1
uint8 BUTTON1 = 2

# Motor errors
# 电机错误信息
uint8 ERROR_LEFT_MOTOR  = 1
uint8 ERROR_RIGHT_MOTOR = 2

# Motor torque
# 电机的力矩控制
uint8 TORQUE_ON  = 1
uint8 TORQUE_OFF = 2

########################################
# Messages -- 真正的消息结构体
########################################
Header  header                                                      # 消息头
uint8   bumper                                                      # 碰撞检测器状态
float32 cliff                                                       # 跌落传感器数值
float32 sonar                                                       # 超声波传感器数值
float32 illumination                                                # 光照传感器数值
uint8   led                                                         # OpenCR led 状态
uint8  button                                                       # OpenCR 板载按钮 状态
bool   torque                                                       # ？ 力矩控制？
int32  left_encoder    # (-2,147,483,648 ~ 2,147,483,647)           # 左轮编码器数值
int32  right_encoder   # (-2,147,483,648 ~ 2,147,483,647)           # 右轮编码器数值
float32  battery                                                    # 电池状态
