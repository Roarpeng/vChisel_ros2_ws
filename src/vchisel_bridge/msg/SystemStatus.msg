# SystemStatus.msg
# 系统状态消息 - 聚合所有子系统状态

# 连接状态
bool plc_connected             # PLC连接状态
bool camera_connected          # 相机连接状态

# 当前配方
string current_recipe          # 当前激活的配方名称

# 运行状态
string IDLE = "IDLE"
string RUNNING = "RUNNING"
string ERROR = "ERROR"
string RECOVERING = "RECOVERING"
string CALIBRATING = "CALIBRATING"

string operation_state         # 当前操作状态

# 统计信息
uint32 points_processed        # 已处理点数
uint32 cycles_completed        # 已完成周期数

# 标定状态
bool calib_mode_active         # 标定模式是否激活
int32 calib_points_captured    # 已采集的标定点数
int32 calib_quality_score      # 标定质量评分

# 错误信息
string last_error              # 最后一次错误信息
string last_error_time         # 最后一次错误时间

# 时间戳
builtin_interfaces/Time stamp
