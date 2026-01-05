# Alarm.msg
# 报警消息

# 报警级别
uint8 LEVEL_INFO = 0
uint8 LEVEL_WARNING = 1
uint8 LEVEL_ERROR = 2
uint8 LEVEL_CRITICAL = 3

uint8 level                    # 报警级别

# 报警信息
string code                    # 报警代码
string message                 # 报警消息
string source                  # 报警来源（节点名称）

# 时间
builtin_interfaces/Time timestamp

# 是否已确认
bool acknowledged
