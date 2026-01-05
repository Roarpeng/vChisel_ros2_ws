# CalibrationStatus.msg
# 标定状态消息

# 标定模式
string IDLE = "IDLE"
string INTRINSIC = "INTRINSIC"
string HANDEYE = "HANDEYE"

string mode                    # 当前标定模式

# 检测状态
bool board_detected            # 标定板是否检测到
float32 coverage_percent       # 标定板覆盖面积百分比
float32 sharpness_score        # 图像清晰度评分
int32 corners_count            # 检测到的角点数

# 采集进度
int32 captured_count           # 已采集数量
int32 target_count             # 目标数量

# 标定结果
bool has_result                # 是否有标定结果
float32 reprojection_error     # 重投影误差
int32 quality_score            # 质量评分 (0-100)
string quality_message         # 质量描述
