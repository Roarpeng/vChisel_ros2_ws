# Recipe.msg
# 完整配方数据

string name                    # 配方名称
string description             # 配方描述
string version                 # 版本号
string created_at              # 创建时间
string updated_at              # 更新时间

# 区域设置
float64 box_len                # 盒子长度 (mm)
int32 box_row                  # 行数
int32 box_column               # 列数

# 范围限制 (mm)
float64 x_min
float64 x_max
float64 y_min
float64 y_max
float64 z_min
float64 z_max

# 严格模式参数
float64 strict_norm_th         # 法向量阈值
float64 strict_hole_dist       # 孔洞距离
float64 strict_curv_th         # 曲率阈值

# 宽松模式参数
float64 relaxed_norm_th
float64 relaxed_hole_dist
float64 relaxed_curv_th

# 评分权重
float64 height_weight
float64 curv_weight
float64 angle_weight
float64 center_weight

# 凸起处理
float64 protrusion_th          # 凸起阈值
float64 tip_crop_ratio         # 顶部裁剪比例
float64 base_crop_ratio        # 底部裁剪比例
