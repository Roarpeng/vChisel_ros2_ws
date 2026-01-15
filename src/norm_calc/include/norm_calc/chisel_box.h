#ifndef CHISEL_BOX_H
#define CHISEL_BOX_H

#include <cmath>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace chisel_box {

// 状态机定义
enum BoxState {
  STATE_PENDING = 0,     // 初始状态：待处理
  STATE_SKIPPED_ONCE,    // 中间状态：严格模式失败
  STATE_SKIPPED_TWICE,   // 中间状态：宽松模式失败
  STATE_COMPLETED,       // 终态：已规划/已完成
  STATE_UNREACHABLE      // 终态：无法作业（三次都找不到）
};

// 参数聚合
typedef struct {
  float BOX_LEN;
  int BOX_ROW;      // [修复] 新增
  int BOX_COLUMN;   // [修复] 新增
  float XMIN, XMAX; // [修复] 补全 XMAX
  float YMIN, YMAX; // [修复] 补全 YMAX

  float STRICT_NORM_TH;
  float STRICT_HOLE_DIST;
  float STRICT_CURV_TH;

  float RELAXED_NORM_TH;
  float RELAXED_HOLE_DIST;
  float RELAXED_CURV_TH;

  float HEIGHT_WEIGHT;
  float CURV_WEIGHT;
  float ANGLE_WEIGHT;
  float CENTER_WEIGHT;

  // [新增] 凸起策略参数
  float PROTRUSION_TH;   // 判定为凸起的高度差阈值
  float TIP_CROP_RATIO;  // 切顶比例（基准值，会根据高度差动态调整）
  float BASE_CROP_RATIO; // 切底比例

  // [新增] 山腰深坑防滑参数
  float MOUNTAIN_HOLE_DIST;  // 山腰位置的深坑安全距离（3-4cm）
  float MOUNTAIN_NORM_TH;    // 山腰位置的法向角度阈值（25度，cos(25°)≈0.91）
  float HOLE_SAFE_DIST;      // 深坑安全距离阈值（5cm，用于动态调整法向角度）
  bool ENABLE_HOLE_DIR_CHECK; // 是否启用深坑方向检查
  float MAX_SLOPE_ANGLE;     // 最大允许坡度（30度，用于防止滑移）
  float SLOPE_CHECK_RADIUS;  // 坡度检查半径（2cm）
  float DEPRESSION_DIST;     // 低洼区域检测距离（3cm）

  // [新增] 随机模式参数
  float RANDOM_OFFSET_RANGE;  // 随机位置偏移范围
  float RANDOM_ANGLE_RANGE;   // 随机法向角度范围

  // [新增] 平面判定参数
  float FLAT_CURV_TH;  // 曲率阈值：< 0.03 认为平面，>= 0.03 认为凹凸
} ChiselParam;

class ChiselBox {
public:
  ChiselBox(int row, int col, ChiselParam param);
  ~ChiselBox();

  // 重置为 PENDING
  void reset();

  // 强制标记为完成
  void markCompleted();

  // 获取当前状态
  BoxState getState() const { return state_; }
  int getRow() const { return row_; }
  int getCol() const { return col_; }

  // 核心接口：在 ROI 点云中寻找最佳点
  bool findBestPoint(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_roi,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles,
                     pcl::PointXYZRGBNormal &out_point);

private:
  int row_, col_;
  ChiselParam param_;
  BoxState state_;

  // [新增] 上一次点位存储
  pcl::PointXYZRGBNormal last_point_;
  bool has_last_point_;

  // [新增] 计算点云的凸包面积（平方米）
  float calculateConvexHullArea(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);

  // [新增] 随机模式搜索（基于上一次点位或网格中心）
  bool searchWithRandomMode(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles,
                           pcl::PointXYZRGBNormal &result);

  // 内部通用搜索逻辑
  bool searchWithCriteria(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles,
                          float norm_th, float hole_dist_th, float curv_th,
                          pcl::PointXYZRGBNormal &result);
};

} // namespace chisel_box
#endif