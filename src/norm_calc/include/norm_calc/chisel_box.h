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
  float PROTRUSION_CURV_TH;  // 判定为凸起的曲率阈值（平均曲率超过此值判定为凸起）
  float PROTRUSION_TH;   // 判定为凸起的高度差阈值
  float TIP_CROP_RATIO;  // 切顶比例
  float BASE_CROP_RATIO; // 切底比例

  // [新增] 随机模式参数
  float RANDOM_OFFSET_RANGE;  // 随机位置偏移范围
  float RANDOM_ANGLE_RANGE;   // 随机法向角度范围

  // [新增] 平面面积阈值（平方米）
  float PLANE_AREA_HIGH;  // 15cm² - 平面模式阈值
  float PLANE_AREA_LOW;   // 8cm² - 凹凸面模式阈值

  // [新增] Z-range阈值（米）
  float Z_RANGE_HYBRID_TH;      // 30mm - 超过此值使用混合模式（35°）
  float Z_RANGE_PROTRUSION_TH;  // 50mm - 超过此值使用凸起模式（45°）

  // [新增] 混合模式参数
  float HYBRID_NORM_TH;   // 混合法向阈值
  float HYBRID_HOLE_DIST; // 混合避障距离
  float HYBRID_CURV_TH;   // 混合曲率阈值

// [新增] 平面判定参数（基于曲率阈值内的面积比例）
  float PLANE_FLAT_RATIO_HIGH;  // 平面点比例上限（>此值判定为平面，默认0.7）
  float PLANE_FLAT_RATIO_LOW;   // 平面点比例下限（<此值判定为凸起，默认0.5）
  float PLANE_HOLE_DIST;        // 平面区域避障距离（默认0.015m，即1.5cm）
  float PLANE_ANGLE_WEIGHT_MULT;  // 平面策略法向权重倍数（默认3.0）
  float PLANE_CURV_WEIGHT_MULT;   // 平面策略曲率权重倍数（默认5.0）
  float PLANE_CENTER_WEIGHT_MULT; // 平面策略中心权重倍数（默认0.2）
  float PLANE_AVOID_CONCAVE_DIST;  // 平面区域避开凹面边缘的距离（默认0.015m，即1.5cm）

  // [新增] 凸起策略参数优化
  float PROTRUSION_ANGLE_WEIGHT_MULT;  // 凸起策略法向权重倍数（默认2.0，确保山腰点垂直）
  float PROTRUSION_CURV_WEIGHT_MULT;   // 凸起策略曲率权重倍数（默认2.0，加倍惩罚曲率）
  float PROTRUSION_HEIGHT_WEIGHT_MULT; // 凸起策略高度权重倍数（默认0.5，弱化高度权重）
  float PROTRUSION_CENTER_WEIGHT_MULT; // 凸起策略中心权重倍数（默认1.0，优先选择网格中心）
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

  // [新增] 搜索模式枚举
  enum SearchMode { MODE_PLANE, MODE_HYBRID, MODE_PROTRUSION };

  // [新增] 上一次点位存储
  pcl::PointXYZRGBNormal last_point_;
  bool has_last_point_;

  // [新增] 计算点云的凸包面积（平方米）
  float calculateConvexHullArea(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);

  // [新增] 根据面积和Z-range确定搜索模式
  SearchMode determineSearchMode(float area, float z_range);

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