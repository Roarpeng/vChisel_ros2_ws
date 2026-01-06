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
  STATE_PENDING = 0,  // 初始状态：待处理
  STATE_SKIPPED_ONCE, // 中间状态：上次没找到，跳过一次
  STATE_COMPLETED,    // 终态：已规划/已完成
  STATE_UNREACHABLE   // 终态：无法作业（两次都找不到）
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
  float TIP_CROP_RATIO;  // 切顶比例
  float BASE_CROP_RATIO; // 切底比例

  // [新增] 平面面积阈值（平方米）
  float PLANE_AREA_HIGH;  // 3.5cm² - 平面模式阈值
  float PLANE_AREA_LOW;   // 2.5cm² - 凹凸面模式阈值

  // [新增] 混合模式参数
  float HYBRID_NORM_TH;   // 混合法向阈值
  float HYBRID_HOLE_DIST; // 混合避障距离
  float HYBRID_CURV_TH;   // 混合曲率阈值
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

  // [新增] 计算点云的凸包面积（平方米）
  float calculateConvexHullArea(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);

  // [新增] 根据面积确定搜索模式
  SearchMode determineSearchMode(float area);

  // 内部通用搜索逻辑
  bool searchWithCriteria(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles,
                          float norm_th, float hole_dist_th, float curv_th,
                          pcl::PointXYZRGBNormal &result);
};

} // namespace chisel_box
#endif