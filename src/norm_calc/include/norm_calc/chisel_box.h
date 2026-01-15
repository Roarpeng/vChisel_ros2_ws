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

  // [新增] 平面优先策略参数
  float PLANE_AREA_TH;       // 平面面积阈值（1平方cm）
  float PLANE_CURV_TH;       // 平面曲率阈值（< 0.035 认为平面）
  float PLANE_NORM_TH;       // 平面法向阈值（cos(18°)≈0.95）
  float PLANE_HOLE_DIST;     // 平面避障距离（2cm）
  float PLANE_BONUS;         // 平面奖励权重

  // [新增] 凹凸山腰策略参数
  float PROTRUSION_TH;       // 凸起高度差阈值（2cm）
  float TIP_CROP_RATIO;      // 切顶比例基准值（动态调整）
  float BASE_CROP_RATIO;     // 切底比例（固定10%）
  float MOUNTAIN_NORM_TH;    // 山腰法向阈值（cos(25°)≈0.91）
  float MOUNTAIN_HOLE_DIST;  // 山腰避障距离（4cm）

  // [新增] 凹坑避让参数
  float HOLE_SAFE_DIST;      // 凹坑安全距离（5cm）
  bool ENABLE_HOLE_DIR_CHECK; // 启用凹坑方向检查
  float DEPRESSION_DIST;     // 低洼区域检测距离（3cm）
  float MAX_SLOPE_ANGLE;     // 最大允许坡度（30度）
  float SLOPE_CHECK_RADIUS;  // 坡度检查半径（2cm）

  // [新增] 防滑移参数
  float MAX_NORMAL_Y;        // 最大法向Y分量（防止向右滑移）
  float MAX_NORMAL_Z;        // 最大法向Z分量（防止向下滑移）

  // [新增] 评分权重
  float HEIGHT_WEIGHT;        // 高度权重（山腰优先）
  float CURV_WEIGHT;          // 曲率权重（平整度优先）
  float ANGLE_WEIGHT;         // 法向角度权重（垂直度优先）
  float CENTER_WEIGHT;        // 中心权重（降低）

  // [新增] 随机模式参数
  float RANDOM_OFFSET_RANGE;  // 随机位置偏移范围
  float RANDOM_ANGLE_RANGE;   // 随机法向角度范围

  // [新增] 局部平面拟合和高度残差参数
  float RESIDUAL_WEIGHT;      // 高度残差权重（凸起优先）
  float RANSAC_THRESHOLD;     // RANSAC平面拟合阈值（米）
  int MIN_PLANE_POINTS;       // RANSAC平面拟合最小点数

  // [新增] 目标高度和Cell停止参数
  float DELTA_Z;              // 期望削减量（米，每次凿击期望下降的高度）
  float CELL_FLAT_THRESHOLD;  // Cell停止阈值（米，高度差小于此值时标记为完成）

  // [新增] 混合策略参数（平面优先和评分权重优化）
  float FLAT_POINT_BONUS;     // 平面点奖励（平面优先）
  float CURVATURE_THRESHOLD;  // 曲率阈值（用于平面识别）
  float CELL_STD_THRESHOLD;   // Cell停止标准差阈值（米）

  // [新增] 法向趋同约束参数（防止重复凿击）
  float NORMAL_SIMILARITY_THRESHOLD;  // 法向趋同阈值（弧度，法向角度差小于此值时认为趋同）

  // [新增] 位置距离避让参数（防止重复凿击）
  float POSITION_DISTANCE_THRESHOLD;  // 位置距离阈值（米，与上次点位的距离小于此值时认为太近）
} ChiselParam;

// [新增] 局部参考平面结构体
struct LocalPlane {
  Eigen::Vector3f normal;    // 平面法向量
  float d;                    // 平面方程: n·p + d = 0
  float avg_height;           // 平均高度
  float max_residual;         // 最大高度残差
  bool is_valid;              // 平面是否有效
  LocalPlane() : d(0.0f), avg_height(0.0f), max_residual(0.0f), is_valid(false) {}
};

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

  // [新增] 拟合局部参考平面（使用RANSAC）
  bool fitLocalPlane(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, LocalPlane& plane);

  // [新增] 计算点相对于局部平面的高度残差
  float calculateHeightResidual(const pcl::PointXYZRGBNormal& point, const LocalPlane& plane);

  // [新增] 计算点云的中位数高度
  float calculateMedian(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);

  // [新增] 计算目标高度（中位数 - 期望削减量）
  float calculateTargetHeight(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);

  // [新增] 检查Cell是否完成（高度差小于阈值）
  bool isCellComplete(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);

  // [新增] 计算点云的标准差（用于Cell停止条件）
  float calculateStandardDeviation(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);

  // [新增] 检查法向是否趋同（防止重复凿击）
  bool isNormalSimilar(const pcl::PointXYZRGBNormal& current_point,
                      const pcl::PointXYZRGBNormal& last_point);

  // [新增] 检查位置距离是否太近（防止重复凿击）
  bool isPositionTooClose(const pcl::PointXYZRGBNormal& current_point,
                         const pcl::PointXYZRGBNormal& last_point);

  // 内部通用搜索逻辑
  bool searchWithCriteria(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles,
                          float norm_th, float hole_dist_th, float curv_th,
                          pcl::PointXYZRGBNormal &result,
                          bool is_large_plane = false,
                          bool is_plane = false,
                          float z_target = 0.0f);
};

} // namespace chisel_box
#endif