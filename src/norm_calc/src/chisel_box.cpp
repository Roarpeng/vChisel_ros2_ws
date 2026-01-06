#include "norm_calc/chisel_box.h"
#include <iostream>
#include <limits>
#include <pcl/surface/convex_hull.h>

namespace chisel_box {

ChiselBox::ChiselBox(int row, int col, ChiselParam param)
    : row_(row), col_(col), param_(param), state_(STATE_PENDING) {}

ChiselBox::~ChiselBox() {}

void ChiselBox::reset() { state_ = STATE_PENDING; }
void ChiselBox::markCompleted() { state_ = STATE_COMPLETED; }

bool ChiselBox::findBestPoint(
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_roi,
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles,
    pcl::PointXYZRGBNormal &out_point) {
  // 如果已经是终态，直接返回
  if (state_ == STATE_COMPLETED || state_ == STATE_UNREACHABLE)
    return false;
  if (cloud_roi->empty())
    return false;

  // === 基于面积的平面优先策略 ===

  // 第一步：计算点云的凸包面积
  float area = calculateConvexHullArea(cloud_roi);

  // 第二步：根据面积确定搜索模式
  SearchMode mode = determineSearchMode(area);

  // 第三步：根据模式选择参数
  float norm_th, hole_dist, curv_th;
  std::string mode_name;
  switch (mode) {
    case MODE_PLANE:
      // 平面模式：严格标准
      norm_th = param_.STRICT_NORM_TH;      // 0.90
      hole_dist = param_.STRICT_HOLE_DIST;  // 0.05
      curv_th = param_.STRICT_CURV_TH;      // 0.07
      mode_name = "PLANE";
      break;
    case MODE_HYBRID:
      // 混合模式：使用中间值
      norm_th = param_.HYBRID_NORM_TH;      // 0.885
      hole_dist = param_.HYBRID_HOLE_DIST;  // 0.0425
      curv_th = param_.HYBRID_CURV_TH;      // 0.085
      mode_name = "HYBRID";
      break;
    case MODE_PROTRUSION:
      // 凹凸面模式：宽松标准
      norm_th = param_.RELAXED_NORM_TH;     // 0.87
      hole_dist = param_.RELAXED_HOLE_DIST; // 0.035
      curv_th = param_.RELAXED_CURV_TH;     // 0.10
      mode_name = "PROTRUSION";
      break;
  }

  std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Mode: " << mode_name
            << " (norm_th: " << norm_th << ", hole_dist: " << hole_dist << ", curv_th: " << curv_th << ")" << std::endl;

  // 第四步：执行搜索
  bool found = searchWithCriteria(cloud_roi, obstacles,
                                 norm_th, hole_dist, curv_th,
                                 out_point);

  // 第五步：更新状态
  if (found) {
    state_ = STATE_COMPLETED;
  } else {
    state_ = STATE_UNREACHABLE;
  }

  return found;
}

bool ChiselBox::searchWithCriteria(
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles, float norm_th,
    float hole_dist_th, float curv_th, pcl::PointXYZRGBNormal &result) {
  if (cloud->empty())
    return false;

  // ==========================================
  // 第一步：拓扑分析 (计算 Z 范围和平均曲率)
  // ==========================================
  float z_min = std::numeric_limits<float>::max();
  float z_max = -std::numeric_limits<float>::max();
  float z_sum = 0.0f;
  float curv_sum = 0.0f;

  for (const auto &pt : cloud->points) {
    if (pt.z < z_min)
      z_min = pt.z;
    if (pt.z > z_max)
      z_max = pt.z;
    z_sum += pt.z;
    curv_sum += pt.curvature;
  }

  float z_range = z_max - z_min;
  float avg_z = z_sum / cloud->size();
  float avg_curv = curv_sum / cloud->size();

  // 改进的凸起检测：主要基于曲率，高度差作为辅助条件
  // 1. 曲率超过阈值（主要判定：曲面/凸起）
  // 2. 或者曲率适中但高度差很大（辅助判定：明显凸起）
  bool is_protrusion = (avg_curv > 0.02) ||  // 平均曲率 > 0.02，判定为曲面
                       (avg_curv > 0.01 && z_range > param_.PROTRUSION_TH);  // 曲率适中但高度差 > 3cm

  std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Z-range: " << z_range * 1000.0f << "mm, Avg-curv: " << avg_curv
            << ", Is-protrusion: " << (is_protrusion ? "YES" : "NO") << std::endl;

  // 定义有效的高度区间 [valid_z_min, valid_z_max]
  float valid_z_min = z_min;
  float valid_z_max = z_max;

  if (is_protrusion) {
    // 【策略核心】：如果是凸起，切掉顶部和底部
    // 顶部容易滑，底部可能太深打不到
    // 目标：半山腰 (Waist) 到 根部 (Base)
    valid_z_max = z_max - (z_range * param_.TIP_CROP_RATIO);  // 切顶
    valid_z_min = z_min + (z_range * param_.BASE_CROP_RATIO); // 切底

    // 调试信息 (可选)
    // std::cout << "Detected Protrusion! Range: " << z_range
    //           << " Target Z: " << valid_z_min << " ~ " << valid_z_max <<
    //           std::endl;
  } else {
    // 如果是平面，优先打微凸的地方，不限制顶部
    // 保持 valid_z_max = z_max
  }

  // ==========================================
  // 第二步：遍历选点
  // ==========================================
  float best_score = -std::numeric_limits<float>::infinity();
  int best_idx = -1;
  float dist_sq_th = hole_dist_th * hole_dist_th;

  // 调试统计
  int filtered_by_height = 0;
  int filtered_by_norm = 0;
  int filtered_by_curv = 0;
  int filtered_by_obstacle = 0;

  // 网格中心 (用于 Center Weight)
  float cx = param_.XMIN + col_ * param_.BOX_LEN + param_.BOX_LEN / 2.0f;
  float cy = param_.YMIN + row_ * param_.BOX_LEN + param_.BOX_LEN / 2.0f;

  for (size_t i = 0; i < cloud->size(); ++i) {
    const auto &pt = cloud->points[i];

    // --- 1. 几何过滤 (Geometry Filter) ---

    // [新增] 高度区间过滤：如果是凸起，直接丢弃顶部和底部的点
    if (pt.z > valid_z_max || pt.z < valid_z_min) {
      filtered_by_height++;
      continue;
    }

    // 法向约束
    if (std::abs(pt.normal_z) < norm_th) {
      filtered_by_norm++;
      continue;
    }

    // 曲率约束
    if (pt.curvature > curv_th) {
      filtered_by_curv++;
      continue;
    }

    // --- 2. 避障过滤 (Obstacle Filter) ---
    bool clash = false;
    if (obstacles && !obstacles->empty()) {
      for (const auto &obs : obstacles->points) {
        float dx = pt.x - obs.x;
        float dy = pt.y - obs.y;
        if (dx * dx + dy * dy < dist_sq_th) {
          clash = true;
          break;
        }
      }
    }
    if (clash) {
      filtered_by_obstacle++;
      continue;
    }

    // --- 3. 智能评分 (Scoring) ---
    float dist_center =
        std::sqrt(std::pow(pt.x - cx, 2) + std::pow(pt.y - cy, 2));

    float score = 0.0f;

    if (is_protrusion) {
      // 【凸起策略评分】
      // 1. 在有效区间内，优先选法向变化小（平整）的地方，防止打在棱上
      score -= param_.CURV_WEIGHT * pt.curvature * 2.0f; // 加倍惩罚曲率

      // 2. 弱化高度权重：既然已经切顶了，剩下的区间里，高度没那么重要了
      // 只要在区间内，主要看是否好下刀
      score += param_.HEIGHT_WEIGHT * pt.z * 0.5f;

      // 3. 极度强调法向垂直度：半山腰下刀，必须保证不滑
      score += param_.ANGLE_WEIGHT * std::abs(pt.normal_z) * 1.5f;
    } else {
      // 【平面策略评分】 (原有逻辑)
      // 优先打稍微凸起一点的地方（好破碎）
      score += param_.HEIGHT_WEIGHT * pt.z;
      score -= param_.CURV_WEIGHT * pt.curvature;
      score += param_.ANGLE_WEIGHT * std::abs(pt.normal_z);
    }

    // 通用：优先打网格中心
    score -= param_.CENTER_WEIGHT * dist_center;

    if (score > best_score) {
      best_score = score;
      best_idx = i;
    }
  }

  if (best_idx >= 0) {
    result = cloud->points[best_idx];
    std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Found point! Total: " << cloud->size()
              << ", Filtered: " << filtered_by_height << "(height) + " << filtered_by_norm << "(norm) + "
              << filtered_by_curv << "(curv) + " << filtered_by_obstacle << "(obstacle) = "
              << (filtered_by_height + filtered_by_norm + filtered_by_curv + filtered_by_obstacle) << std::endl;
    return true;
  }

  std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] No point found! Total: " << cloud->size()
            << ", Filtered: " << filtered_by_height << "(height) + " << filtered_by_norm << "(norm) + "
            << filtered_by_curv << "(curv) + " << filtered_by_obstacle << "(obstacle) = "
            << (filtered_by_height + filtered_by_norm + filtered_by_curv + filtered_by_obstacle) << std::endl;
  return false;
}

// [新增] 计算点云的凸包面积（平方米）
float ChiselBox::calculateConvexHullArea(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
  if (cloud->empty() || cloud->size() < 3) {
    std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Area: 0.0 (points: " << cloud->size() << ")" << std::endl;
    return 0.0f;
  }

  // 投影到XY平面
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_2d->resize(cloud->size());
  for (size_t i = 0; i < cloud->size(); ++i) {
    cloud_2d->points[i].x = cloud->points[i].x;
    cloud_2d->points[i].y = cloud->points[i].y;
    cloud_2d->points[i].z = 0.0f;
  }

  // 计算凸包
  pcl::ConvexHull<pcl::PointXYZ> chull;
  chull.setInputCloud(cloud_2d);
  pcl::PointCloud<pcl::PointXYZ>::Ptr hull_points(new pcl::PointCloud<pcl::PointXYZ>);
  chull.reconstruct(*hull_points);

  // 计算凸包面积（使用鞋带公式）
  if (hull_points->size() < 3) {
    return 0.0f;
  }

  float total_area = 0.0f;
  for (size_t i = 0; i < hull_points->size(); ++i) {
    size_t j = (i + 1) % hull_points->size();
    total_area += hull_points->points[i].x * hull_points->points[j].y;
    total_area -= hull_points->points[j].x * hull_points->points[i].y;
  }

  float area = std::abs(total_area) / 2.0f;
  std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Area: " << area * 10000.0f << " cm² (points: " << cloud->size() << ")" << std::endl;
  return area;
}

// [新增] 根据面积确定搜索模式
ChiselBox::SearchMode ChiselBox::determineSearchMode(float area) {
  const float PLANE_AREA_HIGH = param_.PLANE_AREA_HIGH;  // 0.00035 m² (3.5cm²)
  const float PLANE_AREA_LOW = param_.PLANE_AREA_LOW;   // 0.00025 m² (2.5cm²)

  if (area >= PLANE_AREA_HIGH) {
    return MODE_PLANE;
  } else if (area >= PLANE_AREA_LOW) {
    return MODE_HYBRID;
  } else {
    return MODE_PROTRUSION;
  }
}

} // namespace chisel_box