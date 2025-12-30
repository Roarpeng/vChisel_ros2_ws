#include "norm_calc/chisel_box.h"
#include <iostream>
#include <limits>

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

  bool found = false;

  // === 状态机核心逻辑 ===

  if (state_ == STATE_PENDING) {
    // 【第一次尝试】：使用严格标准 (Strict Mode)
    // 目标：只打最好的位置，宁缺毋滥
    found = searchWithCriteria(cloud_roi, obstacles, param_.STRICT_NORM_TH,
                               param_.STRICT_HOLE_DIST, param_.STRICT_CURV_TH,
                               out_point);
    if (found) {
      state_ = STATE_COMPLETED; // 规划成功
      return true;
    } else {
      state_ = STATE_SKIPPED_ONCE; // 没找到，标记跳过，等待下一轮
      return false;
    }
  } else if (state_ == STATE_SKIPPED_ONCE) {
    // 【第二次尝试】：
    // 1. 先重试严格标准 (万一上次是因为遮挡或噪声没看见)
    found = searchWithCriteria(cloud_roi, obstacles, param_.STRICT_NORM_TH,
                               param_.STRICT_HOLE_DIST, param_.STRICT_CURV_TH,
                               out_point);

    // 2. 如果还不行，降级使用宽松标准 (Relaxed Mode - 次优点)
    if (!found) {
      found = searchWithCriteria(cloud_roi, obstacles, param_.RELAXED_NORM_TH,
                                 param_.RELAXED_HOLE_DIST,
                                 param_.RELAXED_CURV_TH, out_point);
    }

    if (found) {
      state_ = STATE_COMPLETED;
      return true;
    } else {
      state_ = STATE_UNREACHABLE; // 彻底放弃该网格
      return false;
    }
  }

  return false;
}

bool ChiselBox::searchWithCriteria(
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles, float norm_th,
    float hole_dist_th, float curv_th, pcl::PointXYZRGBNormal &result) {
  if (cloud->empty())
    return false;

  // ==========================================
  // 第一步：拓扑分析 (计算 Z 范围)
  // ==========================================
  float z_min = std::numeric_limits<float>::max();
  float z_max = -std::numeric_limits<float>::max();

  for (const auto &pt : cloud->points) {
    if (pt.z < z_min)
      z_min = pt.z;
    if (pt.z > z_max)
      z_max = pt.z;
  }

  float z_range = z_max - z_min;
  bool is_protrusion = (z_range > param_.PROTRUSION_TH); // 是否为凸起

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

  // 网格中心 (用于 Center Weight)
  float cx = param_.XMIN + col_ * param_.BOX_LEN + param_.BOX_LEN / 2.0f;
  float cy = param_.YMIN + row_ * param_.BOX_LEN + param_.BOX_LEN / 2.0f;

  for (size_t i = 0; i < cloud->size(); ++i) {
    const auto &pt = cloud->points[i];

    // --- 1. 几何过滤 (Geometry Filter) ---

    // [新增] 高度区间过滤：如果是凸起，直接丢弃顶部和底部的点
    if (pt.z > valid_z_max || pt.z < valid_z_min)
      continue;

    // 法向约束
    if (std::abs(pt.normal_z) < norm_th)
      continue;

    // 曲率约束
    if (pt.curvature > curv_th)
      continue;

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
    if (clash)
      continue;

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
    return true;
  }
  return false;
}

} // namespace chisel_box