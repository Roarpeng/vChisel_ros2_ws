#include "norm_calc/chisel_box.h"
#include <iostream>
#include <limits>
#include <pcl/surface/convex_hull.h>

namespace chisel_box {

ChiselBox::ChiselBox(int row, int col, ChiselParam param)
    : row_(row), col_(col), param_(param), state_(STATE_PENDING), has_last_point_(false) {}

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

  // === 三段式策略：严格模式 → 宽松模式 → 随机模式 ===

  if (state_ == STATE_PENDING) {
    // 【第一次尝试】：严格模式
    std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Attempt 1: STRICT mode" << std::endl;
    found = searchWithCriteria(cloud_roi, obstacles,
                               param_.STRICT_NORM_TH,
                               param_.STRICT_HOLE_DIST,
                               param_.STRICT_CURV_TH,
                               out_point);
    if (found) {
      state_ = STATE_COMPLETED;
      last_point_ = out_point;
      has_last_point_ = true;
      std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] STRICT mode succeeded" << std::endl;
      return true;
    } else {
      state_ = STATE_SKIPPED_ONCE;
      std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] STRICT mode failed, trying RELAXED mode" << std::endl;
    }
  }

  if (state_ == STATE_SKIPPED_ONCE) {
    // 【第二次尝试】：宽松模式
    std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Attempt 2: RELAXED mode" << std::endl;
    found = searchWithCriteria(cloud_roi, obstacles,
                               param_.RELAXED_NORM_TH,
                               param_.RELAXED_HOLE_DIST,
                               param_.RELAXED_CURV_TH,
                               out_point);
    if (found) {
      state_ = STATE_COMPLETED;
      last_point_ = out_point;
      has_last_point_ = true;
      std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] RELAXED mode succeeded" << std::endl;
      return true;
    } else {
      state_ = STATE_SKIPPED_TWICE;
      std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] RELAXED mode failed, trying RANDOM mode" << std::endl;
    }
  }

  if (state_ == STATE_SKIPPED_TWICE) {
    // 【第三次尝试】：随机模式
    std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Attempt 3: RANDOM mode" << std::endl;
    found = searchWithRandomMode(cloud_roi, obstacles, out_point);
    if (found) {
      state_ = STATE_COMPLETED;
      last_point_ = out_point;
      has_last_point_ = true;
      std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] RANDOM mode succeeded" << std::endl;
      return true;
    } else {
      // 随机模式也失败，使用最接近网格中心的点作为备用方案
      std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] RANDOM mode failed, using fallback" << std::endl;
      found = searchWithCriteria(cloud_roi, obstacles,
                                 0.5,  // 非常宽松的法向阈值
                                 0.01, // 非常宽松的避障距离
                                 1.0,  // 非常宽松的曲率阈值
                                 out_point);
      if (found) {
        state_ = STATE_COMPLETED;
        last_point_ = out_point;
        has_last_point_ = true;
        std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Fallback succeeded" << std::endl;
        return true;
      }
    }
  }

  // 三次尝试都失败
  state_ = STATE_UNREACHABLE;
  std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] All attempts failed, marking as UNREACHABLE" << std::endl;
  return false;
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

  // 【自适应平整度检测】
  // 平面判定：曲率 < 0.03 认为平面
  // 凹凸判定：曲率 ≥ 0.03 认为凹凸
  bool is_flat_surface = (avg_curv < param_.FLAT_CURV_TH);
  
  // 凸起检测：高度差 ≥ 2cm 且 曲率 ≥ 0.03
  bool is_protrusion = (z_range >= param_.PROTRUSION_TH) && (!is_flat_surface);

  std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Z-range: " << z_range * 1000.0f << "mm, Avg-curv: " << avg_curv
            << ", Is-flat: " << (is_flat_surface ? "YES" : "NO")
            << ", Is-protrusion: " << (is_protrusion ? "YES" : "NO") << std::endl;

  // 定义有效的高度区间 [valid_z_min, valid_z_max]
  float valid_z_min = z_min;
  float valid_z_max = z_max;

  if (is_protrusion) {
    // 【动态切顶切底策略】
    // 根据高度差动态调整切顶比例：
    // - 高度差 < 2cm：切顶0%（不切顶）
    // - 高度差 2-3cm：切顶5%
    // - 高度差 > 3cm：切顶10%
    float tip_ratio = 0.0f;
    if (z_range < 0.02f) {
      tip_ratio = 0.0f;  // 不切顶
    } else if (z_range < 0.03f) {
      tip_ratio = 0.05f;  // 切顶5%
    } else {
      tip_ratio = 0.10f;  // 切顶10%
    }
    
    // 切底：固定10%
    valid_z_max = z_max - (z_range * tip_ratio);  // 动态切顶
    valid_z_min = z_min + (z_range * param_.BASE_CROP_RATIO); // 切底10%

    std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Dynamic tip crop: " << tip_ratio * 100.0f 
              << "% (z_range=" << z_range * 1000.0f << "mm)" << std::endl;
  } else {
    // 【平整表面策略】
    // 不限制顶部，优先随机分布
    valid_z_max = z_max;
    valid_z_min = z_min;
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
    float min_hole_dist = std::numeric_limits<float>::max();
    bool points_to_hole = false;
    
    if (obstacles && !obstacles->empty()) {
      // 【动态避障距离】
      // 山腰位置使用更大的避障距离，防止滑入深坑
      float dynamic_hole_dist = is_protrusion ? param_.MOUNTAIN_HOLE_DIST : hole_dist_th;
      float dynamic_dist_sq_th = dynamic_hole_dist * dynamic_hole_dist;
      
      for (const auto &obs : obstacles->points) {
        float dx = pt.x - obs.x;
        float dy = pt.y - obs.y;
        float dist_sq = dx * dx + dy * dy;
        
        // 记录最小深坑距离
        if (dist_sq < min_hole_dist) {
          min_hole_dist = dist_sq;
        }
        
        // 距离检查
        if (dist_sq < dynamic_dist_sq_th) {
          clash = true;
          break;
        }
        
        // 【深坑方向检查】（仅在山腰位置且启用时）
        if (is_protrusion && param_.ENABLE_HOLE_DIR_CHECK) {
          // 计算指向深坑的向量
          float hole_dir_x = obs.x - pt.x;
          float hole_dir_y = obs.y - pt.y;
          // 归一化指向深坑的向量
          float hole_dir_len = std::sqrt(hole_dir_x * hole_dir_x + hole_dir_y * hole_dir_y);
          if (hole_dir_len > 0.001f) {
            hole_dir_x /= hole_dir_len;
            hole_dir_y /= hole_dir_len;
            
            // 计算法向量与指向深坑向量的夹角
            // 法向量在XY平面的投影
            float normal_xy_len = std::sqrt(pt.normal_x * pt.normal_x + pt.normal_y * pt.normal_y);
            if (normal_xy_len > 0.001f) {
              float normal_xy_x = pt.normal_x / normal_xy_len;
              float normal_xy_y = pt.normal_y / normal_xy_len;
              
              // 计算点积（夹角的余弦）
              float dot_product = normal_xy_x * hole_dir_x + normal_xy_y * hole_dir_y;
              
              // 如果点积 > 0，说明法向量指向深坑方向（夹角 < 90度）
              if (dot_product > 0.0f) {
                points_to_hole = true;
                clash = true;
                break;
              }
            }
          }
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

      // 2. 【山腰优先】：优先选择高度在区间中间位置（山腰）的点
      // 计算点在有效区间中的归一化位置（0=底部，1=顶部）
      float normalized_height = (pt.z - valid_z_min) / (valid_z_max - valid_z_min);
      // 山腰位置（0.4-0.6）给予额外加分
      if (normalized_height >= 0.4f && normalized_height <= 0.6f) {
        score += param_.HEIGHT_WEIGHT * 2.0f;  // 山腰位置加倍奖励
      } else {
        score += param_.HEIGHT_WEIGHT * pt.z * 0.5f;  // 其他位置弱化高度权重
      }

      // 3. 【深坑距离权重】：优先选择远离深坑的点
      if (min_hole_dist < std::numeric_limits<float>::max()) {
        float hole_dist = std::sqrt(min_hole_dist);
        // 如果距离深坑 > 5cm，给予额外奖励
        if (hole_dist > param_.HOLE_SAFE_DIST) {
          score += param_.CENTER_WEIGHT * 2.0f;  // 远离深坑加倍奖励
        } else {
          // 距离越近，惩罚越大
          score -= param_.CENTER_WEIGHT * (param_.HOLE_SAFE_DIST - hole_dist) * 10.0f;
        }
      }

      // 4. 【动态法向角度限制】：根据距离深坑的距离动态调整法向角度权重
      // 距离深坑 > 5cm：法向角度放宽到35°（cos(35°)≈0.82）
      // 距离深坑 3-5cm：法向角度适中25°（cos(25°)≈0.91）
      // 距离深坑 < 3cm：法向角度严格16°（cos(16°)≈0.90）
      float norm_weight = param_.ANGLE_WEIGHT;
      if (min_hole_dist < std::numeric_limits<float>::max()) {
        float hole_dist = std::sqrt(min_hole_dist);
        if (hole_dist < 0.03f) {
          // 距离深坑 < 3cm：极度强调法向垂直度
          norm_weight *= 2.0f;
        } else if (hole_dist < param_.HOLE_SAFE_DIST) {
          // 距离深坑 3-5cm：适度强调法向垂直度
          norm_weight *= 1.5f;
        }
        // 距离深坑 > 5cm：使用正常权重
      }
      score += norm_weight * std::abs(pt.normal_z);
    } else {
      // 【平面策略评分】 (原有逻辑)
      // 优先打稍微凸起一点的地方（好破碎）
      score += param_.HEIGHT_WEIGHT * pt.z;
      score -= param_.CURV_WEIGHT * pt.curvature;
      score += param_.ANGLE_WEIGHT * std::abs(pt.normal_z);
    }

    // 通用：优先打网格中心（但在全局补充时降低权重）
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

// [新增] 随机模式搜索（基于上一次点位或网格中心）
bool ChiselBox::searchWithRandomMode(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles,
                                     pcl::PointXYZRGBNormal &result) {
  if (cloud->empty()) return false;

  // 确定基准点（上一次点位或网格中心）
  pcl::PointXYZRGBNormal base_point;
  if (has_last_point_) {
    base_point = last_point_;
    std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Random mode: using last point as base" << std::endl;
  } else {
    // 使用网格中心作为基准点
    base_point.x = param_.XMIN + col_ * param_.BOX_LEN + param_.BOX_LEN / 2.0f;
    base_point.y = param_.YMIN + row_ * param_.BOX_LEN + param_.BOX_LEN / 2.0f;
    base_point.z = 0.0f;  // Z坐标会在后面更新
    base_point.normal_z = 1.0f;  // 默认法向垂直向上
    std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Random mode: using grid center as base" << std::endl;
  }

  // 生成随机偏移
  float random_angle = ((float)rand() / RAND_MAX) * 2.0f * M_PI;  // 0-2π
  float random_offset = ((float)rand() / RAND_MAX) * param_.RANDOM_OFFSET_RANGE;  // 0-RANDOM_OFFSET_RANGE

  // 计算目标位置
  float target_x = base_point.x + cos(random_angle) * random_offset;
  float target_y = base_point.y + sin(random_angle) * random_offset;

  // 在点云中寻找最接近目标位置的点
  float min_dist = std::numeric_limits<float>::max();
  int best_idx = -1;

  for (size_t i = 0; i < cloud->size(); ++i) {
    const auto &pt = cloud->points[i];

    // 计算到目标位置的距离
    float dist = std::sqrt(std::pow(pt.x - target_x, 2) + std::pow(pt.y - target_y, 2));

    // 避障检查（简化版：只检查距离）
    bool clash = false;
    if (obstacles && !obstacles->empty()) {
      for (const auto &obs : obstacles->points) {
        float dx = pt.x - obs.x;
        float dy = pt.y - obs.y;
        if (dx * dx + dy * dy < param_.RELAXED_HOLE_DIST * param_.RELAXED_HOLE_DIST) {
          clash = true;
          break;
        }
      }
    }
    if (clash) continue;

    // 选择最接近目标位置的点
    if (dist < min_dist) {
      min_dist = dist;
      best_idx = i;
    }
  }

  if (best_idx >= 0) {
    result = cloud->points[best_idx];

    // 随机旋转法向量
    float norm_angle = ((float)rand() / RAND_MAX - 0.5f) * 2.0f * param_.RANDOM_ANGLE_RANGE;  // -RANDOM_ANGLE_RANGE 到 +RANDOM_ANGLE_RANGE
    float cos_angle = cos(norm_angle);
    float sin_angle = sin(norm_angle);

    // 在XY平面旋转法向量
    float new_normal_x = result.normal_x * cos_angle - result.normal_y * sin_angle;
    float new_normal_y = result.normal_x * sin_angle + result.normal_y * cos_angle;
    result.normal_x = new_normal_x;
    result.normal_y = new_normal_y;

    // 归一化法向量
    float norm = std::sqrt(result.normal_x * result.normal_x +
                           result.normal_y * result.normal_y +
                           result.normal_z * result.normal_z);
    if (norm > 0.001f) {
      result.normal_x /= norm;
      result.normal_y /= norm;
      result.normal_z /= norm;
    }

    std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Random mode found point at ("
              << result.x << ", " << result.y << ", " << result.z << ") with normal ("
              << result.normal_x << ", " << result.normal_y << ", " << result.normal_z << ")" << std::endl;
    return true;
  }

  std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Random mode failed: no point found near target" << std::endl;
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

} // namespace chisel_box