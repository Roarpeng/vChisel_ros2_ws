#include "norm_calc/chisel_box.h"
#include <iostream>
#include <limits>
#include <pcl/surface/convex_hull.h>

namespace chisel_box {

ChiselBox::ChiselBox(int row, int col, ChiselParam param)
    : row_(row), col_(col), param_(param), state_(STATE_PENDING), has_last_point_(false), last_success_mode_(MODE_PROTRUSION) {}

ChiselBox::~ChiselBox() {}

void ChiselBox::reset() { state_ = STATE_PENDING; }
void ChiselBox::markCompleted() { state_ = STATE_COMPLETED; }

bool ChiselBox::findBestPoint(
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_roi,
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr obstacles,
    pcl::PointXYZRGBNormal &out_point) {
  // 如果已经是终态，直接返回
  if (state_ == STATE_COMPLETED || state_ == STATE_UNREACHABLE)
    return false;
  if (cloud_roi->empty())
    return false;

  bool found = false;

  // === 基于面积的智能策略：先计算面积和Z-range，选择合适的模式 ===
  if (state_ == STATE_PENDING) {
    // 计算Z-range
    float z_min = std::numeric_limits<float>::max();
    float z_max = -std::numeric_limits<float>::max();
    for (const auto &pt : cloud_roi->points) {
      if (pt.z < z_min)
        z_min = pt.z;
      if (pt.z > z_max)
        z_max = pt.z;
    }
    float z_range = z_max - z_min;

    // 计算凸包面积
    float area = calculateConvexHullArea(cloud_roi);
    
    // 计算平坦率
    float flat_ratio = calculateFlatRatio(cloud_roi);
    
    // 根据面积、Z-range和平坦率确定搜索模式
    SearchMode mode = determineSearchMode(area, z_range, flat_ratio);

    // 根据模式选择参数
    float norm_th, hole_dist_th, curv_th;
    std::string mode_name;

    switch (mode) {
      case MODE_PLANE:
        norm_th = param_.STRICT_NORM_TH;
        hole_dist_th = param_.STRICT_HOLE_DIST;
        curv_th = param_.STRICT_CURV_TH;
        mode_name = "PLANE (25.8°)";
        break;
      case MODE_HYBRID:
        norm_th = param_.HYBRID_NORM_TH;
        hole_dist_th = param_.HYBRID_HOLE_DIST;
        curv_th = param_.HYBRID_CURV_TH;
        mode_name = "HYBRID (35°)";
        break;
      case MODE_PROTRUSION:
        norm_th = param_.RELAXED_NORM_TH;
        hole_dist_th = param_.RELAXED_HOLE_DIST;
        curv_th = param_.RELAXED_CURV_TH;
        mode_name = "PROTRUSION (45°)";
        break;
    }

    std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Attempt 1: " << mode_name << " (Z-range: " << (z_range * 1000.0f) << "mm)" << std::endl;
    found = searchWithCriteria(cloud_roi, obstacles, norm_th, hole_dist_th, curv_th, out_point);
    if (found) {
      state_ = STATE_COMPLETED;
      last_point_ = out_point;
      has_last_point_ = true;
      last_success_mode_ = mode;  // [新增] 记录成功时使用的模式
      std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] " << mode_name << " succeeded" << std::endl;
      return true;
    } else {
      state_ = STATE_SKIPPED_ONCE;
      std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] " << mode_name << " failed, trying RELAXED mode" << std::endl;
    }
  }

  // === 后备策略：三段式降级 ===

  if (state_ == STATE_SKIPPED_ONCE) {
    // 【第二次尝试】：宽松模式（45°）
    std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Attempt 2: RELAXED mode (45°)" << std::endl;
    found = searchWithCriteria(cloud_roi, obstacles,
                               param_.RELAXED_NORM_TH,
                               param_.RELAXED_HOLE_DIST,
                               param_.RELAXED_CURV_TH,
                               out_point);
    if (found) {
      state_ = STATE_COMPLETED;
      last_point_ = out_point;
      has_last_point_ = true;
      last_success_mode_ = MODE_PROTRUSION;  // [新增] RELAXED模式对应PROTRUSION
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
      last_success_mode_ = MODE_PROTRUSION;  // [新增] RANDOM模式也对应PROTRUSION
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
        last_success_mode_ = MODE_PROTRUSION;  // [新增] 备用方案也对应PROTRUSION
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
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr obstacles, float norm_th,
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

  // [改进] 使用曲率阈值内的面积比例来判断平面/凸起
  // 这种方法比单纯使用平均曲率更准确，因为它考虑了格子内大部分区域的特征
  int flat_point_count = 0;  // 曲率 < PROTRUSION_CURV_TH 的点数
  for (const auto &pt : cloud->points) {
    if (pt.curvature < param_.PROTRUSION_CURV_TH) {
      flat_point_count++;
    }
  }
  float flat_ratio = (float)flat_point_count / cloud->size();  // 平面点比例

  // 判定逻辑（使用可配置参数）：
  // 1. 平面点比例 > PLANE_FLAT_RATIO_HIGH：判定为平面
  // 2. 平面点比例 < PLANE_FLAT_RATIO_LOW：判定为凸起
  // 3. 两者之间：结合高度差辅助判断
  bool is_protrusion;
  if (flat_ratio > param_.PLANE_FLAT_RATIO_HIGH) {
    // 大部分区域是平面
    is_protrusion = false;
  } else if (flat_ratio < param_.PLANE_FLAT_RATIO_LOW) {
    // 大部分区域是凸起
    is_protrusion = true;
  } else {
    // 中间情况：结合高度差判断
    is_protrusion = z_range > param_.PROTRUSION_TH;
  }

  std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Z-range: " << z_range * 1000.0f << "mm, Avg-curv: " << avg_curv
            << ", Flat-ratio: " << (flat_ratio * 100.0f) << "%, Is-protrusion: " << (is_protrusion ? "YES" : "NO") << std::endl;

  // [新增] 根据是否是凸起调整避障距离（使用可配置参数）
  // 平面区域使用更小的避障距离，避免过度过滤
  // 凸起区域使用传入的避障距离（保持原有逻辑）
  float adjusted_hole_dist = is_protrusion ? hole_dist_th : param_.PLANE_HOLE_DIST;

  std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Hole distance: " << adjusted_hole_dist * 1000.0f << "mm"
            << " (" << (is_protrusion ? "protrusion" : "plane") << " mode)" << std::endl;

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
  float dist_sq_th = adjusted_hole_dist * adjusted_hole_dist;  // 使用调整后的避障距离

  // 调试统计
  int filtered_by_height = 0;
  int filtered_by_norm = 0;
  int filtered_by_curv = 0;
  int filtered_by_obstacle = 0;

  // 网格中心 (用于 Center Weight)
  float cx = param_.XMIN + col_ * param_.BOX_LEN + param_.BOX_LEN / 2.0f;
  float cy = param_.YMIN + row_ * param_.BOX_LEN + param_.BOX_LEN / 2.0f;

  // [新增] 收集凹面边缘点（用于平面策略避开凹面边缘）
  // 凹面边缘定义：曲率 > PROTRUSION_CURV_TH 的点
  pcl::PointCloud<pcl::PointXYZ>::Ptr concave_edges(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto &pt : cloud->points) {
    if (pt.curvature > param_.PROTRUSION_CURV_TH) {
      pcl::PointXYZ edge_pt;
      edge_pt.x = pt.x;
      edge_pt.y = pt.y;
      edge_pt.z = pt.z;
      concave_edges->push_back(edge_pt);
    }
  }

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
    // 检查与上一次法向点是否相同
    bool is_same_as_last = false;
    if (has_last_point_) {
      float dx = pt.x - last_point_.x;
      float dy = pt.y - last_point_.y;
      float dz = pt.z - last_point_.z;
      float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
      if (dist < param_.AVOID_LAST_POINT_DIST) {  // 距离<1cm认为是同一个点
        is_same_as_last = true;
      }
    }
    if (is_same_as_last) {
      filtered_by_obstacle++;
      continue;
    }

    // 检查与所有obstacles的距离和方向
    bool too_close_or_same_direction = false;
    if (obstacles && !obstacles->empty()) {
      for (const auto &obs : obstacles->points) {
        float dx = pt.x - obs.x;
        float dy = pt.y - obs.y;
        float dz = pt.z - obs.z;
        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        // 检查距离（使用调整后的避障距离：平面2cm，凸起2.5-3.5cm）
        if (dist < adjusted_hole_dist) {
          too_close_or_same_direction = true;
          break;
        }
        
        // 检查方向是否相反
        if (param_.CHECK_OPPOSITE_DIRECTION) {
          // 计算障碍物法向量的模
          float obs_norm = std::sqrt(obs.normal_x * obs.normal_x + 
                                     obs.normal_y * obs.normal_y + 
                                     obs.normal_z * obs.normal_z);
          
          // 跳过零向量（法向量未定义的点，如孔洞点）
          if (obs_norm < 0.001f) {
            continue;  // 跳过零向量，不进行方向检查
          }
          
          // 计算归一化的点积
          float dot_product = (pt.normal_x * obs.normal_x + 
                              pt.normal_y * obs.normal_y + 
                              pt.normal_z * obs.normal_z) / obs_norm;
          
          // 只有当点积 < 0（方向相反）时才需要避障
          if (dot_product < 0.0f) {
            too_close_or_same_direction = true;
            break;
          }
        }
      }
    }
    if (too_close_or_same_direction) {
      filtered_by_obstacle++;
      continue;
    }

    // --- 3. 智能评分 (Scoring) ---
    float dist_center =
        std::sqrt(std::pow(pt.x - cx, 2) + std::pow(pt.y - cy, 2));

    float score = 0.0f;

    if (is_protrusion) {
      // 【凸起策略评分】 (使用可配置参数)
      // 1. 在有效区间内，优先选法向变化小（平整）的地方，防止打在棱上
      score -= param_.CURV_WEIGHT * pt.curvature * param_.PROTRUSION_CURV_WEIGHT_MULT;

      // 2. 弱化高度权重：既然已经切顶了，剩下的区间里，高度没那么重要了
      // 只要在区间内，主要看是否好下刀
      score += param_.HEIGHT_WEIGHT * pt.z * param_.PROTRUSION_HEIGHT_WEIGHT_MULT;

      // 3. 极度强调法向垂直度：半山腰下刀，必须保证不滑
      // 使用可配置参数 PROTRUSION_ANGLE_WEIGHT_MULT
      score += param_.ANGLE_WEIGHT * std::abs(pt.normal_z) * param_.PROTRUSION_ANGLE_WEIGHT_MULT;

      // 4. 凸起策略优先选择网格中心（避免边缘效应）
      score -= param_.CENTER_WEIGHT * dist_center * param_.PROTRUSION_CENTER_WEIGHT_MULT;
    } else {
      // 【平面策略评分】 (使用可配置参数)
      // 平面区域应该优先选择：
      // 1. 法向最接近Z轴的点（垂直凿击）
      // 2. 曲率最小的点（最平整）
      // 3. 避开凹面边缘（新增）
      // 4. 不再优先选择Z坐标大的点（避免选择凸起位置）

      // 极度强调法向垂直度（平面必须垂直凿击）
      score += param_.ANGLE_WEIGHT * std::abs(pt.normal_z) * param_.PLANE_ANGLE_WEIGHT_MULT;

      // 大幅惩罚曲率（平面必须平整）
      score -= param_.CURV_WEIGHT * pt.curvature * param_.PLANE_CURV_WEIGHT_MULT;

      // 轻微偏向Z坐标中部的点（避免边缘效应）
      float z_mid = (valid_z_min + valid_z_max) / 2.0f;
      float z_dist = std::abs(pt.z - z_mid);
      score -= param_.HEIGHT_WEIGHT * z_dist * 0.1f;

      // 平面策略降低中心权重，让算法更关注法向和曲率，而不是位置
      score -= param_.CENTER_WEIGHT * dist_center * param_.PLANE_CENTER_WEIGHT_MULT;

      // [新增] 平面策略避开凹面边缘
      // 计算到最近凹面边缘点的距离，如果太近则降低分数
      if (!concave_edges->empty()) {
        float min_edge_dist = std::numeric_limits<float>::max();
        for (const auto &edge : concave_edges->points) {
          float dx = pt.x - edge.x;
          float dy = pt.y - edge.y;
          float dist = std::sqrt(dx * dx + dy * dy);
          if (dist < min_edge_dist) {
            min_edge_dist = dist;
          }
        }
        // 如果距离凹面边缘太近，降低分数
        if (min_edge_dist < param_.PLANE_AVOID_CONCAVE_DIST) {
          float penalty = (param_.PLANE_AVOID_CONCAVE_DIST - min_edge_dist) / param_.PLANE_AVOID_CONCAVE_DIST;
          score -= penalty * 100.0f;  // 大幅惩罚靠近凹面边缘的点
        }
      }
    }

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
                                     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr obstacles,
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

// [新增] 计算点云的平坦率（曲率 < PROTRUSION_CURV_TH 的点数比例）
float ChiselBox::calculateFlatRatio(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
  if (cloud->empty()) return 0.0f;
  
  int flat_point_count = 0;
  for (const auto &pt : cloud->points) {
    if (pt.curvature < param_.PROTRUSION_CURV_TH) {
      flat_point_count++;
    }
  }
  return (float)flat_point_count / cloud->size();
}

// [新增] 根据面积、Z-range和平坦率确定搜索模式
ChiselBox::SearchMode ChiselBox::determineSearchMode(float area, float z_range, float flat_ratio) {
  // 优先判断面积和平坦率：如果面积大且平坦，即使Z-range稍大也使用PLANE模式
  if (area >= param_.PLANE_AREA_HIGH && flat_ratio > param_.PLANE_FLAT_RATIO_HIGH) {
    // 面积大且平坦，优先使用PLANE模式（除非Z-range过大）
    if (z_range < param_.Z_RANGE_PROTRUSION_TH) {  // 50mm
      return MODE_PLANE;  // 25.8°
    } else {
      return MODE_HYBRID;  // 35° (Z-range太大，降级)
    }
  }
  
  // 其次判断Z-range：如果Z-range超过阈值，使用更大的角度
  if (z_range > param_.Z_RANGE_PROTRUSION_TH) {
    return MODE_PROTRUSION;  // 45°
  } else if (z_range > param_.Z_RANGE_HYBRID_TH) {
    return MODE_HYBRID;  // 35°
  }
  
  // 按面积判断
  if (area >= param_.PLANE_AREA_HIGH) {
    return MODE_PLANE;
  } else if (area >= param_.PLANE_AREA_LOW) {
    return MODE_HYBRID;
  } else {
    return MODE_PROTRUSION;
  }
}

} // namespace chisel_box