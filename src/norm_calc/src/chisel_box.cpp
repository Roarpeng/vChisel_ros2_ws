#include "norm_calc/chisel_box.h"
#include <iostream>
#include <limits>
#include <pcl/surface/convex_hull.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

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

  // === 平面优先策略 ===
  
  // 【步骤1】计算凸包面积和平均曲率
  float area = calculateConvexHullArea(cloud_roi);
  
  // 计算平均曲率
  float avg_curv = 0.0f;
  for (const auto &pt : cloud_roi->points) {
    avg_curv += pt.curvature;
  }
  avg_curv /= cloud_roi->size();
  
  // 【步骤2】判断是否为平面
  // 平面判定：面积 > 1平方cm 且 曲率 < 0.035
  bool is_plane = (area > param_.PLANE_AREA_TH) && (avg_curv < param_.PLANE_CURV_TH);
  
  std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Area: " << area * 10000.0f 
            << " cm², Avg-curv: " << avg_curv << ", Is-plane: " 
            << (is_plane ? "YES" : "NO") << std::endl;
  
  // 【步骤3】根据表面类型选择策略
  bool found = false;
  
  if (is_plane) {
    // 【平面策略】：垂直凿击
    std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Using PLANE strategy" << std::endl;
    found = searchWithCriteria(cloud_roi, obstacles,
                               param_.PLANE_NORM_TH,      // 法向阈值（非常严格）
                               param_.PLANE_HOLE_DIST,    // 避障距离
                               param_.PLANE_CURV_TH,      // 曲率阈值
                               out_point,
                               true,  // is_large_plane = true
                               true); // is_plane = true
  } else {
    // 【山腰策略】：凹凸不平时的最佳凿击位置
    std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Using MOUNTAIN strategy" << std::endl;
    found = searchWithCriteria(cloud_roi, obstacles,
                               param_.MOUNTAIN_NORM_TH,   // 法向阈值（适中）
                               param_.MOUNTAIN_HOLE_DIST, // 避障距离（更大）
                               0.15,                     // 曲率阈值（宽松）
                               out_point,
                               false, // is_large_plane = false
                               false); // is_plane = false
  }
  
  // 【步骤4】如果平面/山腰策略失败，尝试随机模式
  if (!found) {
    std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Primary strategy failed, trying RANDOM mode" << std::endl;
    found = searchWithRandomMode(cloud_roi, obstacles, out_point);
  }
  
  // 【步骤5】如果随机模式也失败，使用备用方案
  if (!found) {
    std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] RANDOM mode failed, using fallback" << std::endl;
    found = searchWithCriteria(cloud_roi, obstacles,
                               0.5,  // 非常宽松的法向阈值
                               0.01, // 非常宽松的避障距离
                               1.0,  // 非常宽松的曲率阈值
                               out_point,
                               false, // is_large_plane = false
                               false); // is_plane = false
  }
  
  // 【步骤6】更新状态
  if (found) {
    state_ = STATE_COMPLETED;
    last_point_ = out_point;
    has_last_point_ = true;
    std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Found point successfully!" << std::endl;
    return true;
  } else {
    state_ = STATE_UNREACHABLE;
    std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] All strategies failed, marking as UNREACHABLE" << std::endl;
    return false;
  }
  state_ = STATE_UNREACHABLE;
  std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] All attempts failed, marking as UNREACHABLE" << std::endl;
  return false;
}

bool ChiselBox::searchWithCriteria(
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles, float norm_th,
    float hole_dist_th, float curv_th, pcl::PointXYZRGBNormal &result,
    bool is_large_plane, bool is_plane) {
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

  // 【判断表面类型】
  // 如果is_plane为true，强制认为是平面
  // 如果is_plane为false，根据曲率判断是否为平面
  bool is_flat_surface = is_plane || (avg_curv < param_.PLANE_CURV_TH);
  
  // 凸起检测：高度差 ≥ 2cm 且 不是平面
  bool is_protrusion = (z_range >= param_.PROTRUSION_TH) && (!is_flat_surface);

  std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Z-range: " << z_range * 1000.0f << "mm, Avg-curv: " << avg_curv
            << ", Is-flat: " << (is_flat_surface ? "YES" : "NO")
            << ", Is-protrusion: " << (is_protrusion ? "YES" : "NO") << std::endl;

  // ==========================================
  // 第二步：拟合局部参考平面（使用RANSAC）
  // ==========================================
  LocalPlane local_plane;
  bool has_valid_plane = fitLocalPlane(cloud, local_plane);

  // 定义有效的高度区间 [valid_z_min, valid_z_max]
  float valid_z_min = z_min;
  float valid_z_max = z_max;

  if (is_plane) {
    // 【平面策略】：不限制高度区间，优先随机分布
    valid_z_max = z_max;
    valid_z_min = z_min;
  } else if (is_protrusion) {
    // 【山腰策略】：动态切顶切底
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

    // 法向约束（X+方向是向墙里面凿击）
    // 要求法向X分量接近+1（沿着X+方向，向墙里面凿击）
    if (pt.normal_x < norm_th) {
      filtered_by_norm++;
      continue;
    }

    // 【防滑移约束】（Y轴是平移方向，Z轴是上下）
    // 防止法向有向右的分量（normal_y < 0），避免电锤向右滑移到凹区域
    if (is_protrusion && pt.normal_y < -0.1f) {  // 法向向右分量 < -0.1（约6度）
      std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Point filtered by rightward normal: " 
                << pt.normal_y << std::endl;
      filtered_by_norm++;
      continue;
    }

    // 防止法向有向下的分量（normal_z > 0），避免电锤向下滑移
    if (is_protrusion && pt.normal_z > 0.1f) {  // 法向向下分量 > 0.1（约6度）
      std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Point filtered by downward normal: " 
                << pt.normal_z << std::endl;
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

    // --- 2.5 坡度和低洼区域检查（仅山腰位置） ---
    float slope_angle = 0.0f;
    float min_depression_dist = std::numeric_limits<float>::max();
    bool is_unstable = false;
    
    if (is_protrusion) {
      // 【坡度检查】
      // 计算局部坡度：检查周围点的法向量变化
      // X轴是凿击方向，坡度应该是法向偏离X轴的程度
      int neighbor_count = 0;
      float normal_x_sum = 0.0f;
      
      for (const auto &neighbor : cloud->points) {
        float dx = pt.x - neighbor.x;
        float dy = pt.y - neighbor.y;
        float dist_sq = dx * dx + dy * dy;
        
        if (dist_sq < param_.SLOPE_CHECK_RADIUS * param_.SLOPE_CHECK_RADIUS && dist_sq > 0.0001f) {
          normal_x_sum += std::abs(neighbor.normal_x);
          neighbor_count++;
        }
      }
      
      if (neighbor_count > 0) {
        float avg_normal_x = normal_x_sum / neighbor_count;
        // 坡度角度 = acos(平均法向X分量)
        // 坡度越大，法向X分量越小
        slope_angle = std::acos(std::min(1.0f, std::max(-1.0f, avg_normal_x)));
        
        // 如果坡度超过阈值，过滤该点
        if (slope_angle > param_.MAX_SLOPE_ANGLE) {
          std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Point filtered by slope: " 
                    << slope_angle * 180.0f / M_PI << "° (max: " 
                    << param_.MAX_SLOPE_ANGLE * 180.0f / M_PI << "°)" << std::endl;
          filtered_by_obstacle++;
          continue;
        }
      }
      
      // 【低洼区域检测】
      // 检查周围是否有比当前点低很多的位置（低洼区域）
      for (const auto &neighbor : cloud->points) {
        float dx = pt.x - neighbor.x;
        float dy = pt.y - neighbor.y;
        float dist_sq = dx * dx + dy * dy;
        float dist = std::sqrt(dist_sq);
        
        if (dist > 0.001f && dist < param_.DEPRESSION_DIST) {
          // 如果邻居点比当前点低超过2cm，认为是低洼区域
          if (neighbor.z < pt.z - 0.02f) {
            min_depression_dist = std::min(min_depression_dist, dist);
          }
        }
      }
      
      // 如果距离低洼区域太近（< 1.5cm），过滤该点
      if (min_depression_dist < 0.015f) {
        std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Point filtered by depression: " 
                  << min_depression_dist * 1000.0f << "mm" << std::endl;
        filtered_by_obstacle++;
        continue;
      }
      
      // 【滑移方向检查】
      // 检查电锤可能滑移的方向，避免滑向低洼区域或深坑
      if (param_.ENABLE_HOLE_DIR_CHECK && min_depression_dist < std::numeric_limits<float>::max()) {
        // 计算指向低洼区域的向量
        for (const auto &neighbor : cloud->points) {
          if (neighbor.z < pt.z - 0.02f) {
            float dx = neighbor.x - pt.x;
            float dy = neighbor.y - pt.y;
            float dist = std::sqrt(dx * dx + dy * dy);
            
            if (dist > 0.001f && dist < param_.DEPRESSION_DIST) {
              // 归一化指向低洼区域的向量
              float depression_dir_x = dx / dist;
              float depression_dir_y = dy / dist;
              
              // 计算法向量在XY平面的投影
              float normal_xy_len = std::sqrt(pt.normal_x * pt.normal_x + pt.normal_y * pt.normal_y);
              if (normal_xy_len > 0.001f) {
                float normal_xy_x = pt.normal_x / normal_xy_len;
                float normal_xy_y = pt.normal_y / normal_xy_len;
                
                // 计算点积（夹角的余弦）
                float dot_product = normal_xy_x * depression_dir_x + normal_xy_y * depression_dir_y;
                
                // 如果点积 > 0，说明法向量指向低洼区域方向（夹角 < 90度）
                if (dot_product > 0.0f) {
                  std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Point filtered by slide direction" << std::endl;
                  is_unstable = true;
                  break;
                }
              }
            }
          }
        }
      }
      
      if (is_unstable) {
        filtered_by_obstacle++;
        continue;
      }
    }

    // --- 3. 智能评分 (Scoring) ---
    float dist_center =
        std::sqrt(std::pow(pt.x - cx, 2) + std::pow(pt.y - cy, 2));

    // [新增] 计算高度残差（相对于局部参考平面）
    float height_residual = 0.0f;
    if (has_valid_plane) {
      height_residual = calculateHeightResidual(pt, local_plane);
    }

    float score = 0.0f;

    if (is_plane) {
      // 【平面策略评分】：垂直凿击
      // 1. 平面位置奖励
      score += param_.PLANE_BONUS;  // 平面位置额外奖励

      // 2. 强调法向X分量（沿着凿击方向，垂直于墙面）
      score += param_.ANGLE_WEIGHT * pt.normal_x;

      // 3. 平整度奖励（曲率越小越好）
      score -= param_.CURV_WEIGHT * pt.curvature;

      // [新增] 4. 高度残差评分（已削平区域自动降低优先级）
      if (has_valid_plane) {
        // 如果高度残差接近0（已削平），给予轻微惩罚，降低优先级
        if (std::abs(height_residual) < 0.005f) {  // 残差 < 5mm
          score -= param_.RESIDUAL_WEIGHT * 2.0f;  // 已削平区域降低优先级
        } else if (height_residual > 0.01f) {  // 凸起 > 1cm
          score += param_.RESIDUAL_WEIGHT * height_residual * 5.0f;  // 凸起优先
        }
      }

      // 5. 随机分布（不强调中心位置）
      score -= param_.CENTER_WEIGHT * dist_center * 0.1f;

    } else {
      // 【山腰策略评分】：凹凸不平时的最佳凿击位置

      // 1. 在有效区间内，优先选法向变化小（平整）的地方，防止打在棱上
      score -= param_.CURV_WEIGHT * pt.curvature * 2.0f; // 加倍惩罚曲率

      // [新增] 1.5. 高度残差评分（凸起优先）
      if (has_valid_plane) {
        if (height_residual > 0.01f) {  // 凸起 > 1cm
          // 凸起程度越大，分数越高（凸起优先）
          score += param_.RESIDUAL_WEIGHT * height_residual * 10.0f;
        } else if (height_residual < -0.005f) {  // 凹陷 > 0.5cm
          // 凹陷区域给予惩罚
          score -= param_.RESIDUAL_WEIGHT * std::abs(height_residual) * 5.0f;
        }
      }

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
      // X轴是凿击方向，要求法向X分量大（沿着凿击方向）
      float norm_weight = param_.ANGLE_WEIGHT;
      if (min_hole_dist < std::numeric_limits<float>::max()) {
        float hole_dist = std::sqrt(min_hole_dist);
        if (hole_dist < 0.03f) {
          // 距离深坑 < 3cm：极度强调法向沿着凿击方向
          norm_weight *= 2.5f;
        } else if (hole_dist < param_.HOLE_SAFE_DIST) {
          // 距离深坑 3-5cm：适度强调法向沿着凿击方向
          norm_weight *= 2.0f;
        }
        // 距离深坑 > 5cm：使用正常权重
      }
      score += norm_weight * std::abs(pt.normal_x);  // 强调法向X分量（凿击方向）
      
      // 5. 【防滑移评分】：惩罚法向向下分量（Z轴是上下，重力方向）
      // 如果法向有向下的分量（normal_z > 0），给予惩罚
      if (pt.normal_z > 0.0f) {
        float slide_penalty = pt.normal_z * param_.CENTER_WEIGHT * 20.0f;
        score -= slide_penalty;
      }
      
      // 6. 【坡度评分】：优先选择坡度小的位置
      if (slope_angle > 0.0f) {
        // 坡度越小，奖励越大
        float slope_penalty = (slope_angle / param_.MAX_SLOPE_ANGLE) * param_.CURV_WEIGHT * 3.0f;
        score -= slope_penalty;
      }
      
      // 7. 【低洼区域距离评分】：优先选择远离低洼区域的点
      if (min_depression_dist < std::numeric_limits<float>::max()) {
        // 距离越近，惩罚越大
        if (min_depression_dist < param_.DEPRESSION_DIST) {
          float depression_penalty = (param_.DEPRESSION_DIST - min_depression_dist) * param_.CENTER_WEIGHT * 15.0f;
          score -= depression_penalty;
        }
      }
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
        if (dx * dx + dy * dy < param_.PLANE_HOLE_DIST * param_.PLANE_HOLE_DIST) {
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

// [新增] 拟合局部参考平面（使用RANSAC）
bool ChiselBox::fitLocalPlane(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, LocalPlane& plane) {
  if (cloud->empty() || cloud->size() < static_cast<size_t>(param_.MIN_PLANE_POINTS)) {
    plane.is_valid = false;
    std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Local plane fitting failed: not enough points ("
              << cloud->size() << " < " << param_.MIN_PLANE_POINTS << ")" << std::endl;
    return false;
  }

  // 创建点云副本（只保留XYZ）
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_xyz->resize(cloud->size());
  for (size_t i = 0; i < cloud->size(); ++i) {
    cloud_xyz->points[i].x = cloud->points[i].x;
    cloud_xyz->points[i].y = cloud->points[i].y;
    cloud_xyz->points[i].z = cloud->points[i].z;
  }

  // 创建 SAC 分割器
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  // 设置参数
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(param_.RANSAC_THRESHOLD);
  seg.setMaxIterations(1000);

  // 执行分割
  seg.setInputCloud(cloud_xyz);
  seg.segment(*inliers, *coefficients);

  // 检查是否找到平面
  if (inliers->indices.size() < static_cast<size_t>(param_.MIN_PLANE_POINTS)) {
    plane.is_valid = false;
    std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Local plane fitting failed: not enough inliers ("
              << inliers->indices.size() << " < " << param_.MIN_PLANE_POINTS << ")" << std::endl;
    return false;
  }

  // 提取平面参数
  plane.normal[0] = coefficients->values[0];
  plane.normal[1] = coefficients->values[1];
  plane.normal[2] = coefficients->values[2];
  plane.d = coefficients->values[3];

  // 归一化法向量
  float norm = plane.normal.norm();
  if (norm > 0.001f) {
    plane.normal /= norm;
  }

  // 计算平均高度和最大残差
  float z_sum = 0.0f;
  float max_residual = 0.0f;

  for (size_t i = 0; i < cloud->size(); ++i) {
    const auto& pt = cloud->points[i];
    z_sum += pt.z;

    // 计算点到平面的距离（高度残差）
    float residual = std::abs(plane.normal.dot(Eigen::Vector3f(pt.x, pt.y, pt.z)) + plane.d);
    if (residual > max_residual) {
      max_residual = residual;
    }
  }

  plane.avg_height = z_sum / cloud->size();
  plane.max_residual = max_residual;
  plane.is_valid = true;

  std::cout << "[DEBUG] Grid[" << row_ << "," << col_ << "] Local plane fitted: normal=("
            << plane.normal[0] << ", " << plane.normal[1] << ", " << plane.normal[2]
            << "), d=" << plane.d << ", avg_height=" << plane.avg_height
            << ", max_residual=" << max_residual * 1000.0f << "mm" << std::endl;

  return true;
}

// [新增] 计算点相对于局部平面的高度残差
float ChiselBox::calculateHeightResidual(const pcl::PointXYZRGBNormal& point, const LocalPlane& plane) {
  if (!plane.is_valid) {
    return 0.0f;
  }

  // 计算点到平面的有符号距离
  // 公式：distance = n·p + d
  float distance = plane.normal.dot(Eigen::Vector3f(point.x, point.y, point.z)) + plane.d;

  // 如果法向量的Z分量为正，则距离为正表示点在平面上方（凸起）
  // 如果法向量的Z分量为负，则距离为正表示点在平面下方（凸起）
  // 我们希望得到有符号的残差，正值表示凸起，负值表示凹陷
  float signed_residual = distance;

  // 如果法向量指向下方（Z < 0），则反转符号
  if (plane.normal[2] < 0.0f) {
    signed_residual = -signed_residual;
  }

  return signed_residual;
}

} // namespace chisel_box