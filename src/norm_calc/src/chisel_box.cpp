#include "norm_calc/chisel_box.h"
#include "rclcpp/rclcpp.hpp"
namespace chisel_box
{
ChiselBox::ChiselBox(
  size_t boxColumn,
  float affectRadius,
  float heightWeight,
  float curvWeight,
  float angleWeight,
  float normTH):inBoxColumn(boxColumn), inAffectRadius(affectRadius), inHeightWeight(heightWeight),
                inCurvWeight(curvWeight), inAngleWeight(angleWeight), inNormTH(normTH)
{
  isTarExist  = false;
  num         = 0;
  memset(indexList,0 ,NORM_NUM_MAX*sizeof(size_t));
  memset(&bestPoint,0,sizeof(ChiselNormPoint));
  bestIndex   = 0;
  indexRow    = 0;
  indexColumn = 0;
  bestScore   = 0.0f;
}

ChiselBox::ChiselBox(const ChiselBox &obj)
{

}

ChiselBox::~ChiselBox(void)
{

}

// 清零重置
bool ChiselBox::reset(void)
{
  isTarExist  = false;
  num         = 0;
  memset(indexList,0 ,NORM_NUM_MAX*sizeof(size_t));
  memset(&bestPoint,0,sizeof(ChiselNormPoint));
  bestIndex   = 0;
  indexRow    = 0;
  indexColumn = 0;
  bestScore   = 0.0f;
  return true;
}

// 入选点
bool ChiselBox::addPointInd(size_t index)
{
  if(num >= NORM_NUM_MAX)
  {
    return false;
  }
  indexList[num] = index;
  num++;
  
  return true;
}

// 最佳点投票
bool ChiselBox::voteTarPoint(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_holes, size_t row, size_t column, ChiselNormPoint *tarPointList)
{
  float score = 0.0f;
  float heightScore = 0.0f;
  float curvScore = 0.0f;
  float angleScore;
  float holeScore = 0.0f;
  float grayScore = 0.0f;
  float hX, hY, hD;
  float dist  = 0.0f;
  size_t ind;
  uint8_t grayVal;
  indexRow    = row;
  indexColumn = column;

  size_t thOver = 0;

  if(num > 0)
  {
    for(size_t i = 0; i < num; i++)
    {
      // 角度太大直接放弃
      if (fabs(cloud->points[indexList[i]].normal_z) <= inNormTH)
      {
        thOver++;
        continue;
      }
      // 深度太深直接放弃
      if (cloud->points[indexList[i]].z > 0.56f)
      {
        continue;
      }


      if(indexRow > 0) // 非第一排
      {
        if(indexColumn > 0)
        {
          // 左上方点避重叠
          ind = (indexRow - 1) * inBoxColumn + (indexColumn  - 1);
          if(tarPointList[ind].status)
          {
            dist = pow(cloud->points[indexList[i]].x - tarPointList[ind].ox, 2) + pow(cloud->points[indexList[i]].y - tarPointList[ind].oy, 2);
            if (dist < inAffectRadius)
            {
              continue;
            }
          }
          // 左方点避重叠
          ind = indexRow * inBoxColumn + indexColumn - 1;
          if(tarPointList[ind].status)
          {
            dist = pow(cloud->points[indexList[i]].x - tarPointList[ind].ox, 2) + pow(cloud->points[indexList[i]].y - tarPointList[ind].oy, 2);
            if (dist < inAffectRadius)
            {
              continue;
            }
          }
        }
        // 上方点避重叠
        ind = (indexRow - 1) * inBoxColumn + indexColumn;
        if(tarPointList[ind].status)
        {
          dist = pow(cloud->points[indexList[i]].x - tarPointList[ind].ox, 2) + pow(cloud->points[indexList[i]].y - tarPointList[ind].oy, 2);
          if (dist < inAffectRadius)
          {
            continue;
          }
        }
        // 右上方点避重叠
        if(indexColumn < inBoxColumn - 1)
        {
          ind = (indexRow - 1) * inBoxColumn + (indexColumn  + 1);
          if(tarPointList[ind].status)
          {
            dist = pow(cloud->points[indexList[i]].x - tarPointList[ind].ox, 2) + pow(cloud->points[indexList[i]].y - tarPointList[ind].oy, 2);
            if (dist < inAffectRadius)
            {
              continue;
            }
          }
        }
      }
      else // 第一排
      {
        // 左方点避重叠
        if(indexColumn > 0)
        {
          ind = indexRow * inBoxColumn + indexColumn - 1;
          if(tarPointList[ind].status)
          {
            dist = pow(cloud->points[indexList[i]].x - tarPointList[ind].ox, 2) + pow(cloud->points[indexList[i]].y - tarPointList[ind].oy, 2);
            if (dist < inAffectRadius)
            {
              continue;
            }
          }
        }
      }

      // 空洞影响计算
      holeScore = 0.0f;
      for (size_t holeInd = 0; holeInd < cloud_holes->points.size(); holeInd++)
      {
        hX = fabs(cloud_holes->points[holeInd].x - cloud->points[indexList[i]].x);
        hY = fabs(cloud_holes->points[holeInd].y - cloud->points[indexList[i]].y);
        hD = sqrt(hX*hX + hY*hY);

        float gap = 0.04f;
        if (hD < gap)
        {
           holeScore += gap / (hD + gap / 10.0f) - 10.0f/11.0f;
        }
      }

      if (holeScore > 10.0f)
      {
        holeScore = 10.0f;
      }

      // 灰度影响计算
      // grayVal = (uint8_t)((cloud->points[indexList[i]].r * 19595 +
      //                     cloud->points[indexList[i]].g * 38469 +
      //                     cloud->points[indexList[i]].b * 7472) >> 16);
      grayVal = (uint8_t)cloud->points[indexList[i]].a;
      grayScore = (float)grayVal / 500.0f;

      // 高度影响
      heightScore = inHeightWeight * cloud->points[indexList[i]].z;

      // 曲率影响
      curvScore =  inCurvWeight * fabs(cloud->points[indexList[i]].curvature);

      // 角度影响
      angleScore = inAngleWeight * fabs(cloud->points[indexList[i]].normal_z);
      // 改进的评分计算：确保平面区域也能得到合理分数
      float randomFactor = ((float)rand() / RAND_MAX) * 15.0f; // 增加随机因子到0-15
      float baseScore = 50.0f; // 基础分数，确保即使扣分后仍有正分
      
      score = baseScore
            - heightScore * 0.5f      // 降低高度权重影响
            - curvScore * 0.5f        // 降低曲率权重影响  
            + angleScore * 1.5f       // 增加角度权重
            - 0.1f*holeScore
            + grayScore
            + randomFactor; // 增强随机扰动
      // score = 100.0f 
      //       - heightScore
      //       - curvScore
      //       + angleScore 
      //       - 0.1f*holeScore;

      // if (cloud->points[indexList[i]].x > -0.15f && cloud->points[indexList[i]].x < -0.125f
      // && cloud->points[indexList[i]].y > -0.075f && cloud->points[indexList[i]].y < -0.05f)
      // {
      //   ROS_INFO("xxx,y,z= %f,%f, %f",cloud->points[indexList[i]].x, cloud->points[indexList[i]].y, cloud->points[indexList[i]].z );
      //   ROS_INFO("height, curv, angle, hole, gray, total= %f,%f, %f, %f, %f, %f",heightScore,curvScore,angleScore,holeScore, grayScore, score);
      // }
      
      if (score > bestScore)
      {
        bestScore = score;
        bestIndex = indexList[i];
        // ROS_INFO("x,y,z= %f,%f, %f",cloud->points[indexList[i]].x, cloud->points[indexList[i]].y, cloud->points[indexList[i]].z );
        // ROS_INFO("height, curv, angle, hole, gray, total= %f,%f, %f, %f, %f, %f",heightScore,curvScore,angleScore,holeScore, grayScore, score);
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "THOVER = %d", thOver);

    
    // 修改条件：确保每个格子都能返回结果，即使分数为负
    if (bestIndex > 0 && num > thOver) // 只要有有效点和足够的数量就返回结果
    {
      isTarExist        = true;
      bestPoint.status  = true;
      bestPoint.ox      = cloud->points[bestIndex].x;
      bestPoint.oy      = cloud->points[bestIndex].y;
      bestPoint.oz      = cloud->points[bestIndex].z;
      bestPoint.nx      = cloud->points[bestIndex].normal_x;
      bestPoint.ny      = cloud->points[bestIndex].normal_y;
      bestPoint.nz      = cloud->points[bestIndex].normal_z;
      bestPoint.curv    = cloud->points[bestIndex].curvature;
      
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Box[%zu,%zu] selected point: (%.3f,%.3f,%.3f) score: %.2f", 
                 indexRow, indexColumn, bestPoint.ox, bestPoint.oy, bestPoint.oz, bestScore);
      return true;
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Box[%zu,%zu] no valid point (num=%zu, thOver=%zu)", 
                  indexRow, indexColumn, num, thOver);
      return false;
    }
  }
  else
  {
    return false;
  }
}

// 获取最佳点状态
bool ChiselBox::getTarStatus(void)
{
  if (isTarExist)
  {
    return true;
  }
  else
  {
    return false;
  }
}

// 返回最佳点
bool ChiselBox::getTarPoint(ChiselNormPoint &tarPoint)
{
  if (isTarExist)
  {
    memcpy(&tarPoint,&bestPoint,sizeof(ChiselNormPoint));
    return true;
  }
  else
  {
    memset(&tarPoint,0,sizeof(ChiselNormPoint));
    return false;
  }
}

// 返回区域内入选点数
size_t ChiselBox::getPointNum()
{
  return num;
}

// 检查点云质量
bool ChiselBox::checkQuality()
{
  // 基本质量检查：点数是否足够
  if (num < 100) return false;  // 最少需要100点
  
  // 可以添加更多质量检查：
  // - 点分布均匀性
  // - 法向量一致性
  // - 深度变化合理性
  
  return true;
}

// 是否需要密度增强
bool ChiselBox::needsEnhancement()
{
  return (num < 500);  // 少于500点需要增强
}

}
