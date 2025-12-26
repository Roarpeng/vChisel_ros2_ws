#include "norm_calc/chisel_box.h"
// #include <ros/ros.h> // Remove ROS1 header
#include "rclcpp/rclcpp.hpp" // Add ROS2 header if needed for logging or other functions
#include <cmath> // Include cmath for abs() function
#include <limits>

// Define missing constants that were used in chisel_box.cpp
#define BOX_LEN 0.05f
#define XMIN -0.15f
#define YMIN -0.1f
#define ZMIN 0.4f
#define ZMAX 0.65f

namespace chisel_box
{
ChiselBox::ChiselBox(
  size_t boxColumn,
  float affectRadius,
  float heightWeight,
  float curvWeight,
  float angleWeight,
  float normTH)
{
  inBoxColumn    = boxColumn;
  inAffectRadius = affectRadius;
  inHeightWeight = heightWeight;
  inCurvWeight   = curvWeight;
  inAngleWeight  = angleWeight;
  inNormTH       = normTH;
  reset();
}

ChiselBox::ChiselBox(const ChiselBox &obj)
{
  inBoxColumn    = obj.inBoxColumn;
  inAffectRadius = obj.inAffectRadius;
  inHeightWeight = obj.inHeightWeight;
  inCurvWeight   = obj.inCurvWeight;
  inAngleWeight  = obj.inAngleWeight;
  inNormTH       = obj.inNormTH;
  isTarExist     = obj.isTarExist;
  num            = obj.num;
  indexRow       = obj.indexRow;
  indexColumn    = obj.indexColumn;
  bestScore      = obj.bestScore;
  bestIndex      = obj.bestIndex;
  bestPoint      = obj.bestPoint;
  for (size_t i = 0; i < NORM_NUM_MAX; i++)
  {
    indexList[i] = obj.indexList[i];
  }
}

ChiselBox::~ChiselBox()
{
}

bool ChiselBox::reset(void)
{
  num = 0;
  isTarExist = false;
  bestScore = 0;
  bestIndex = 0;
  for (size_t i = 0; i < NORM_NUM_MAX; i++)
  {
    indexList[i] = 0;
  }
  return true;
}

bool ChiselBox::addPointInd(size_t index)
{
  if (num < NORM_NUM_MAX)
  {
    indexList[num] = index;
    num++;
    return true;
  }
  else
  {
    return false;
  }
}

bool ChiselBox::voteTarPoint(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_holes,size_t row, size_t column, ChiselNormPoint *tarPointList)
{
  indexRow = row;
  indexColumn = column;
  float bX = (indexColumn * BOX_LEN) + XMIN;
  float bY = (indexRow * BOX_LEN) + YMIN;
  float bZ = ZMIN;
  float tX = bX + BOX_LEN;
  float tY = bY + BOX_LEN;
  float tZ = ZMAX;

  for (size_t i = 0; i < num; i++)
  {
    if (cloud->points[indexList[i]].x > bX &&
        cloud->points[indexList[i]].x < tX &&
        cloud->points[indexList[i]].y > bY &&
        cloud->points[indexList[i]].y < tY &&
        cloud->points[indexList[i]].z > bZ &&
        cloud->points[indexList[i]].z < tZ)
    {
      float score = 0;
      score += (cloud->points[indexList[i]].curvature * inCurvWeight);
      score += (abs(cloud->points[indexList[i]].normal_z) * inAngleWeight);
      score += (cloud->points[indexList[i]].z * inHeightWeight);
      if (score > bestScore)
      {
        bestScore = score;
        bestIndex = indexList[i];
        bestPoint.status = true;
        bestPoint.ox = cloud->points[indexList[i]].x;
        bestPoint.oy = cloud->points[indexList[i]].y;
        bestPoint.oz = cloud->points[indexList[i]].z;
        bestPoint.nx = cloud->points[indexList[i]].normal_x;
        bestPoint.ny = cloud->points[indexList[i]].normal_y;
        bestPoint.nz = cloud->points[indexList[i]].normal_z;
        bestPoint.curv = cloud->points[indexList[i]].curvature;
      }
    }
  }
  if (bestScore != 0)
  {
    isTarExist = true;
  }

  // check if bestPoint is close to hole (use squared-distance compare to inAffectRadius)
  float minDist2 = std::numeric_limits<float>::max();
  for (size_t i = 0; i < cloud_holes->points.size(); i++)
  {
    float dx = cloud_holes->points[i].x - bestPoint.ox;
    float dy = cloud_holes->points[i].y - bestPoint.oy;
    float dz = cloud_holes->points[i].z - bestPoint.oz;
    float dist2 = dx*dx + dy*dy + dz*dz;
    if (dist2 < minDist2)
    {
      minDist2 = dist2;
    }
  }

  // Interpret inAffectRadius as radius^2 (consistent with ChiselParam comment). If inAffectRadius <= 0, skip check.
  bool nearHole = false;
  if (inAffectRadius > 0.0f && minDist2 < inAffectRadius)
  {
    nearHole = true;
    isTarExist = false;
  }

  // For human-friendly logging compute actual minDist (sqrt) when available
  float minDist = (minDist2 < std::numeric_limits<float>::max()) ? sqrt(minDist2) : 100.0f;

  // Log detailed info for debugging why a box is accepted or rejected
  std::string reason;
  if (num == 0)
  {
    reason = "EMPTY";
  }
  else if (bestScore == 0)
  {
    reason = "NO_CANDIDATE_SCORE0";
  }
  else if (nearHole)
  {
    reason = "NEAR_HOLE";
  }
  else if (isTarExist)
  {
    reason = "SUCCESS";
  }
  else
  {
    reason = "UNKNOWN_FAIL";
  }

  float threshRad = (inAffectRadius > 0.0f) ? sqrt(inAffectRadius) : -1.0f;
  RCLCPP_INFO(rclcpp::get_logger("norm_calc"), "box[%zu,%zu]: num=%zu bestScore=%.4f minDist=%.4f thresh=%.4f isTar=%d reason=%s",
              indexRow, indexColumn, num, bestScore, minDist, threshRad, isTarExist ? 1 : 0, reason.c_str());

  // Also print to stdout to ensure visibility regardless of rclcpp logger configuration
  std::cout << "DBG_BOX[" << indexRow << "," << indexColumn << "] num=" << num
            << " bestScore=" << bestScore << " minDist=" << minDist
            << " thresh=" << threshRad
            << " isTar=" << (isTarExist ? 1 : 0) << " reason=" << reason << std::endl;

  return true;
}

bool ChiselBox::getTarStatus()
{
  return isTarExist;
}

bool ChiselBox::getTarPoint(ChiselNormPoint &tarPoint)
{
  if (isTarExist)
  {
    tarPoint = bestPoint;
    return true;
  }
  else
  {
    return false;
  }
}

size_t ChiselBox::getPointNum()
{
  return num;
}

} // namespace chisel_box