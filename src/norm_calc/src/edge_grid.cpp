#include "norm_calc/edge_grid.h"
#include <cstdlib>  // for rand()
#include <cmath>    // for sqrt()

namespace edge_grid
{

EdgeGrid::EdgeGrid(
    size_t inRow,
    size_t inColumn,
    float inTHDEEP,
    float inTHDEEPNORM,
    float inTHHALF,
    float inTHFULL,
    float inTHANGLE
):gridRow(inRow), gridColumn(inColumn), thDeep(inTHDEEP), thDeepNorm(inTHDEEPNORM), thHalf(inTHHALF), thFull(inTHFULL), thAngle(inTHANGLE)
{
    normNum = 0;
    memset(indexList,0 ,MAX_NORM_EACH_GRID*sizeof(size_t));
    memset(&gridNorm,0,sizeof(GridNormPoint));
    memset(&lastValidPoint, 0, sizeof(GridNormPoint));
    minDistanceThreshold = 0.03f;  // Default threshold of 0.03m
    hasLastValidPoint = false;
}

EdgeGrid::EdgeGrid(const EdgeGrid &obj)
{
    gridRow = obj.gridRow;
    gridColumn = obj.gridColumn;
    thDeep = obj.thDeep;
    thDeepNorm = obj.thDeepNorm;
    thHalf = obj.thHalf;
    thFull = obj.thFull;
    thAngle = obj.thAngle;
    normNum = obj.normNum;
    minDistanceThreshold = obj.minDistanceThreshold;
    hasLastValidPoint = obj.hasLastValidPoint;

    // Copy arrays
    memcpy(indexList, obj.indexList, MAX_NORM_EACH_GRID * sizeof(size_t));
    memcpy(&gridNorm, &obj.gridNorm, sizeof(GridNormPoint));
    memcpy(&lastValidPoint, &obj.lastValidPoint, sizeof(GridNormPoint));
}

EdgeGrid::~EdgeGrid(void)
{

}

bool EdgeGrid::reset(void)
{
    normNum = 0;
    memset(indexList, 0, MAX_NORM_EACH_GRID * sizeof(size_t));
    memset(&gridNorm, 0, sizeof(GridNormPoint));
    // Keep lastValidPoint and hasLastValidPoint unchanged to preserve history
    return true;
}

void EdgeGrid::setMinDistanceThreshold(float threshold)
{
    minDistanceThreshold = threshold;
}

// Helper function to calculate distance between two 3D points
float EdgeGrid::calculateDistance(const GridNormPoint& p1, const GridNormPoint& p2)
{
    float dx = p1.ox - p2.ox;
    float dy = p1.oy - p2.oy;
    float dz = p1.oz - p2.oz;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

// Helper function to apply random offset to a point
GridNormPoint EdgeGrid::applyRandomOffset(const GridNormPoint& point)
{
    GridNormPoint offsetPoint = point;

    // Generate small random offsets (±0.01m)
    float offset_range = 0.01f; // 1cm
    offsetPoint.ox += ((float)rand() / RAND_MAX - 0.5f) * 2.0f * offset_range;
    offsetPoint.oy += ((float)rand() / RAND_MAX - 0.5f) * 2.0f * offset_range;
    offsetPoint.oz += ((float)rand() / RAND_MAX - 0.5f) * 2.0f * offset_range;

    return offsetPoint;
}

// 栅格入选点
bool EdgeGrid::addPointInd(size_t index)
{
  if(normNum >= MAX_NORM_EACH_GRID)
  {
    return false;
  }
  indexList[normNum] = index;
  normNum++;

  return true;
}

// 计算栅格位姿
bool EdgeGrid::calcGridPos(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
    if (normNum < thHalf)
    {
        gridNorm.status = EMPTY;
    }
    else
    {
        for (size_t i = 0; i < normNum; i++)
        {
            gridNorm.ox += cloud->points[indexList[i]].x;
            gridNorm.oy += cloud->points[indexList[i]].y;
            gridNorm.oz += cloud->points[indexList[i]].z;

            gridNorm.nx += cloud->points[indexList[i]].normal_x;
            gridNorm.ny += cloud->points[indexList[i]].normal_y;
            gridNorm.nz += cloud->points[indexList[i]].normal_z;
        }

        gridNorm.ox /= normNum;
        gridNorm.oy /= normNum;
        gridNorm.oz /= normNum;
        gridNorm.nx /= normNum;
        gridNorm.ny /= normNum;
        gridNorm.nz /= normNum;

        std::cout << "grid[" << gridRow << "," << gridColumn << "]:("
           << gridNorm.ox << ", "
           << gridNorm.oy << ", "
           << gridNorm.oz << ") ("
           << gridNorm.nx << ", "
           << gridNorm.ny << ", "
           << gridNorm.nz << ") ("
           << normNum << ")" << std::endl;

        // Check distance from last valid point
        if (hasLastValidPoint) {
            float distance = calculateDistance(gridNorm, lastValidPoint);
            std::cout << "Distance from last valid point: " << distance << "m, threshold: " << minDistanceThreshold << "m" << std::endl;

            if (distance < minDistanceThreshold) {
                std::cout << "Current point is too close to the last valid point. Applying random offset." << std::endl;

                // Apply random offset to current point
                GridNormPoint offsetPoint = applyRandomOffset(gridNorm);

                // Check distance again after offset
                float offsetDistance = calculateDistance(offsetPoint, lastValidPoint);
                if (offsetDistance >= minDistanceThreshold) {
                    std::cout << "Random offset successful. New distance: " << offsetDistance << "m" << std::endl;
                    gridNorm = offsetPoint;  // Use the offset point
                } else {
                    std::cout << "Random offset was not sufficient. Distance after offset: " << offsetDistance << "m" << std::endl;
                    gridNorm = offsetPoint;  // Use the offset point anyway
                }
            } else {
                std::cout << "Current point is far enough from the last valid point. Accepting this result." << std::endl;
            }
        } else {
            std::cout << "No previous valid point to compare with. Accepting this result." << std::endl;
        }

        if (gridNorm.oz > thDeep || fabs(gridNorm.nz) < thDeepNorm)
        {
            gridNorm.status = DEEP;
        }
        else
        {
            if (normNum < thFull)
            {
                if (fabs(gridNorm.nz) < thAngle)
                {
                    gridNorm.status = CRACK_SLANT;
                }
                else
                {
                    gridNorm.status = CRACK_FLAT;
                }
            }
            else
            {
                if (fabs(gridNorm.nz) < thAngle)
                {
                    gridNorm.status = SLANT;
                }
                else
                {
                    gridNorm.status = INTACT;
                }
            }
        }

        // Update last valid point if this point is valid
        if (gridNorm.status != EMPTY && gridNorm.status != DEEP) {
            lastValidPoint = gridNorm;
            hasLastValidPoint = true;
        }
    }

    return true;

}

// 返回栅格位姿
bool EdgeGrid::getGridPos(GridNormPoint &retPoint)
{
    memcpy(&retPoint, &gridNorm, sizeof(GridNormPoint));
    return true;
}

}