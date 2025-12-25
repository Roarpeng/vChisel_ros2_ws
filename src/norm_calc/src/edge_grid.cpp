#include "norm_calc/edge_grid.h"

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
}

EdgeGrid::EdgeGrid(const EdgeGrid &obj)
{

}

EdgeGrid::~EdgeGrid(void)
{

}

bool EdgeGrid::reset(void)
{

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

    }

    return true;

}

// 返回栅格位姿
bool EdgeGrid::getGridPos(GridNormPoint &retPoint)
{
    memcpy(&retPoint, &gridNorm, sizeof(GridNormPoint));
}

}