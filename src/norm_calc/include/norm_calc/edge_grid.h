#ifndef EDGE_GRID_H
#define EDGE_GRID_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

#define MAX_NORM_EACH_GRID 5000

namespace edge_grid {

typedef enum {
  EMPTY,       // 空
  DEEP,        // 深坑
  CRACK_SLANT, // 碎裂倾斜
  CRACK_FLAT,  // 碎裂平整
  SLANT,       // 倾斜
  INTACT,      // 完整的
} GridStatus;

typedef struct {
  GridStatus status; //
  float ox, oy, oz;  // 凿击点坐标
  float nx, ny, nz;  // 凿击法向
  float curv;        // 凿击点曲率
} GridNormPoint;

typedef struct {
  float GRID_LEN;  // 每个正方形凿击区块边长
  int GRID_ROW;    // 凿击区块行数
  int GRID_COLUMN; // 凿击区块列树
  float XMIN;      // 实际可凿击区域边界位置
  float XMAX;
  float YMIN;
  float YMAX;
  float ZMIN;
  float ZMAX;
  float THDEEP;
  float THDEEPNORM;
  float THHALF;
  float THFULL;
  float THANGLE;
} GridParam;

class EdgeGrid {
public:
  EdgeGrid(size_t inBox, size_t inColumn, float inTHDEEP, float inTHDEEPNORM,
           float inTHHALF, float inTHFULL, float inTHANGLE);
  EdgeGrid(const EdgeGrid &obj);
  ~EdgeGrid();
  bool reset(void);
  void setMinDistanceThreshold(
      float threshold); // Set the minimum distance threshold
  bool addPointInd(size_t index);
  bool calcGridPos(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
  bool getGridPos(GridNormPoint &retPoint);

private:
  float calculateDistance(
      const GridNormPoint &p1,
      const GridNormPoint &p2); // Calculate distance between two points
  GridNormPoint applyRandomOffset(
      const GridNormPoint &point); // Apply random offset to a point

private:
  size_t normNum;
  size_t gridRow;
  size_t gridColumn;
  float thDeep;
  float thDeepNorm;
  float thHalf;
  float thFull;
  float thAngle;
  size_t indexList[MAX_NORM_EACH_GRID];
  GridNormPoint gridNorm;
  // Variables for tracking last valid point and distance threshold
  GridNormPoint lastValidPoint;
  float minDistanceThreshold; // Distance threshold (default 0.03m)
  bool hasLastValidPoint;     // Flag to indicate if we have a previous point
};
} // namespace edge_grid

#endif