#ifndef CHISEL_BOX_H
#define CHISEL_BOX_H

#include <cmath> // Include cmath for abs() function
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

// #define MULTI_FRAME           // 多帧融合开关
#define USING_STATIS // 统计滤波开关
// #define USING_SMOOTH          // 平滑开关
// #define AVOID_RIM             // 避开边缘开关

#define NORM_NUM_MAX 10000 // 每个凿击区块存储法向量上限
// #define BOX_LEN       0.05f    // 每个正方形凿击区块边长
// #define BOX_ROW       4       // 凿击区块行数
// #define BOX_COLUMN    6       // 凿击区块列树
// #define XMIN          -0.15f   // 实际可凿击区域边界位置
// #define XMAX          0.15f
// #define YMIN          -0.1f
// #define YMAX          0.1f
// #define ZMIN          0.2f
// #define ZMAX          0.6f
// #define BORDER_WIDTH  0.02f   //
// 可凿击区域外延伸的边界宽度，用于更好计算可凿击边界法向量 #define NORM_TH 0.9f
// // 可凿击法向量z阈值 acos(0.9) = 25.8degree #define AFFECT_RADIUS 0.0009f //
// 凿击影响区域半径平方，0.03*0.03 #define HEIGHT_WEIGHT 3.0f    //
// 选取凿击法向量投票函数中高度的权重 #define CURV_WEIGHT   2.0f   //
// 选取凿击法向量投票函数中曲率的权重 #define ANGLE_WEIGHT  1.0f   //
// 选取凿击法向量投票函数中jiaodu的权重 #define SEARCH_RADIUS 0.03f   //
// 邻域统计半径 #define SEARCH_NUM_TH 100     // 邻域统计数量阈值

namespace chisel_box {
/*自定义凿击点法向量结构体*/
typedef struct {
  bool status;      // 状态：true = 激活
  float ox, oy, oz; // 凿击点坐标
  float nx, ny, nz; // 凿击法向
  float curv;       // 凿击点曲率
} ChiselNormPoint;

typedef struct {
  float BOX_LEN;  // 每个正方形凿击区块边长
  int BOX_ROW;    // 凿击区块行数
  int BOX_COLUMN; // 凿击区块列树
  float XMIN;     // 实际可凿击区域边界位置
  float XMAX;
  float YMIN;
  float YMAX;
  float ZMIN;
  float ZMAX;
  float
      BORDER_WIDTH; // 可凿击区域外延伸的边界宽度，用于更好计算可凿击边界法向量
  float NORM_TH;    // 可凿击法向量z阈值 acos(0.9) = 25.8degree
  float AFFECT_RADIUS; // 凿击影响区域半径平方，0.03*0.03
  float HEIGHT_WEIGHT; // 选取凿击法向量投票函数中高度的权重
  float CURV_WEIGHT;   // 选取凿击法向量投票函数中曲率的权重
  float ANGLE_WEIGHT;  // 选取凿击法向量投票函数中jiaodu的权重
  float SEARCH_RADIUS; // 邻域统计半径
  int SEARCH_NUM_TH;   // 邻域统计数量阈值
} ChiselParam;

/* 凿击区域类 */
class ChiselBox {
public:
  ChiselBox(size_t boxColumn, float affectRadius, float heightWeight,
            float curvWeight, float angleWeight, float normTH);
  ChiselBox(const ChiselBox &obj);
  ~ChiselBox();
  bool reset(void);
  bool addPointInd(size_t index);
  bool voteTarPoint(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_holes, size_t row,
                    size_t column, ChiselNormPoint *tarPointList);
  bool getTarStatus();
  bool getTarPoint(ChiselNormPoint &tarPoint);
  size_t getPointNum();

private:
  bool isTarExist;
  size_t num;
  size_t indexList[NORM_NUM_MAX];
  ChiselNormPoint bestPoint;
  size_t bestIndex;
  size_t indexRow;
  size_t indexColumn;
  float bestScore;

  size_t inBoxColumn;
  float inAffectRadius;
  float inHeightWeight;
  float inCurvWeight;
  float inAngleWeight;
  float inNormTH;
};

} // namespace chisel_box
#endif /* CHISEL_BOX_H */