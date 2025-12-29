#include "norm_calc/norm_calc.h"
#include "norm_calc/edge_grid.h"

#include <iostream>
using namespace std;

bool normCalc(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_holes,
              pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
              pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downSampled,
              pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered,
              pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_smoothed,
              pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
              pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals,
              pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_shrink,
              pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_tarPoint,
              pcl::PolygonMesh::Ptr triangles,
              chisel_box::ChiselNormPoint *tarPointList,
              chisel_box::ChiselParam chiselParam,
              edge_grid::GridParam gridParam) {

  cout << "input cloud: " << cloud->points.size() << endl;

  // DEBUG: Print actual data range
  if (cloud->points.size() > 0) {
    float minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9, minZ = 1e9,
          maxZ = -1e9;
    for (const auto &p : cloud->points) {
      minX = std::min(minX, p.x);
      maxX = std::max(maxX, p.x);
      minY = std::min(minY, p.y);
      maxY = std::max(maxY, p.y);
      minZ = std::min(minZ, p.z);
      maxZ = std::max(maxZ, p.z);
    }
    cout << "Point cloud actual range:" << endl;
    cout << "  X: [" << minX << ", " << maxX << "] (filter: ["
         << chiselParam.XMIN - chiselParam.BORDER_WIDTH << ", "
         << chiselParam.XMAX + chiselParam.BORDER_WIDTH << "])" << endl;
    cout << "  Y: [" << minY << ", " << maxY << "] (filter: ["
         << chiselParam.YMIN - chiselParam.BORDER_WIDTH << ", "
         << chiselParam.YMAX + chiselParam.BORDER_WIDTH << "])" << endl;
    cout << "  Z: [" << minZ << ", " << maxZ << "] (filter: ["
         << chiselParam.ZMIN << ", " << chiselParam.ZMAX << "])" << endl;
  }

  // ********直通滤波start********
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud);    // 设置输入点云
  pass.setFilterFieldName("z"); // 设置过滤时所需要点云类型的Z字段
  pass.setFilterLimits(chiselParam.ZMIN,
                       chiselParam.ZMAX); // 设置在过滤字段的范围
  // pass.setFilterLimitsNegative (true);   //设置保留范围内还是过滤掉范围内
  pass.filter(*cloud_filtered);

  pass.setInputCloud(cloud_filtered); // 设置输入点云
  pass.setFilterFieldName("y"); // 设置过滤时所需要点云类型的y字段
  pass.setFilterLimits(chiselParam.YMIN - chiselParam.BORDER_WIDTH,
                       chiselParam.YMAX +
                           chiselParam.BORDER_WIDTH); // 设置在过滤字段的范围
  // pass.setFilterLimitsNegative (true);   //设置保留范围内还是过滤掉范围内
  pass.filter(*cloud_filtered);

  pass.setInputCloud(cloud_filtered); // 设置输入点云
  pass.setFilterFieldName("x"); // 设置过滤时所需要点云类型的x字段
  pass.setFilterLimits(chiselParam.XMIN - chiselParam.BORDER_WIDTH,
                       chiselParam.XMAX +
                           chiselParam.BORDER_WIDTH); // 设置在过滤字段的范围
  // pass.setFilterLimitsNegative (true);   //设置保留范围内还是过滤掉范围内
  pass.filter(*cloud_filtered);
  cout << "pass cloud: " << cloud_filtered->points.size() << endl;
  // ********直通滤波end********

  // ********下采样start********
  pcl::VoxelGrid<pcl::PointXYZRGB> downSampled; // 创建滤波对象
  downSampled.setInputCloud(cloud_filtered); // 设置需要过滤的点云给滤波对象
  downSampled.setLeafSize(0.0025f, 0.0025f,
                          0.0025f); // 设置滤波时创建的体素体积为5mm的立方体
  downSampled.filter(*cloud_downSampled); // 执行滤波处理，存储输出
  cout << "downSampled cloud: " << cloud_downSampled->points.size() << endl;
// ********下采样end********

// ********统计滤波start********
#ifdef USING_STATIS
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud_downSampled);
  sor.setMeanK(50);
  sor.setStddevMulThresh(3.0);
  sor.filter(*cloud_downSampled);
  cout << "statis cloud: " << cloud_downSampled->points.size() << endl;
#endif
// ********统计滤波end********

// ********点云平滑start********
#ifdef USING_SMOOTH
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treeSampling(
      new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB> mls_point;
  pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;
  treeSampling->setInputCloud(cloud_downSampled);
  mls.setComputeNormals(false);
  mls.setInputCloud(cloud_downSampled);
  mls.setPolynomialOrder(3);
  mls.setPolynomialFit(false);
  mls.setSearchMethod(treeSampling);
  mls.setSearchRadius(0.02);
  mls.process(mls_point);
  cloud_smoothed = mls_point.makeShared();
  cout << "smoothed cloud: " << cloud_smoothed->points.size() << endl;
#endif
  // ********点云平滑end********

  // ********法线估计start********
  // Create an empty kdtree representation, and pass it to the normal estimation
  // object. Its content will be filled inside the object, based on the given
  // input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treeNorm(
      new pcl::search::KdTree<pcl::PointXYZRGB>);
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
#ifdef USING_SMOOTH
  treeNorm->setInputCloud(cloud_smoothed);
  ne.setInputCloud(cloud_smoothed);
#else
  treeNorm->setInputCloud(cloud_downSampled);
  ne.setInputCloud(cloud_downSampled);
#endif
  ne.setSearchMethod(treeNorm);
  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch(0.01);
  // Use K search
  // ne.setKSearch(20);
  // Compute the features
  ne.compute(*cloud_normals);
  cout << "normals cloud: " << cloud_normals->size() << endl;
// ********法线估计end********

//* cloud_with_normals = cloud_smoothed + cloud_normals
#ifdef USING_SMOOTH
  pcl::concatenateFields(*cloud_smoothed, *cloud_normals,
                         *cloud_with_normals); // 连接字段
#else
  pcl::concatenateFields(*cloud_downSampled, *cloud_normals,
                         *cloud_with_normals); // 连接字段
#endif

// ********三角化start********
// //定义搜索树对象
// pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr treeTriangle (new
// pcl::search::KdTree<pcl::PointXYZRGBNormal>);
// // Initialize objects
// pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
// //定义三角化对象
// // pcl::PolygonMesh triangles; //存储最终三角化的网络模型
// treeTriangle->setInputCloud(cloud_with_normals);
// // Set the maximum distance between connected points (maximum edge length)
// gp3.setSearchRadius (0.15);
// //设置连接点之间的最大距离，（即是三角形最大边长）
// // 设置各参数值
// gp3.setMu (2.5);
// //设置被样本点搜索其近邻点的最远距离为2.5，为了使用点云密度的变化
// gp3.setMaximumNearestNeighbors (100);       //设置样本点可搜索的邻域个数
// gp3.setMaximumSurfaceAngle(M_PI/4);
// //设置某点法线方向偏离样本点法线的最大角度45 gp3.setMinimumAngle(M_PI/18);
// //设置三角化后得到的三角形内角的最小的角度为10 gp3.setMaximumAngle(2*M_PI/3);
// //设置三角化后得到的三角形内角的最大角度为120
// gp3.setNormalConsistency(false);            //设置该参数保证法线朝向一致
// // Get result
// gp3.setInputCloud (cloud_with_normals);     //设置输入点云为有向点云
// gp3.setSearchMethod (treeTriangle);         //设置搜索方式
// gp3.reconstruct (*triangles);                //重建提取三角化
// // 附加顶点信息
// std::vector<int> parts = gp3.getPartIDs();
// std::vector<int> states = gp3.getPointStates();
// cout << "triangles cloud: " << triangles->polygons.size() << endl;
// ********三角化end********

// ********边缘点剔除start********
#ifdef AVOID_RIM // 根据邻域点数量确定边缘点；边缘点不纳入投票
  pcl::RadiusOutlierRemoval<pcl::PointXYZRGBNormal> ror;
  ror.setInputCloud(cloud_with_normals);
  ror.setRadiusSearch(chiselParam.SEARCH_RADIUS);
  ror.setMinNeighborsInRadius(chiselParam.SEARCH_NUM_TH);
  ror.filter(*cloud_shrink);
  cout << "shrink cloud: " << cloud_shrink->size() << endl;
#else
  cloud_shrink = cloud_with_normals;
#endif
  // ********边缘点剔除end********

  // holes process
  pcl::PassThrough<pcl::PointXYZ> holePass;
  holePass.setInputCloud(cloud_holes); // 设置输入点云
  holePass.setFilterFieldName("z"); // 设置过滤时所需要点云类型的Z字段
  holePass.setFilterLimits(chiselParam.ZMIN,
                           chiselParam.ZMAX); // 设置在过滤字段的范围
  holePass.filter(*cloud_holes);

  holePass.setInputCloud(cloud_holes); // 设置输入点云
  holePass.setFilterFieldName("y"); // 设置过滤时所需要点云类型的y字段
  holePass.setFilterLimits(
      chiselParam.YMIN - chiselParam.BORDER_WIDTH,
      chiselParam.YMAX + chiselParam.BORDER_WIDTH); // 设置在过滤字段的范围
  holePass.filter(*cloud_holes);

  holePass.setInputCloud(cloud_holes); // 设置输入点云
  holePass.setFilterFieldName("x"); // 设置过滤时所需要点云类型的x字段
  holePass.setFilterLimits(
      chiselParam.XMIN - chiselParam.BORDER_WIDTH,
      chiselParam.XMAX + chiselParam.BORDER_WIDTH); // 设置在过滤字段的范围
  holePass.filter(*cloud_holes);

#ifdef EDGE_CHISEL_MODE
  // ********边缘凿击点筛选start********

  edge_grid::EdgeGrid *edgeTable[EDGE_ROW][EDGE_COL];
  edge_grid::GridNormPoint gridNormTable[EDGE_ROW][EDGE_COL];

  for (size_t i = 0; i < EDGE_ROW; i++) {
    for (size_t j = 0; j < EDGE_COL; j++) {
      edgeTable[i][j] = new edge_grid::EdgeGrid(
          i, j, gridParam.THDEEP, gridParam.THDEEPNORM, gridParam.THHALF,
          gridParam.THFULL, gridParam.THANGLE);
    }
  }
  int indRow, indColumn;
  for (size_t i = 0; i < cloud_shrink->points.size();
       i++) // 将符合条件的法向点归纳至20*10栅格区域类
  {
    indRow = floor((cloud_shrink->points[i].y - gridParam.YMIN) /
                   gridParam.GRID_LEN);
    indColumn = floor((cloud_shrink->points[i].x - gridParam.XMIN) /
                      gridParam.GRID_LEN);
    if (0 <= indRow && indRow < gridParam.GRID_ROW && 0 <= indColumn &&
        indColumn < gridParam.GRID_COLUMN) {
      if (!edgeTable[indRow][indColumn]->addPointInd(i)) {
        static int err_count = 0;
        if (err_count++ % 100 == 0) {
          cout << "add point err (grid full)! " << i << "," << indRow << ","
               << indColumn << endl;
        }
        continue; // Don't return false, just skip this point
      }
    }
  }

  // 栅格特征计算
  for (size_t ii = 0; ii < EDGE_ROW; ii++) {
    for (size_t jj = 0; jj < EDGE_COL; jj++) {
      edgeTable[ii][jj]->calcGridPos(cloud_shrink);
      edgeTable[ii][jj]->getGridPos(gridNormTable[ii][jj]);
    }
  }

  size_t notEmpty = 0;
  size_t tarIndex = 0;
  bool nextLine = false;
  // 栅格凿击点评选
  for (size_t ii = 1; ii < EDGE_ROW - 1; ii++) {
    notEmpty = 0;
    nextLine = false;
    for (size_t jj = 1; jj < EDGE_COL; jj++) {
      cout << ii << "," << jj << "," << gridNormTable[ii][jj].status << endl;
      if (gridNormTable[ii][jj].status == edge_grid::INTACT ||
          gridNormTable[ii][jj].status == edge_grid::SLANT) {
        tarPointList[tarIndex].status = true;
        tarPointList[tarIndex].ox = gridNormTable[ii][jj].ox;
        tarPointList[tarIndex].oy = gridNormTable[ii][jj].oy;
        tarPointList[tarIndex].oz = gridNormTable[ii][jj].oz;
        tarPointList[tarIndex].nx = gridNormTable[ii][jj].nx;
        tarPointList[tarIndex].ny = gridNormTable[ii][jj].ny;
        tarPointList[tarIndex].nz = gridNormTable[ii][jj].nz;
        tarPointList[tarIndex].curv = gridNormTable[ii][jj].curv;
        nextLine = true;
        switch (gridNormTable[ii][jj - 1].status) {

        case edge_grid::EMPTY:
        case edge_grid::DEEP:
          if (gridNormTable[ii][jj].status == edge_grid::SLANT) {
            tarPointList[tarIndex].ox =
                gridNormTable[ii][jj].ox + 0.15f * gridParam.GRID_LEN;
          } else {
            tarPointList[tarIndex].ox =
                gridNormTable[ii][jj].ox; // - 0.15f * gridParam.GRID_LEN;
          }
          break;
        case edge_grid::CRACK_SLANT:
        case edge_grid::CRACK_FLAT:
          tarPointList[tarIndex].ox =
              gridNormTable[ii][jj].ox - 0.05f * gridParam.GRID_LEN;
          break;
        default:
          break;
        }
      }

      if (nextLine) {
        if ((gridNormTable[ii - 1][jj].status == edge_grid::INTACT ||
             gridNormTable[ii - 1][jj].status == edge_grid::SLANT) &&
            (gridNormTable[ii + 1][jj].status == edge_grid::EMPTY ||
             gridNormTable[ii + 1][jj].status == edge_grid::DEEP)) {
          tarPointList[tarIndex].oy -= 0.25f * gridParam.GRID_LEN;
        }
        if ((gridNormTable[ii + 1][jj].status == edge_grid::INTACT ||
             gridNormTable[ii + 1][jj].status == edge_grid::SLANT) &&
            (gridNormTable[ii - 1][jj].status == edge_grid::EMPTY ||
             gridNormTable[ii - 1][jj].status == edge_grid::DEEP)) {
          tarPointList[tarIndex].oy += 0.25f * gridParam.GRID_LEN;
        }

        tarIndex++;
        break;
      }
    }
  }

  // 最小间隔保证
  // for(size_t i = 1; i < tarIndex; i++)
  // {
  //   if (tarPointList[i-1].status == true && tarPointList[i].status == true)
  //   {
  //     if((pow(tarPointList[i].ox - tarPointList[i-1].ox, 2) +
  //       pow(tarPointList[i].oy - tarPointList[i-1].oy, 2))
  //       < 0.25f)
  //     {
  //       tarPointList[i].status = false;
  //     }
  //   }
  // }

  cloud_tarPoint->width = gridParam.GRID_ROW;
  cloud_tarPoint->height = 1;
  cloud_tarPoint->is_dense = false;
  cloud_tarPoint->points.resize(cloud_tarPoint->width * cloud_tarPoint->height);
  for (size_t i = 0; i < cloud_tarPoint->width; i++) // 输出结果
  {
    if (tarPointList[i].status) {
      cloud_tarPoint->points[i].x = tarPointList[i].ox;
      cloud_tarPoint->points[i].y = tarPointList[i].oy;
      cloud_tarPoint->points[i].z = tarPointList[i].oz;
      cloud_tarPoint->points[i].normal_x = tarPointList[i].nx;
      cloud_tarPoint->points[i].normal_y = tarPointList[i].ny;
      cloud_tarPoint->points[i].normal_z = tarPointList[i].nz;
      cloud_tarPoint->points[i].curvature = tarPointList[i].curv;
      cout << "tar[" << i << "]: [" << tarPointList[i].ox << ", "
           << tarPointList[i].oy << ", " << tarPointList[i].oz << "] ["
           << tarPointList[i].nx << ", " << tarPointList[i].ny << ", "
           << tarPointList[i].nz << ", " << tarPointList[i].curv << "]" << endl;
    }
  }

  for (size_t ii = 0; ii < EDGE_ROW; ii++) {
    for (size_t jj = 0; jj < EDGE_COL; jj++) {
      delete edgeTable[ii][jj];
    }
  }

// ********边缘凿击点筛选end********
#else

  // ********凿击点筛选start********
  chisel_box::ChiselBox *chiselTable[4][6];
  for (size_t i = 0; i < 4; i++) {
    for (size_t j = 0; j < 6; j++) {
      chiselTable[i][j] = new chisel_box::ChiselBox(
          chiselParam.BOX_COLUMN, chiselParam.AFFECT_RADIUS,
          chiselParam.HEIGHT_WEIGHT, // 选取凿击法向量投票函数中高度的权重
          chiselParam.CURV_WEIGHT,   // 选取凿击法向量投票函数中曲率的权重
          chiselParam.ANGLE_WEIGHT, chiselParam.NORM_TH);
    }
  }
  float fRow, fCol;
  int indRow, indColumn;
  for (size_t i = 0; i < cloud_shrink->points.size();
       i++) // 将符合条件的法向点归纳至4*6凿击区域类
  {
    fRow = (cloud_shrink->points[i].y - chiselParam.YMIN) / chiselParam.BOX_LEN;
    fCol = (cloud_shrink->points[i].x - chiselParam.XMIN) / chiselParam.BOX_LEN;

    if (-0.3f <= fRow && fRow <= 0.0f) {
      indRow = 0;
    } else if (chiselParam.BOX_ROW <= fRow &&
               fRow <= 0.3f + chiselParam.BOX_ROW) {
      indRow = chiselParam.BOX_ROW - 1;
    } else {
      indRow = floor(fRow);
    }

    if (-0.3f <= fCol && fCol <= 0.0f) {
      indColumn = 0;
    } else if (chiselParam.BOX_COLUMN <= fCol &&
               fCol <= 0.3f + chiselParam.BOX_COLUMN) {
      indColumn = chiselParam.BOX_COLUMN - 1;
    } else {
      indColumn = floor(fCol);
    }

    if (0 <= indRow && indRow < chiselParam.BOX_ROW && 0 <= indColumn &&
        indColumn < chiselParam.BOX_COLUMN) {
      if (!chiselTable[indRow][indColumn]->addPointInd(i)) {
        static int err_count = 0;
        if (err_count++ % 100 == 0) {
          cout << "add point err (box full)! " << i << "," << indRow << ","
               << indColumn << endl;
        }
        continue; // Don't return false, just skip this point
      }
    }
  }
  for (indRow = 0; indRow < chiselParam.BOX_ROW; indRow++) // 凿击区域法向点票选
  {
    for (indColumn = 0; indColumn < chiselParam.BOX_COLUMN; indColumn++) {
      cout << indRow << "," << indColumn << ": "
           << chiselTable[indRow][indColumn]->getPointNum() << endl;
      chiselTable[indRow][indColumn]->voteTarPoint(
          cloud_shrink, cloud_holes, indRow, indColumn, tarPointList);
      if (chiselTable[indRow][indColumn]->getTarStatus()) {
        chiselTable[indRow][indColumn]->getTarPoint(
            tarPointList[indRow * chiselParam.BOX_COLUMN + indColumn]);
      }
    }
  }

  cloud_tarPoint->width = chiselParam.BOX_ROW * chiselParam.BOX_COLUMN;
  cloud_tarPoint->height = 1;
  cloud_tarPoint->is_dense = false;
  cloud_tarPoint->points.resize(cloud_tarPoint->width * cloud_tarPoint->height);
  for (size_t i = 0; i < chiselParam.BOX_ROW * chiselParam.BOX_COLUMN;
       i++) // 输出结果
  {
    if (tarPointList[i].status) {
      cloud_tarPoint->points[i].x = tarPointList[i].ox;
      cloud_tarPoint->points[i].y = tarPointList[i].oy;
      cloud_tarPoint->points[i].z = tarPointList[i].oz;
      cloud_tarPoint->points[i].normal_x = tarPointList[i].nx;
      cloud_tarPoint->points[i].normal_y = tarPointList[i].ny;
      cloud_tarPoint->points[i].normal_z = tarPointList[i].nz;
      cloud_tarPoint->points[i].curvature = tarPointList[i].curv;
      cout << "tar[" << i << "]: [" << tarPointList[i].ox << ", "
           << tarPointList[i].oy << ", " << tarPointList[i].oz << "] ["
           << tarPointList[i].nx << ", " << tarPointList[i].ny << ", "
           << tarPointList[i].nz << ", " << tarPointList[i].curv << "]" << endl;
    }
  }
  // ********凿击点筛选end********
  for (size_t ii = 0; ii < 4; ii++) {
    for (size_t jj = 0; jj < 6; jj++) {
      delete chiselTable[ii][jj];
    }
  }
#endif

  return true;
}