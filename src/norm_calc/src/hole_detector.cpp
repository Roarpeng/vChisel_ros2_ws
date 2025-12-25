#include "norm_calc/hole_detector.h"

using namespace std;
using namespace cv;

bool holeDetector(cv::Mat src, cv::Mat depthInfo, sensor_msgs::CameraInfo camInfo, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_holes)
{
    cv::Mat src_in, src_binary,src_morp, src_gray,src_distance;
    
    src_in = src.clone();
    cout << "Width : " << src_in.size().width << endl;
	cout << "Height: " << src_in.size().height << endl;
	cout<<"Channels: :"<< src_in.channels() << endl;
    cout << "input typr:" << src_in.type() << endl;

    
    
    // imshow("原图片", src_in);

    //cout << "sss1"<< endl;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3), Point(-1, -1));
    morphologyEx(src_in, src_morp, MORPH_OPEN, kernel);
    // imshow("形态学",src_morp);
    //cout << "sss2"<< endl;
    cvtColor(src_morp, src_gray, COLOR_BGR2GRAY);

    //cout << "sss3"<< endl;
    // threshold(src_gray, src_binary, 45, 255, THRESH_BINARY_INV);
    adaptiveThreshold(src_gray, src_binary, 255, ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY_INV,69,25);
    
    //cout << "sss4"<< endl;
    // imshow("二值化", src_binary);
    vector<vector<Point>> contours;
    findContours(src_binary, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));
    RNG rng(12345);
    double area;
    //cout << "sss5"<< endl;
    Point2i PL, PC;
    size_t ptNum = 0;
    float w2h = 0.0f;
    RotatedRect minRect;
    vector<Point2i> centerLists;
    for (size_t i = 0; i < contours.size(); i++)
    {
        area = contourArea(contours[i]);
        if (area < 25 || area > 1600) continue;

        minRect = minAreaRect(contours[i]);
        w2h = (float)minRect.size.width / (float)minRect.size.height;
        if (w2h < 0.2f || w2h > 5.0f ) continue;

        ptNum++;
        PL = contours[i].front();
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        drawContours(src_in, contours, i, color, 2, 8);
        // putText(src_in, to_string(ptNum), PL, FONT_HERSHEY_COMPLEX, 1, color, 2);   
        
        PC = Point2i(minRect.center.x, minRect.center.y);
        centerLists.push_back(PC);
        circle(src_in,PC,5,color,FILLED);
    }
    cout << "Holes Num: " << centerLists.size() << endl;
    // imshow("计数结果", src_in);

    if (centerLists.size() > 0)
    {
        cloud_holes->width = centerLists.size();
        cloud_holes->height = 1;
        cloud_holes->is_dense = false;
        cloud_holes->points.resize(cloud_holes->width * cloud_holes->height);

        for (int i = 0; i < centerLists.size(); i++)
        {
            cloud_holes->points[i].z = 0.001f * depthInfo.at<u_int16_t>(centerLists[i].y, centerLists[i].x);
            cloud_holes->points[i].x = (centerLists[i].x - camInfo.K.at(2))/camInfo.K.at(0) * cloud_holes->points[i].z;
            cloud_holes->points[i].y = (centerLists[i].y - camInfo.K.at(5))/camInfo.K.at(4) * cloud_holes->points[i].z;
            cloud_holes->points[i].z -= 0.01f; // 为了显示不遮挡
            // cout << centerLists[i].x << ", "<<centerLists[i].y << endl;
            // cout << cloud_holes->points[i].x << ", "<<cloud_holes->points[i].y << ", "<<cloud_holes->points[i].z << endl;
        }
    }
    
    // waitKey(0);
    return true;
}
