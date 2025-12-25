#include "hand_eye_calib/hand_eye_calib.h"


template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
	if (!v.empty()) {
		out << '[';
		std::copy(v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
		out << "\b\b]";
	}
	return out;
}
// int num = 15;
//相机中组标定板的位姿，x,y,z,w,x,y,z
// x(m),y,z,X,Y,Z,W
// Mat_<double> CalPose = (cv::Mat_<double>(num, 7) <<
// 	0.06251636, 0.07627178, 0.57626521, 0.91542277, 0.38694012, -0.04580691, 0.10089702,
// 	-0.07842323, 0.07752807, 0.57412624, 0.72674460, -0.67947212, 0.08117323, 0.05975632,
// 	0.05894823, 0.00844423, 0.59284132, 0.99575735, -0.03783592, -0.00848754, 0.08344875,
// 	0.02503873, -0.02977146, 0.83278369, 0.96893213, -0.19874529, 0.08512130, -0.12010490,
// 	-0.01422380, 0.05328534, 0.62632089, 0.97363611, -0.15106572, 0.13701240, -0.10217364,
// 	-0.05387131, 0.07313375, 0.63002890, 0.99171286, 0.00572263, -0.03165138, -0.12438262,
// 	0.15386903, -0.03253585, 0.59770458, 0.95329576, 0.24044574, -0.18228419, -0.01361956,
// 	0.12562420, 0.02792373, 0.61213356, 0.94997601, -0.25754389, -0.17341483, 0.03382308,
// 	0.15373201, 0.06419070, 0.60104608, 0.93313423, -0.30920840, -0.16106739, -0.08779490,
// 	0.10381514, 0.01401372, 0.60763895, 0.95894534, 0.21769059, -0.09602409, -0.15431783,
// 	0.06813184, 0.01466719, 0.62141454, 0.98050296, -0.09521642, -0.13111810, -0.11108239,
// 	0.06537242, 0.07495456, 0.61725592, 0.98924776, 0.00377058, -0.13450104, 0.05730722,
// 	0.09762854, -0.04692571, 0.59945517, 0.99029946, -0.00356752, -0.13883696, -0.00430494,
// 	0.16990689, -0.04563133, 0.59773087, 0.99867977, -0.00761785, -0.01340489, -0.04899989,
// 	0.12183582, -0.14362716, 0.56415432, 0.99792892, -0.00645546, -0.04019975, -0.04980133);
// //机械臂末端的位姿，x,y,z,w,x,y,z
// //x(mm),y,z,a(rz),b(ry),c(rx) Tool1 动轴（内旋）欧拉角
// Mat_<double> ToolPose = (cv::Mat_<double>(num, 6) <<
// 	971.78, 217.96, 1455.55, 8.26, -8.43, 131.77,
// 	971.75, 370.71, 1455.56, -11.77, -0.46, -95.70,
// 	971.76, 318.45, 1385.20, -1.23, -11.72, -177.46,
// 	736.75, 436.73, 1700.78, -8.67, 11.67, -159.38,
// 	944.19, 422.94, 1700.78, -15.02, 10.15, -165.39,
// 	944.19, 145.07, 1751.91, 3.55, 11.54, 178.15,
// 	944.19, 91.58, 1552.22, 19.65, 8.45, 151.88,
// 	944.19, 129.46, 1443.02, 17.92, -11.75, -153.57,
// 	944.19, 129.46, 1669.19, 17.65, 0.91, -145.24,
// 	944.22, 229.67, 1716.29, 7.74, 15.94, 153.98,
// 	944.24, 109.76, 1657.59, 15.69, 8.27, -169.50,
// 	944.24, 114.94, 1488.18, 14.97, -8.25, 176.54,
// 	944.25, 147.92, 1488.17, 14.80, 0.83, 178.96,
// 	944.26, 437.51, 1488.17, -1.98, 0.83, 178.96,
// 	963.36, 336.82, 1390.41, 2.11, 0.82, 178.96);
//R和T转RT矩阵
Mat R_T2RT(Mat& R, Mat& T)
{
	Mat RT;
	Mat_<double> R1 = (cv::Mat_<double>(4, 3) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
		R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
		R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2),
		0.0, 0.0, 0.0);
	cv::Mat_<double> T1 = (cv::Mat_<double>(4, 1) << T.at<double>(0, 0), T.at<double>(1, 0), T.at<double>(2, 0), 1.0);

	cv::hconcat(R1, T1, RT);//C=A+B左右拼接
	return RT;
}

//RT转R和T矩阵
void RT2R_T(Mat& RT, Mat& R, Mat& T)
{
	cv::Rect R_rect(0, 0, 3, 3);
	cv::Rect T_rect(3, 0, 1, 3);
	R = RT(R_rect);
	T = RT(T_rect);
}

//判断是否为旋转矩阵
bool isRotationMatrix(const cv::Mat& R)
{
	cv::Mat tmp33 = R({ 0,0,3,3 });
	cv::Mat shouldBeIdentity;

	shouldBeIdentity = tmp33.t() * tmp33;

	cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

	return  cv::norm(I, shouldBeIdentity) < 1e-6;
}

/** @brief 欧拉角 -> 3*3 的R
*	@param 	eulerAngle		角度值
*	@param 	seq				指定欧拉角xyz的排列顺序如："xyz" "zyx"
*/
cv::Mat eulerAngleToRotatedMatrix(const cv::Mat& eulerAngle, const std::string& seq)
{
	CV_Assert(eulerAngle.rows == 1 && eulerAngle.cols == 3);

	eulerAngle /= 180 / CV_PI;
	cv::Matx13d m(eulerAngle);
	// auto rx = m(0, 0), ry = m(0, 1), rz = m(0, 2);
	auto rz = m(0, 0), ry = m(0, 1), rx = m(0, 2);
	auto xs = std::sin(rx), xc = std::cos(rx);
	auto ys = std::sin(ry), yc = std::cos(ry);
	auto zs = std::sin(rz), zc = std::cos(rz);

	cv::Mat rotX = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, xc, -xs, 0, xs, xc);
	cv::Mat rotY = (cv::Mat_<double>(3, 3) << yc, 0, ys, 0, 1, 0, -ys, 0, yc);
	cv::Mat rotZ = (cv::Mat_<double>(3, 3) << zc, -zs, 0, zs, zc, 0, 0, 0, 1);

	cv::Mat rotMat;
	// 动轴（内旋）欧拉角度
	if (seq == "zyx")		rotMat = rotZ * rotY * rotX;
	else if (seq == "yzx")	rotMat = rotY * rotZ * rotX;
	else if (seq == "zxy")	rotMat = rotZ * rotX * rotY;
	else if (seq == "xzy")	rotMat = rotX * rotZ * rotY;
	else if (seq == "yxz")	rotMat = rotY * rotX * rotZ;
	else if (seq == "xyz")	rotMat = rotX * rotY * rotZ;
	else {
		cv::error(cv::Error::StsAssert, "Euler angle sequence string is wrong.",
			__FUNCTION__, __FILE__, __LINE__);
	}

	if (!isRotationMatrix(rotMat)) {
		cv::error(cv::Error::StsAssert, "Euler angle can not convert to rotated matrix",
			__FUNCTION__, __FILE__, __LINE__);
	}

	return rotMat;
	//cout << isRotationMatrix(rotMat) << endl;
}

/** @brief 四元数转旋转矩阵
*	@note  数据类型double； 四元数定义 q = w + x*i + y*j + z*k
*	@param q 四元数输入{w,x,y,z}向量
*	@return 返回旋转矩阵3*3
*/
cv::Mat quaternionToRotatedMatrix(const cv::Vec4d& q)
{
	// double w = q[0], x = q[1], y = q[2], z = q[3];
	double w = q[3], x = q[0], y = q[1], z = q[2];

	double x2 = x * x, y2 = y * y, z2 = z * z;
	double xy = x * y, xz = x * z, yz = y * z;
	double wx = w * x, wy = w * y, wz = w * z;

	cv::Matx33d res{
		1 - 2 * (y2 + z2),	2 * (xy - wz),		2 * (xz + wy),
		2 * (xy + wz),		1 - 2 * (x2 + z2),	2 * (yz - wx),
		2 * (xz - wy),		2 * (yz + wx),		1 - 2 * (x2 + y2),
	};
	return cv::Mat(res);
}

/** @brief ((四元数||欧拉角||旋转向量) && 转移向量) -> 4*4 的Rt
*	@param 	m				1*6 || 1*7的矩阵  -> 6  {x,y,z, rx,ry,rz}   7 {x,y,z, qw,qx,qy,qz}
*	@param 	useQuaternion	如果是1*7的矩阵，判断是否使用四元数计算旋转矩阵
*	@param 	seq				如果通过欧拉角计算旋转矩阵，需要指定欧拉角xyz的排列顺序如："xyz" "zyx" 为空表示旋转向量
*/
cv::Mat attitudeVectorToMatrix(cv::Mat m, bool useQuaternion, const std::string& seq)
{
	CV_Assert(m.total() == 6 || m.total() == 7);
	if (m.cols == 1)
		m = m.t();
	cv::Mat tmp = cv::Mat::eye(4, 4, CV_64FC1);
	//如果使用四元数转换成旋转矩阵则读取m矩阵的第第四个成员，读4个数据
	if (useQuaternion)	// normalized vector, its norm should be 1.
	{
		cv::Vec4d quaternionVec = m({ 3, 0, 4, 1 });
		quaternionToRotatedMatrix(quaternionVec).copyTo(tmp({ 0, 0, 3, 3 }));
		// cout << norm(quaternionVec) << endl; 
	}
	else
	{
		cv::Mat rotVec;
		if (m.total() == 6)
			rotVec = m({ 3, 0, 3, 1 });		//6
		else
			rotVec = m({ 7, 0, 3, 1 });		//10

		//如果seq为空表示传入的是旋转向量，否则"xyz"的组合表示欧拉角
		if (0 == seq.compare(""))
			cv::Rodrigues(rotVec, tmp({ 0, 0, 3, 3 }));
		else
			eulerAngleToRotatedMatrix(rotVec, seq).copyTo(tmp({ 0, 0, 3, 3 }));
	}
	tmp({ 3, 0, 1, 3 }) = m({ 0, 0, 3, 1 }).t();
	//std::swap(m,tmp);
	return tmp;
}
bool hand_eye_calib_calc(int num, 
					Mat& CalPose,
					Mat& ToolPose,
					cv::Mat& Homo_cam2gripper)//定义相机camera到末端grab的位姿矩阵
{
	//定义手眼标定矩阵
	std::vector<Mat> R_gripper2base;
	std::vector<Mat> t_gripper2base;
	std::vector<Mat> R_target2cam;
	std::vector<Mat> t_target2cam;
	Mat R_cam2gripper = (Mat_<double>(3, 3));
	Mat t_cam2gripper = (Mat_<double>(3, 1));

	vector<Mat> images;
	size_t num_images = num;

	// 读取末端，标定板的姿态矩阵 4*4
	std::vector<cv::Mat> Homo_gripper2base, Homo_target2cam;
	
	Mat tempR, tempT;

	std::cout << "输入CalPose为： " << std::endl;
	std::cout << CalPose << std::endl;
	std::cout << "输入ToolPose为： " << std::endl;
	std::cout << ToolPose << std::endl;

	for (size_t i = 0; i < num_images; i++)//计算标定板位姿
	{
		cv::Mat tmp = attitudeVectorToMatrix(CalPose.row(i), true, ""); //转移向量转旋转矩阵
		Homo_target2cam.push_back(tmp);
		RT2R_T(tmp, tempR, tempT);

		R_target2cam.push_back(tempR);
		t_target2cam.push_back(tempT);
	}
	//cout << R_target2cam << endl;
	for (size_t i = 0; i < num_images; i++)//计算机械臂位姿
	{
		// mm转为m
		cv::Mat tmpTool = ToolPose.row(i);
		double* data = tmpTool.ptr<double>(0);
		data[0] = data[0] / 1000;
		data[1] = data[1] / 1000;
		data[2] = data[2] / 1000;

		cv::Mat tmp = attitudeVectorToMatrix(tmpTool, false, "zyx"); // 动轴zyx  = 定轴 xyz
		// cv::Mat tmp = attitudeVectorToMatrix(ToolPose.row(i), true, ""); //机械臂位姿为欧拉角-旋转矩阵
		//tmp = tmp.inv();//机械臂基座位姿为欧拉角-旋转矩阵
		Homo_gripper2base.push_back(tmp);
		RT2R_T(tmp, tempR, tempT);

		R_gripper2base.push_back(tempR);
		t_gripper2base.push_back(tempT);

	}
	//cout << t_gripper2base[0] << " " << t_gripper2base[1] << " " << t_gripper2base[2] << " " << endl;
	//cout << t_gripper2base << endl;
	//手眼标定，CALIB_HAND_EYE_TSAI法为11ms，最快
	calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper, t_cam2gripper, CALIB_HAND_EYE_TSAI);

	Homo_cam2gripper = R_T2RT(R_cam2gripper, t_cam2gripper);//矩阵合并

	std::cout << "Homo_cam2gripper 矩阵为： " << std::endl;
	std::cout << Homo_cam2gripper << std::endl;
	cout << "是否为旋转矩阵：" << isRotationMatrix(Homo_cam2gripper) << std::endl << std::endl;//判断是否为旋转矩阵

	//Tool_In_Base*Hcg*Cal_In_Cam，用第一组和第二组进行对比验证
	cout << "第一组和第二组手眼数据验证：" << endl;
	cout << Homo_gripper2base[0] * Homo_cam2gripper * Homo_target2cam[0] << endl << Homo_gripper2base[1] * Homo_cam2gripper * Homo_target2cam[1] << endl << endl;//.inv()

	cout << "标定板在相机中的位姿：" << endl;
	cout << Homo_target2cam[1] << endl;
	cout << "手眼系统反演的位姿为：" << endl;
	//用手眼系统预测第一组数据中标定板相对相机的位姿，是否与Homo_target2cam[1]相同
	cout << Homo_cam2gripper.inv() * Homo_gripper2base[1].inv() * Homo_gripper2base[0] * Homo_cam2gripper * Homo_target2cam[0] << endl << endl;

	cout << "----手眼系统测试----" << endl;
	cout << "机械臂下标定板XYZ为：" << endl;
	for (int i = 0; i < Homo_target2cam.size(); ++i)
	{
		cv::Mat cheesePos{ 0.0,0.0,0.0,1.0 };//4*1矩阵，单独求机械臂下，标定板的xyz
		cv::Mat worldPos = Homo_gripper2base[i] * Homo_cam2gripper * Homo_target2cam[i] * cheesePos;
		cout << i << ": " << worldPos.t() << endl;
	}
	getchar();
	return true;
}

