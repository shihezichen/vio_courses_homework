
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <iomanip>

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <eigen3/Eigen/Dense>
#include "System.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const int nDelayTimes = 2;
string sData_path = "/home/weihao/Datastes/MH_01_easy/mav0/";
string sConfig_path = "../config/";

std::shared_ptr<System> pSystem;//宏定义整个VIO系统类

// 获取IMU数据
void PubImuData()
{
	string sImu_data_file = sConfig_path + "MH_01_easy_imu0.txt";
	cout << "1 PubImuData start sImu_data_filea: " << sImu_data_file << endl;
	ifstream fsImu;
	fsImu.open(sImu_data_file.c_str());
	if (!fsImu.is_open())
	{
		cerr << "Failed to open imu file! " << sImu_data_file << endl;
		return;
	}//读取文件

	std::string sImu_line;
	double dStampNSec = 0.0;//时间戳
	Vector3d vAcc;//加速度
	Vector3d vGyr;//陀螺仪
	while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data
	{
		std::istringstream ssImuData(sImu_line);
		ssImuData >> dStampNSec >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
		// cout << "Imu t: " << fixed << dStampNSec << " gyr: " << vGyr.transpose() << " acc: " << vAcc.transpose() << endl;
		pSystem->PubImuData(dStampNSec / 1e9, vGyr, vAcc);
		usleep(5000*nDelayTimes);//将调用usleep函数的线程挂起一段时间,单位是微秒
	}
	fsImu.close();
}

// 获取图像数据
void PubImageData()
{
	string sImage_file = sConfig_path + "MH_01_easy_cam0.txt";

	cout << "1 PubImageData start sImage_file: " << sImage_file << endl;

	ifstream fsImage;
	fsImage.open(sImage_file.c_str());
	if (!fsImage.is_open())
	{
		cerr << "Failed to open image file! " << sImage_file << endl;
		return;
	}

	std::string sImage_line;
	double dStampNSec;
	string sImgFileName;
	
	// cv::namedWindow("SOURCE IMAGE", CV_WINDOW_AUTOSIZE);
	while (std::getline(fsImage, sImage_line) && !sImage_line.empty())
	{
		std::istringstream ssImuData(sImage_line);//读取图像路径数据
		ssImuData >> dStampNSec >> sImgFileName;
		// cout << "Image t : " << fixed << dStampNSec << " Name: " << sImgFileName << endl;
		string imagePath = sData_path + "cam0/data/" + sImgFileName;

		Mat img = imread(imagePath.c_str(), 0);
		if (img.empty())
		{
			cerr << "image is empty! path: " << imagePath << endl;
			return;
		}
		pSystem->PubImageData(dStampNSec / 1e9, img);
		// cv::imshow("SOURCE IMAGE", img);
		// cv::waitKey(0);
		usleep(50000*nDelayTimes);
	}
	fsImage.close();
}

int main(int argc, char **argv)
{
	if(argc != 3)
	{
		cerr << "./run_euroc PATH_TO_FOLDER/MH-05/mav0 PATH_TO_CONFIG/config \n" 
			<< "For example: ./run_euroc /home/stevencui/dataset/EuRoC/MH-05/mav0/ ../config/"<< endl;
		return -1;
	}
	sData_path = argv[1];
	sConfig_path = argv[2];
    // 多线程处理不同的事情
	pSystem.reset(new System(sConfig_path));//核心类
	// std::thread 初始化为函数,表示直接执行函数,初始化为类函数,则后边应该跟着类
	std::thread thd_BackEnd(&System::ProcessBackEnd, pSystem);//后端优化最重要的
		
	// sleep(5);
	std::thread thd_PubImuData(PubImuData);//获取IMU数据线程

	std::thread thd_PubImageData(PubImageData);//获取图像数据线程
	
	std::thread thd_Draw(&System::Draw, pSystem);//显示轨迹线程

	// 数据运行完就把线程停掉,但画图和主线程仍然在运行
	thd_PubImuData.join();
	thd_PubImageData.join();

	// thd_BackEnd.join();
	// thd_Draw.join();

	cout << "main end... see you ..." << endl;
	return 0;
}
