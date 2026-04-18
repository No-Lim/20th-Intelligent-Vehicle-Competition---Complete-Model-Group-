/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习,请勿商用).
 *            The code ADAPTS the corresponding hardware circuit board(代码适配百度Edgeboard-智能汽车赛事版),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file common.hpp
 * @author Leo
 * @brief 通用方法类
 * @version 0.1
 * @date 2024-01-12
 *
 * @copyright Copyright (c) 2024
 *
 */
 
#pragma once
#include "json.hpp"
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include <sstream>
#include <unistd.h>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <chrono>
#include <cstdint>
#include <vector>



using namespace std;
using namespace cv;


vector<Point> LeftPoint;     // 赛道左边缘点集
vector<Point> RightPoint;    // 赛道右边缘点集
vector<uint16_t>trackwidth;             //赛道宽度集合

#define COLSIMAGE 300    // 图像的列数
#define ROWSIMAGE 200    // 图像的行数
#define COLSIMAGEIPM 320 // IPM图像的列数
#define ROWSIMAGEIPM 400 // IPM图像的行数
#define PWMSERVOMAX 1450 // 舵机PWM最大值（左）
#define PWMSERVOMID 1050// 舵机PWM中值 
#define PWMSERVOMIN 650 // 舵机PWM最小值（右）
#define MT9V03X_W              300             	//图像宽度 	范围1-188
#define MT9V03X_H              200            	//图像高度	范围1-120


int cutimgup = 20;
int cutimgdown = 5;

// #define LABEL_BOMB 0      // AI标签：爆炸物
// #define LABEL_BRIDGE 1    // AI标签：坡道
// #define LABEL_SAFETY 2    // AI标签：普通车辆
// #define LABEL_CONE 3      // AI标签：锥桶
// #define LABEL_CROSSWALK 4 // AI标签：斑马线
// #define LABEL_DANGER 5    // AI标签：危险车辆
// #define LABEL_EVIL 6      // AI标签：恐怖分子
// #define LABEL_BLOCK 7     // AI标签：障碍物
// #define LABEL_PATIENT 8   // AI标签：伤员
// #define LABEL_PROP 9      // AI标签：道具车
// #define LABEL_SPY 10      // AI标签：嫌疑车辆
// #define LABEL_THIEF 11    // AI标签：盗贼
// #define LABEL_TUMBLE 12   // AI标签：跌倒

///避障元素
#define LABEL_BLOCK 1      // AI标签：障碍物
#define LABEL_CONE 6       // AI标签：锥桶
#define LABEL_PEDESTRIAN 8 // AI标签：行人

#define LABEL_BATTERY 0    // AI标签：充电站
#define LABEL_BRIDGE 2     // AI标签：坡道
#define LABEL_BURGER 3     // AI标签：汉堡
#define LABEL_CAR 4        // AI标签：道具车
#define LABEL_COMPANY 5    // AI标签：公司
#define LABEL_CROSSWALK 7  // AI标签：斑马线
#define LABEL_SCHOOL 9     // AI标签：学校


#define AT_IMAGE(img, x, y) (img.at<uint8_t>(y, x)) //先列后行
#define AT_MIN(a, b) (((a) < (b)) ? (a) : (b))
#define AT_MAX(a, b) (((a) > (b)) ? (a) : (b))
#define AT_MINMAX(input, low, upper) MIN(MAX(input, low), upper)

#define IMAGE_WIDTH 300  // 图像的列数(y)
#define IMAGE_HEIGHT 200 // 图像的行数(x)
#define MAZE 240 // 迷宫点数


#define BEGIN_X (140)  // 巡线横坐标起始点(列)
#define BEGIN_Y (180)  // 巡线纵坐标起始点(行)
int16_t begin_x           =7;                                   //搜索左右边线起始点坐标
int16_t begin_y           =IMAGE_HEIGHT *0.9;                                   //搜索左右边线起始点坐标

#define ROAD_WIDTH (0.46)      // 赛道宽度 (0.4)
#define BLOCK_SIZE (7)         // 自适应阈值的block大小 (7)
int16_t block_size = BLOCK_SIZE;

#define CLIP_VALUE (4)         // 自适应阈值的阈值裁减量 (2)
int16_t  clip_value = 4;

#define LINE_BLUR_KERNEL (7)   // 边线三角滤波核的大小 (7)
int16_t line_blur_kernel = 7;

// #define PIXEL_PER_METER (90)  // 俯视图中, 每个像素对应的长度 (205)
// int16_t pixel_per_meter =90;



#define PIXEL_PER_METER (82)  // 俯视图中, 每个像素对应的长度 (205)
int16_t pixel_per_meter =82;

#define SAMPLE_DIST (0.02)     // 边线等距采样的间距 (0.02)
float sample_dist = 0.02;

#define ANGLE_DIST (0.2)       // 计算边线转角时, 三个计算点的距离 (0.2)
float angle_dist = 0.2;

#define PI (3.14159265)

bool angle_check=false;

int i_stop=false;
bool start_bu=false;

//int vector_size = 0;

bool parker_first = false;
bool jump_burger = false;

int circle_count = 0;

bool parker_near_moveFlag_L=false;
bool parker_near_moveFlag_R = false;

// bool anjian_stop = false;
int16_t my_cross_route = 0, my_parker_route = 0, my_circle_route = 0;  //记录十字，停车区，位置距离

bool cirle_1 = true, cross_1 = true;
bool first = false;
int16_t thres             =100;
bool car_right = false;
bool car_left = false;

int Lpt0_found_flag,Lpt1_found_flag;
int far_Lpt0_found_flag,far_Lpt1_found_flag;

int16_t OSTU_thres;                                                  //大津法阈值
/************************************************************************************/
enum flag_str_road_e{
    ROAD_NONE = 0,
    ROAD_START,
    ROAD_ING, 
};
flag_str_road_e flag_str_road = ROAD_NONE;
int16_t road_route = 0;
bool need_brake = false;


/********************************刹车************************************************/
enum flag_brake_e {
    BRAKE_NONE = 0,
    BRAKE_START,
    BRAKE_ING,

};
flag_brake_e flag_brake = BRAKE_NONE;
int16_t brake_route = 0;  


/*************************************************************************************/

uint8_t direction_parker = 0 , parker_out_stop_flag = 0, b_flag = 6;
bool chuting = false, AI_flag = false;
bool flag_50 = false;
bool chonghe = false;


//bool flag_stop = false;

float turnP = 0, turnD = 0;

/*************************************AI元素枚举***********************************/
int16_t element_over_route = 0;

/**********************停车保护*********************************/
int lose_points = 0;
enum Protect_car {
    PROTECT_NONE,
    PROTECT_STOP,
};
Protect_car flag_protect = PROTECT_NONE;
bool stop = false;

enum flag_ramp_e {
    RAMP_NONE = 0,   // 非坡道模式
    RAMP_DETECTION,  // 检测到坡道
    RAMP_UP,         // 上坡阶段
    RAMP_DOWN,       // 下坡阶段
};
flag_ramp_e flag_ramp = RAMP_NONE;


enum flag_parker_e {
    PARKER_NONE = 0,
    PARKER_DETECTION,   // 检测到停车区
    PARKER_READY,       // 将入 
    PARKER_IN,
    PARKER_RUNING,
    PARKER_STOP,        // 停车
    PARKER_OUT,         // 出
    PARKER_FAR_OUT0,
    PARKER_FAR_OUT1,

    PARKER_END,
};
flag_parker_e flag_parker = PARKER_NONE;
bool move_flag = false;

uint8_t jiansu_cout = 5;
bool jiansu_flag = false;

bool qianjin_flag = false, qianjin_used= false;
uint8_t true_flag = 0;


uint8_t parker_ready_0 = 1,  parker_ready_70 = 10, parker_ready_50 = 1; 


uint8_t  parker_stop_0 = 1, parker_stop_50 = 5; //parker_stop_70 = 23,
bool flag_70 = false;

/////////////////////障碍物////////////////////////////
enum flag_obstacle_e 
{
    OBSTACLE_NONE = 0,
    OBSTACLE_DETECTION,   // 检测到障碍物
    OBSTACLE_READY,       // 将入
    OBSTACLE_MULTIPLE ,      // 多个障碍物处理
   // OBSTACLE_MULTIPLE_LEFT_RIGHT,       // 多个障碍物处理
    //OBSTACLE_MULTIPLE_RIGHT_LEFT      // 多个障碍物处理
};
flag_obstacle_e flag_obstacle = OBSTACLE_NONE;
enum flag_block_e
{
    BLOCK_NONE=0,
    BLOCK_VOID
};
flag_block_e flag_block=BLOCK_NONE;


bool block_move_flag=false;
bool avoid_block=false;

bool block_left_ai = false;
bool block_right_ai = false;

bool block_qianzhan_flag = false;
bool block_left_move = false;
bool block_right_move=false;

bool cone_left = false, cone_right = false;
bool layby_left =false,layby_right = false;
bool parker_left = false,parker_right= false;
bool car_up = false,car_down = false;
///////////////////餐饮区///////////////////
enum flag_burger_e    
{
    BURGER_NONE = 0,
    BURGER_DETECTION,
    BURGER_START,
    BURGER_STOP,  
    BURGER_END,
};
flag_burger_e flag_burger = BURGER_NONE ;
bool burger_left = false;
bool burger_right = false;
///////////////////临时停车区///////////////////
enum flag_layby_e {
    LAYBY_NONE = 0,
    LAYBY_DETECTION,   // 检测到停车区

    LAYBY_READY,       // 将入
    LAYBY_READY2,      //   
    LAYBY_RUNING,         // 看不到标识牌
    LAYBY_STOP,        // 停车
    LAYBY_OUT,         // 出
};
flag_layby_e flag_layby = LAYBY_NONE;




enum flag_crosswalk_e    //斑马线停车
{
    CROSSWALK_NONE = 0,
    CROSSWALK_DETECTION,
    CROSSWALK_START,
    CROSSWALK_END,
    CROSSWALK_STOP,
    
};
flag_crosswalk_e flag_crosswalk = CROSSWALK_NONE;
uint8_t crosswalk_stop0 = 2;//crosswalk_stop70 = 30;


/*************************************巡线类型枚举***********************************/

enum flag_track_e 
{
    TRACK_LEFT,    // 寻左线
    TRACK_MIDDLE,  // 寻中线
    TRACK_RIGHT,   // 寻右线
};

flag_track_e flag_track = TRACK_MIDDLE;
/*************************************外部变量***********************************/
// 向下位机传输的数据包
struct Payload_t 
{
    uint16_t tSpeed;   // 速度
    uint16_t tAngle;   // 偏移
    uint8_t element;  // 元素
};

// 从下位机接受的数据包 车模速度
struct Encoder_t 
{
    int16_t speed;  // 速度
    int16_t route;  // 路程
};

enum flag_cross_e 
{
    CROSS_NONE = 0,  // 非十字模式

    CROSS_BEGIN,     // 找到左右两个 L 角点

    CROSS_IN,        // 两个 L 角点很近, 即进入十字内部(此时切换远线控制)  元素结束了？
    CROSS_ING0,
    CROSS_ING,
    CROSS_OUT,

    CROSS_HALF_LEFT,
    CROSS_HALF_RIGHT,
    CROSS_HALF,     
};
flag_cross_e flag_cross = CROSS_NONE;
bool xieru_flag = false;


enum flag_circle_e 
{
    CIRCLE_NONE = 0,  // 非环岛模式

    CIRCLE_LEFT_BEGIN,
    CIRCLE_RIGHT_BEGIN,  // 环岛开始, 识别到单侧 L 角点另一侧长直道。

    CIRCLE_LEFT_IN,
    CIRCLE_RIGHT_IN,  // 环岛进入, 即走到一侧直道, 一侧环岛的位置。
   
    CIRCLE_LEFT_RUNNING,
    CIRCLE_RIGHT_RUNNING,  // 环岛内部。

    CIRCLE_LEFT_OUT,
    CIRCLE_RIGHT_OUT,  // 准备出环岛, 即识别到出环处的 L 角点。

    CIRCLE_LEFT_OUT_NONE,  //出左环到二次角点之间的状态
    CIRCLE_RIGHT_OUT_NONE,  //出右环到二次角点之间的状态

    CIRCLE_RIGHT_END,
    CIRCLE_LEFT_END,

};
flag_circle_e flag_circle = CIRCLE_NONE;


bool circle_angle_flage=false;




#define threshold_max	255*5//此参数可根据自己的需求调节
#define threshold_min	255*2//此参数可根据自己的需求调节

void image_filter(Mat& img)//形态学滤波，简单来说就是膨胀和腐蚀的思想
{
	uint16_t i, j;
	uint32_t num = 0;


	for (i = 1; i < ROWSIMAGE - 1; i++)
	{
		for (j = 1; j < (COLSIMAGE - 1); j++)
		{
			//统计八个方向的像素值
			//img.at<uchar>(i, j)
			num =
				img.at<uchar>(i - 1, j - 1) + img.at<uchar>(i - 1, j) + img.at<uchar>(i - 1, j + 1)
				+ img.at<uchar>(i, j - 1) + img.at<uchar>(i, j + 1)
				+ img.at<uchar>(i + 1, j - 1) + img.at<uchar>(i + 1, j) + img.at<uchar>(i + 1, j + 1);


			if (num >= threshold_max && img.at<uchar>(i, j) == 0)
			{

				img.at<uchar>(i, j) = 255;//白  可以搞成宏定义，方便更改

			}
			if (num <= threshold_min && img.at<uchar>(i, j) == 255)
			{

				img.at<uchar>(i, j) = 0;//黑

			}

		}
	}

}



////////////////////////////////////
using namespace std::chrono;

template <typename T>

class BlockingQueue {
   private:
    BlockingQueue(const BlockingQueue &rhs);
    BlockingQueue &operator=(const BlockingQueue &rhs);

    mutable mutex _mutex;
    condition_variable _condvar;
    deque<T> _queue;
    bool _is_shut_down = false;

   public:
    BlockingQueue() : _mutex(), _condvar(), _queue() {}
    ~BlockingQueue() {}

    void ShutDown() {
        _is_shut_down = true;
        _condvar.notify_all();
        _queue.clear();
    }
    bool IsShutDown() { return _is_shut_down; }

    void Put(const T task) {
        lock_guard<mutex> lock(_mutex);
        if (!_is_shut_down) {
            { _queue.push_back(task); }
            _condvar.notify_all();
        }
    }

    T Take() {
        unique_lock<mutex> lock(_mutex);
        while (_queue.size() <= 0 && !_is_shut_down) {
            _condvar.wait_for(lock, std::chrono::milliseconds(1));
        }
        if (_is_shut_down || _queue.empty()) {
            throw exception();
        }
        T front(_queue.front());
        _queue.pop_front();
        return front;
    }
    //3.19 加 
    //帧率没啥变化 也没出问题 
    bool TryTake(T& item) {
        lock_guard<mutex> lock(_mutex);
        if (_is_shut_down || _queue.empty()) {
            return false;
        }
        item = _queue.front();
        _queue.pop_front();
        return true;
    }

    size_t Size() const {
        lock_guard<mutex> lock(_mutex);
        return _queue.size();
    }
};


/**
 * @brief 场景类型（路况）
 *
 */
// enum Scene
// {
//     NormalScene = 0, // 基础赛道
//     CrossScene,      // 十字道路
//     RingScene,       // 环岛道路
//     BridgeScene,     // 坡道区
//     DangerScene,     // 危险区
//     RescueScene,     // 救援区
//     RacingScene,     // 追逐区
//     BlocksScene,     // 障碍区
//     ParkingScene,    // 停车区
// };

/**
 * @brief Get the Scene object
 *
 * @param scene
 * @return string
 */
// string getScene(Scene scene)
// {
//     switch (scene)
//     {
//     case Scene::NormalScene:
//         return "Normal";
//     case Scene::CrossScene:
//         return "Crossroad";
//     case Scene::RingScene:
//         return "Ring";
//     case Scene::BridgeScene:
//         return "Bridge";
//     case Scene::DangerScene:
//         return "Danger";
//     case Scene::RescueScene:
//         return "Rescue";
//     case Scene::RacingScene:
//         return "Racing";
//     case Scene::BlocksScene:
//         return "Blocks";
//     case Scene::ParkingScene:
//         return "Parking";
//     default:
//         return "Error";
//     }
// }

/**
 * @brief 构建二维坐标
 *
 */
struct POINT
{
    int x = 0;
    int y = 0;
    // float slope = 0.0f;

    POINT(){};
    POINT(int x, int y) : x(x), y(y){};
};


/**
 * @brief 存储图像至本地
 *
 * @param image 需要存储的图像
 */
void savePicture(Mat &image)
{
    // 存图
    string name = ".jpg";
    static int counter = 0;
    counter++;
    string img_path = "../res/samples/train/";
    name = img_path + to_string(counter) + ".jpg";
    imwrite(name, image);
}

//--------------------------------------------------[公共方法]----------------------------------------------------
/**
 * @brief int集合平均值计算
 *
 * @param arr 输入数据集合
 * @return double
 */
double average(vector<int> vec)
{
    if (vec.size() < 1)
        return -1;

    double sum = 0;
    for (uint32_t i = 0; i < vec.size(); i++)
    {
        sum += vec[i];
    }

    return (double)sum / vec.size();
}

/**
 * @brief int集合数据方差计算
 *
 * @param vec Int集合
 * @return double
 */
double sigma(vector<int> vec)
{
    if (vec.size() < 1)
        return 0;

    double aver = average(vec); // 集合平均值
    double sigma = 0;
    for (uint32_t i = 0; i < vec.size(); i++)
    {
        sigma += (vec[i] - aver) * (vec[i] - aver);
    }
    sigma /= (double)vec.size();
    return sigma;
}

/**
 * @brief 赛道点集的方差计算
 *
 * @param vec
 * @return double
 */
double sigma(vector<POINT> vec)
{
    if (vec.size() < 1)
        return 0;

    double sum = 0;
    for (uint32_t i = 0; i < vec.size(); i++)
    {
        sum += vec[i].y;
    }
    double aver = (double)sum / vec.size(); // 集合平均值

    double sigma = 0;
    for (uint32_t i = 0; i < vec.size(); i++)
    {
        sigma += (vec[i].y - aver) * (vec[i].y - aver);
    }
    sigma /= (double)vec.size();
    return sigma;
}
double sigma(float pts[][2], int num) {
    if (num < 1)
        return 0;

    double sum = 0;
    for (int i = 0; i < num; i++) sum += pts[i][0];

    double aver = (double)sum / num;
    double sigma = 0;

    for (int i = 0; i < num; i++)
        sigma += (pts[i][0] - aver) * (pts[i][0] - aver);
    sigma /= (double)num;

    return sigma;
}
int16_t limit_a_b(int16_t x, int a, int b)
{
	if (x < a) x = a;
	if (x > b) x = b;
	return x;
}

int getsmall (int x,int y)
{
    if(x>y) return y;
    else return x;
}

/**
 * @brief 阶乘计算
 *
 * @param x
 * @return int
 */
int factorial(int x)
{
    int f = 1;
    for (int i = 1; i <= x; i++)
    {
        f *= i;
    }
    return f;
}

/**
 * @brief 贝塞尔曲线
 *
 * @param dt
 * @param input
 * @return vector<POINT>
 */
// vector<POINT> Bezier(double dt, vector<POINT> input)
// {
//     vector<POINT> output;

//     double t = 0;
//     while (t <= 1)
//     {
//         POINT p;
//         double x_sum = 0.0;
//         double y_sum = 0.0;
//         int i = 0;
//         int n = input.size() - 1;
//         while (i <= n)
//         {
//             double k =
//                 factorial(n) / (factorial(i) * factorial(n - i)) * pow(t, i) * pow(1 - t, n - i);
//             x_sum += k * input[i].x;
//             y_sum += k * input[i].y;
//             i++;
//         }
//         p.x = x_sum;
//         p.y = y_sum;
//         output.push_back(p);
//         t += dt;
//     }
//     return output;
// }

// 贝塞尔曲线拟合
std::vector<POINT> bezier(double dt, std::vector<POINT> input) 
{
    std::vector<POINT> output;
    double t = 0;
    while (t <= 1) 
    {
        POINT p;
        double x_sum = 0.0;
        double y_sum = 0.0;
        int i = 0;
        int n = input.size() - 1;
        while (i <= n) 
        {
            double k = factorial(n) / (factorial(i) * factorial(n - i)) * pow(t, i) * pow(1 - t, n - i);
            x_sum += k * input[i].x;
            y_sum += k * input[i].y;
            i++;
        }
        p.x = x_sum;
        p.y = y_sum;
        output.push_back(p);
        t += dt;
    }
    return output;
}

// auto formatDoble2String(double val, int fixed)
// {
//     auto str = std::to_string(val);
//     return str.substr(0, str.find(".") + fixed + 1);
// }

/**
 * @brief 点到直线的距离计算
 *
 * @param a 直线的起点
 * @param b 直线的终点
 * @param p 目标点
 * @return double
 */
// double distanceForPoint2Line(POINT a, POINT b, POINT p)
// {
//     int d = 0; // 距离

//     double ab_distance =
//         sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
//     double ap_distance =
//         sqrt((a.x - p.x) * (a.x - p.x) + (a.y - p.y) * (a.y - p.y));
//     double bp_distance =
//         sqrt((p.x - b.x) * (p.x - b.x) + (p.y - b.y) * (p.y - b.y));

//     double half = (ab_distance + ap_distance + bp_distance) / 2;
//     double area = sqrt(half * (half - ab_distance) * (half - ap_distance) * (half - bp_distance));

//     return (2 * area / ab_distance);
// }

/**
 * @brief 两点之间的距离
 *
 * @param a
 * @param b
 * @return double
 */
double distanceForPoints(POINT a, POINT b)
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}



//////////////////////////////////////////////////////////////////////////自己添加的函数/////////////////////////////////////////////////////////////////////////////////////////////
/* ****************************** 透视变换 ******************************* */
/* ******************* (0:原图 -> 俯视, 1:俯视 -> 原图) ******************** */

// // // 原图 -> 俯视  透视矩阵
// cv::Mat warpMatrix =
//     (cv::Mat_<float>(3, 3) << 4.830985915493104, 7.481690140845294,
//      -559.9154929577662, 1.3374922084896e-14, 14.74647887323986,
//      -725.1690140845323, 7.62235112648736e-17, 0.05070422535211418, 1);

// // 原图 -> 俯视  透视矩阵
////////////////////////////////////////////////////////
// cv::Mat warpMatrix =
//     (cv::Mat_<float>(3, 3) << 6.666700000000000,0,-1.323292000000000e+02,4.331400000000000,1.007600000000000,-87.638600000000000,0.027800000000000,0,0.448500000000000
// );

     
// // 俯视 -> 原图  透视矩阵
// cv::Mat warpMatrixT =
//     (cv::Mat_<float>(3, 3) << 
//     0.067300000000000,0,19.849400000000000,-0.651700000000000,0.992500000000000,1.650900000000000,-0.004200000000000,0,1
// );
///////////////////////////////////////////////////



cv::Mat warpMatrix =
    (cv::Mat_<float>(3, 3) << 7.408700000000000,0,-1.471688000000000e+02,5.036100000000000,1.007600000000000,-1.017330000000000e+02,0.031500000000000,0,0.374400000000000
);

     
// 俯视 -> 原图  透视矩阵
cv::Mat warpMatrixT =
    (cv::Mat_<float>(3, 3) << 
    0.050500000000000,0,19.864400000000000,-0.681800000000000,0.992400000000000,1.680800000000000,-0.004300000000000,0,1
);




     // 透视变换 (0:原图 -> 俯视, 1:俯视 -> 原图)

    //  0.2070   -0.1441   11.3765
    //  -0.0000    0.0194   14.0767
    //  -0.0000   -0.0010    0.2863
  
    //  cv::Mat warpMatrixT =
    //  (cv::Mat_<float>(3, 3) << 0.7231273024969306, -0.503533906399236,
    //   39.74299358711961, 4.254746920719163e-16, 0.06781279847182463,
    //   49.17574021012407, 1.303269874835107e-18, -0.003438395415472779, 1);

// cv::Mat warpMatrix =
//     (cv::Mat_<float>(3, 3) << 33.1, -0.3, -5900.3,-1.8, -22.1, 7893.3,0, 0.8, 1);
     
// // 俯视 -> 原图  透视矩阵
// cv::Mat warpMatrixT =
//     (cv::Mat_<float>(3, 3) << 0.0315,    0.0235,    0.6597,
//     -0.0000,   -0.0002,    1.2455,
//      0.0000,   0.0001,    0.0036);
float inv_rot[3][3]={
    //逆透视变换矩阵，由matlab仿真程序生成
    //0.134300000000000,0,19.789100000000000,-0.540800000000000,0.932900000000000,19.531200000000000,-0.003800000000000,0,1
    0.050500000000000,0,19.864400000000000,-0.681800000000000,0.992400000000000,1.680800000000000,-0.004300000000000,0,1

};

// float inv_rot[3][3]={
//     //逆透视变换矩阵，由matlab仿真程序生成
//     //0.134300000000000,0,19.789100000000000,-0.540800000000000,0.932900000000000,19.531200000000000,-0.003800000000000,0,1
//     0.067300000000000,0,19.849400000000000,-0.651700000000000,0.992500000000000,1.650900000000000,-0.004200000000000,0,1

// };
// float rot[3][3]={
//     //透视变换矩阵，由matlab仿真程序生成
//     //4.758700000000000,0,-94.170600000000000,2.376600000000000,1.072000000000000,-67.966900000000000,0.018200000000000,0,0.639000000000000
//     6.666700000000000,0,-1.323292000000000e+02,4.331400000000000,1.007600000000000,-87.638600000000000,0.027800000000000,0,0.448500000000000

// };

float rot[3][3]={
    //透视变换矩阵，由matlab仿真程序生成
    //4.758700000000000,0,-94.170600000000000,2.376600000000000,1.072000000000000,-67.966900000000000,0.018200000000000,0,0.639000000000000
    7.408700000000000,0,-1.471688000000000e+02,5.036100000000000,1.007600000000000,-1.017330000000000e+02,0.031500000000000,0,0.374400000000000

};



//0.067300000000000,0,19.849400000000000,-0.651700000000000,0.992500000000000,1.650900000000000,-0.004200000000000,0,1

void map_perspective(float x, float y, float loc[2], uint8_t mode) {
    float xx, yy, zz;

    if (mode == 0) {
        // zz = warpMatrix.at<float>(2, 0) * x +
        //      warpMatrix.at<float>(2, 1) * y +
        //      warpMatrix.at<float>(2, 2);
        // xx = (warpMatrix.at<float>(0, 0) * x +
        //       warpMatrix.at<float>(0, 1) * y +
        //       warpMatrix.at<float>(0, 2)) /
        //      zz;
        // yy = (warpMatrix.at<float>(1, 0) * x +
        //       warpMatrix.at<float>(1, 1) * y +
        //       warpMatrix.at<float>(1, 2)) /
        //      zz;

        xx = (rot[1][0]*y+rot[1][1]*x+rot[1][2])/(rot[2][0]*y+rot[2][1]*x+rot[2][2]);
        yy = (rot[0][0]*y+rot[0][1]*x+rot[0][2])/(rot[2][0]*y+rot[2][1]*x+rot[2][2]);
        loc[0] = xx;
        loc[1] = yy;
    } else {
        // zz = warpMatrixT.at<float>(2, 0) * x +
        //      warpMatrixT.at<float>(2, 1) * y +
        //      warpMatrixT.at<float>(2, 2);
        // xx = (warpMatrixT.at<float>(0, 0) * x +
        //       warpMatrixT.at<float>(0, 1) * y +
        //       warpMatrixT.at<float>(0, 2)) /
        //      zz;
        // yy = (warpMatrixT.at<float>(1, 0) * x +
        //       warpMatrixT.at<float>(1, 1) * y +
        //       warpMatrixT.at<float>(1, 2)) /
        //      zz;

        xx = (inv_rot[1][0]*y + inv_rot[1][1]*x + inv_rot[1][2])/(inv_rot[2][0]*y+inv_rot[2][1]*x+1);
        yy = (inv_rot[0][0]*y + inv_rot[0][1]*x + inv_rot[0][2])/(inv_rot[2][0]*y+inv_rot[2][1]*x+1);

        loc[0] = xx;
        loc[1] = yy;
    }
}


//////////////////////////////////////////


float Cal_rot_x(float x,float y){
    //透视变换横坐标变换
    float rot_x;
    rot_x = (rot[1][0]*y+rot[1][1]*x+rot[1][2])/(rot[2][0]*y+rot[2][1]*x+rot[2][2]);
    return rot_x;
}
float Cal_rot_y(float x,float y){
    //透视变换纵坐标变换
    float rot_y;
    rot_y = (rot[0][0]*y+rot[0][1]*x+rot[0][2])/(rot[2][0]*y+rot[2][1]*x+rot[2][2]);
    return rot_y;
}


float Cal_inv_rot_x(float x,float y){
    //逆透视变换横坐标变换
    float inv_rot_x;
    inv_rot_x = (inv_rot[1][0]*y + inv_rot[1][1]*x + inv_rot[1][2])/(inv_rot[2][0]*y+inv_rot[2][1]*x+1);
    return inv_rot_x;


}

float Cal_inv_rot_y(float x,float y){
    //逆透视变换纵坐标变换
    float inv_rot_y;
    inv_rot_y = (inv_rot[0][0]*y + inv_rot[0][1]*x + inv_rot[0][2])/(inv_rot[2][0]*y+inv_rot[2][1]*x+1);
    return inv_rot_y;
}
////////////////////////////////////////////////


// 偏差滑动平均滤波
float filter(float value) 
{
    static float filter_buf[2] = {0};

    //filter_buf[2] = filter_buf[1];
    filter_buf[1] = filter_buf[0];
    filter_buf[0] = value;

    //return (filter_buf[2] + filter_buf[1] + filter_buf[0]) / 3.0f;
    return (filter_buf[1] + filter_buf[0]) / 2.0f;
}

// 点到直线的距离计算
// double dis_point_line(POINT a, POINT b, POINT p) {
//     double ab_distance = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
//     double ap_distance = sqrt((a.x - p.x) * (a.x - p.x) + (a.y - p.y) * (a.y - p.y));
//     double bp_distance = sqrt((p.x - b.x) * (p.x - b.x) + (p.y - b.y) * (p.y - b.y));

//     double half = (ab_distance + ap_distance + bp_distance) / 2;
//     double area = sqrt(half * (half - ab_distance) * (half - ap_distance) * (half - bp_distance));

//     return (2 * area / ab_distance);
// }

float point_dis_line(float a[2],float b[2])
{
    float s = sqrt((a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1]));
    return s;
}


// 两点间的距离计算
double dis_point_point(POINT a, POINT b) {
    double s = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    return s;
}

float fit_line(float pts[][2], int num, int cut_h) 
{
    if (num != 0) 
    {
        std::vector<cv::Point> points;
        cv::Vec4f line_para;
        float k, b, mea = 0.0f;
        float trans[2];
        int y_counter = 0;

        for (int i = 0; i < num; i++, y_counter++) 
        {
            map_perspective(pts[i][0], pts[i][1], trans, 1);
            if (trans[1] < cut_h)
                break;

            points.push_back(cv::Point(trans[0], trans[1]));
        }

        if (points.empty() || y_counter == 0) return 100.0f;

        cv::fitLine(points, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);

        if (fabs(line_para[0]) < 1e-6) return 100.0f;

        k = line_para[1] / line_para[0];
        b = line_para[3] - k * line_para[2];

        for (int i = 0; i < y_counter; i++)
            mea += fabs(k * points[i].x + b - points[i].y);

        return (float)(mea / y_counter);
    }
    return 100.0f;
}



//////////////////////////////////////////////////////////////////////////此为边线处理函数////////////////////////////////////////////////////////////////////
   // 限制范围
int clip(int x, int low, int up) 
{
    return x > up ? up : x < low ? low : x;
}



/* 前进方向定义：
 *   0
 * 3   1
 *   2
 */
const int dir_front[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
const int dir_frontleft[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
const int dir_frontright[4][2] = {{1, -1}, {1, 1}, {-1, 1}, {-1, -1}};
// 左手迷宫巡线
void findline_lefthand_adaptive(cv::Mat img, int block_size, int clip_value, int x, int y, int pts[][2], int *num) 
{
    //assert(num && *num >= 0);
    //assert(block_size > 1 && block_size % 2 == 1);

    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;

    while ((step < *num) && (half <= x) && (x <= img.cols - half - 1) &&  (half <= y) && (y <= img.rows - half - 1) && (turn < 4)) 
    {
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++) 
        {
            for (int dx = -half; dx <= half; dx++) 
            {
                local_thres += AT_IMAGE(img, x + dx, y + dy);
            }
        }
        local_thres /= block_size * block_size;
        local_thres -= clip_value;

        int front_value = AT_IMAGE(img, x + dir_front[dir][0], y + dir_front[dir][1]);
        int frontleft_value = AT_IMAGE(img, x + dir_frontleft[dir][0], y + dir_frontleft[dir][1]);
        if (front_value < local_thres) 
        {
            dir = (dir + 1) % 4;
            turn++;
        } 
        else if (frontleft_value < local_thres) 
        {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        } 
        else 
        {
            x += dir_frontleft[dir][0];
            y += dir_frontleft[dir][1];
            dir = (dir + 3) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
    }
    *num = step;
}

// 右手迷宫巡线
void findline_righthand_adaptive(cv::Mat img, int block_size, int clip_value, int x, int y, int pts[][2], int *num) 
{
    //assert(num && *num >= 0);
    //assert(block_size > 1 && block_size % 2 == 1);

    int half = block_size / 2;
    int step = 0, dir = 0, turn = 0;

    while ((step < *num) && (0 < x) && (x <= img.cols - half - 1) && (half <= y) && (y <= img.rows-half  - 1) && (turn < 4)) 
    {
        int local_thres = 0;
        for (int dy = -half; dy <= half; dy++) 
        {
            for (int dx = -half; dx <= half; dx++) 
            {
                local_thres += AT_IMAGE(img, x + dx, y + dy);
            }
        }
        local_thres /= block_size * block_size;
        local_thres -= clip_value;

        int front_value = AT_IMAGE(img, x + dir_front[dir][0], y + dir_front[dir][1]);
        int frontright_value = AT_IMAGE(img, x + dir_frontright[dir][0], y + dir_frontright[dir][1]);
        if (front_value < local_thres) 
        {
            dir = (dir + 3) % 4;
            turn++;
        } 
        else if (frontright_value < local_thres) 
        {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        } 
        else 
        {
            x += dir_frontright[dir][0];
            y += dir_frontright[dir][1];
            dir = (dir + 1) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
    }
    *num = step;
    //cout<<"右手迷宫"<<endl;
}

// 点集三角滤波
void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel) 
{
    assert(kernel % 2 == 1);

    int half = kernel / 2;
    for (int i = 0; i < num; i++) 
    {
        pts_out[i][0] = pts_out[i][1] = 0;
        for (int j = -half; j <= half; j++) 
        {
            pts_out[i][0] += pts_in[clip(i + j, 0, num - 1)][0] * (half + 1 - abs(j));
            pts_out[i][1] += pts_in[clip(i + j, 0, num - 1)][1] * (half + 1 - abs(j));
        }
        pts_out[i][0] /= (2 * half + 2) * (half + 1) / 2;
        pts_out[i][1] /= (2 * half + 2) * (half + 1) / 2;
    }
}

// 点集等距采样  使走过的采样前折线段的距离为`dist`
void resample_points_old(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist) 
{
    int remain = 0, len = 0;
    for (int i = 0; i < num1 - 1 && len < *num2; i++) 
    {
        float x0 = pts_in[i][0];
        float y0 = pts_in[i][1];
        float dx = pts_in[i + 1][0] - x0;
        float dy = pts_in[i + 1][1] - y0;
        float dn = sqrt(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;

        while (remain < dn && len < *num2) 
        {
            x0 += dx * remain;
            pts_out[len][0] = x0;
            y0 += dy * remain;
            pts_out[len][1] = y0;

            len++;
            dn -= remain;
            remain = dist;
        }
        remain -= dn;
    }
    *num2 = len;
}
void resample_points(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist){
    if (num1 < 0) {
        *num2 = 0;
        return;
    }
    pts_out[0][0] = pts_in[0][0];
    pts_out[0][1] = pts_in[0][1];
    int len = 1;
    for (int i = 0; i < num1 - 1 && len < *num2; i++) {
        float x0 = pts_in[i][0];
        float y0 = pts_in[i][1];
        float x1 = pts_in[i + 1][0];
        float y1 = pts_in[i + 1][1];

        do {
            float x = pts_out[len - 1][0];
            float y = pts_out[len - 1][1];

            float dx0 = x0 - x;
            float dy0 = y0 - y;
            float dx1 = x1 - x;
            float dy1 = y1 - y;

            float dist0 = sqrt(dx0 * dx0 + dy0 * dy0);
            float dist1 = sqrt(dx1 * dx1 + dy1 * dy1);

            float r0 = (dist1 - dist) / (dist1 - dist0);
            float r1 = 1 - r0;

            if (r0 < 0 || r1 < 0) break;
            x0 = x0 * r0 + x1 * r1;
            y0 = y0 * r0 + y1 * r1;
            pts_out[len][0] = x0;
            pts_out[len][1] = y0;
            len++;
        } while (len < *num2);

    }
    *num2 = len;
}
// 点集局部角度变化率
void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist) 
{
    for (int i = 0; i < num; i++) 
    {
        if (i <= 0 || i >= num - 1) 
        {
            angle_out[i] = 0;
            continue;
        }
        float dx1 = pts_in[i][0] - pts_in[clip(i - dist, 0, num - 1)][0];
        float dy1 = pts_in[i][1] - pts_in[clip(i - dist, 0, num - 1)][1];
        float dn1 = sqrtf(dx1 * dx1 + dy1 * dy1);
        float dx2 = pts_in[clip(i + dist, 0, num - 1)][0] - pts_in[i][0];
        float dy2 = pts_in[clip(i + dist, 0, num - 1)][1] - pts_in[i][1];
        float dn2 = sqrtf(dx2 * dx2 + dy2 * dy2);
        float c1 = dx1 / dn1;
        float s1 = dy1 / dn1;
        float c2 = dx2 / dn2;
        float s2 = dy2 / dn2;
        angle_out[i] = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);
    }
}

// 角度变化率非极大抑制
void nms_angle(float angle_in[], int num, float angle_out[], int kernel) 
{
    assert(kernel % 2 == 1);

    int half = kernel / 2;
    for (int i = 0; i < num; i++) 
    {
        angle_out[i] = angle_in[i];
        for (int j = -half; j <= half; j++) 
        {
            if (fabs(angle_in[clip(i + j, 0, num - 1)]) > fabs(angle_out[i])) 
            {
                angle_out[i] = 0;
                break;
            }
        }
    }
}

// 左边线跟踪中线
void track_leftline(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist) 
{

    if(num <= 0 || !pts_in || !pts_out) {
        std::cerr << "Invalid parameters to track_leftline!" << std::endl;
        return;
    }
    for (int i = 0; i < num; i++) 
    {
        float dx = pts_in[clip(i + approx_num, 0, num - 1)][0] -
                   pts_in[clip(i - approx_num, 0, num - 1)][0];
        float dy = pts_in[clip(i + approx_num, 0, num - 1)][1] -
                   pts_in[clip(i - approx_num, 0, num - 1)][1];
        float dn = sqrt(dx * dx + dy * dy);

        dx /= dn;
        dy /= dn;

        pts_out[i][0] = pts_in[i][0] - dy * dist;
        pts_out[i][1] = pts_in[i][1] + dx * dist;
    }
}

// 右边线跟踪中线
void track_rightline(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist) 
{
    for (int i = 0; i < num; i++) 
    {
        float dx = pts_in[clip(i + approx_num, 0, num - 1)][0] -
                   pts_in[clip(i - approx_num, 0, num - 1)][0];
        float dy = pts_in[clip(i + approx_num, 0, num - 1)][1] -
                   pts_in[clip(i - approx_num, 0, num - 1)][1];
        float dn = sqrt(dx * dx + dy * dy);

        dx /= dn;
        dy /= dn;

        pts_out[i][0] = pts_in[i][0] + dy * dist;
        pts_out[i][1] = pts_in[i][1] - dx * dist;
    }
}


//////////////////////////////////////////////////////////摄像头多线程//////////////////////////////////////////////////////////////////////////////////////////////
class CaptureInterface {
    public:
        CaptureInterface(){};  // 构造函数
        ~CaptureInterface(){}; // 析构函数

    private:
        std::shared_ptr<cv::VideoCapture> _capture; // 用于视频捕获的指针
        std::thread _worker_thread;                // 工作线程
        BlockingQueue<cv::Mat> _queue;             // 阻塞队列，用于存储捕获的图像帧
    public:
        bool _loop = false;  // 控制循环的标志
        bool _launch = false; // 未使用的标志（预留）

        // 初始化摄像头
        int init() 
        {
            // 创建 VideoCapture 对象并打开摄像头
            _capture = std::make_shared<cv::VideoCapture>("/dev/video0", cv::CAP_V4L);
            if (!_capture->isOpened()) {
                std::cout << "Create Capture Failed." << std::endl; // 如果打开失败，输出错误信息
            }

            // 设置 MJPG 格式以提高捕获速度
            _capture->set(cv::CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));

            // 设置摄像头参数以获得最佳性能
            _capture->set(cv::CAP_PROP_FRAME_WIDTH, 640);  // 设置帧宽度
            _capture->set(cv::CAP_PROP_FRAME_HEIGHT, 480); // 设置帧高度
            _capture->set(cv::CAP_PROP_FPS, 120);          // 设置帧率


            /////////////不敢改 不知道有没有问题////////////////
            // 设置缓冲区大小
            _capture->set(cv::CAP_PROP_BUFFERSIZE, 3);

            // 获取并输出摄像头参数
            double rate = _capture->get(cv::CAP_PROP_FPS);
            double width = _capture->get(cv::CAP_PROP_FRAME_WIDTH);
            double height = _capture->get(cv::CAP_PROP_FRAME_HEIGHT);
            std::cout << "Camera Param: frame fps = " << rate << " width = " << width << " height = " << height << std::endl;

            return 0; // 返回 0 表示初始化成功
        }

        // 启动捕获线程
        int start() 
        {
            _loop = true; // 设置循环标志为 true
            _worker_thread = std::thread(&CaptureInterface::run, this); // 创建工作线程并运行 run 方法
            return 0;
        }
/////////////////没鸡巴用/////////////////////////////////
        // 停止捕获线程
        int stop() 
        {
            _loop = false; // 设置循环标志为 false
            if (_worker_thread.joinable()) { // 如果线程可以 join，则等待线程结束
                _worker_thread.join();
            }
            _queue.ShutDown(); // 关闭队列
            _capture->release(); // 释放摄像头资源
            return 0;
        }
//////////检测出问题的话 就在这  原始版本   无逻辑提取frame 可能会阻塞影响效率
        // 获取图像帧
        void get(cv::Mat &frame) 
        {
            cv::Mat temp;
            // 尝试从队列中非阻塞地取出一帧
            if (_queue.TryTake(temp)) {
                frame = temp.clone(); // 如果成功，克隆并返回
                return;
            }
            // 如果队列为空，则阻塞等待并取出一帧
            frame = _queue.Take().clone();
        }

        // 捕获线程的运行函数      
// /////////////// 删除了延时函数，不确定是否会影响目标检测//////////////
// /////////////// 目标检测也删了 不知道有没有影响//////////////
        void run() 
        {
            cv::Mat _frame; // 用于存储当前捕获的帧

            while (_loop) 
            { // 循环运行
                if (_capture->read(_frame))
                 {

                    cv::resize(_frame, _frame, cv::Size(COLSIMAGE, ROWSIMAGE)); //300//200




                    //帧率没什么变化
                    if (_queue.Size() > 5)
                        _queue.Take();
                    // 将当前帧放入队列
                    _queue.Put(_frame);
//////////////////////////////////////////////////////////////////////////////////////////////
                } 
                else 
                {
                    std::cout << "UsbCamera read failed." << std::endl; // 如果读取失败，输出错误信息
                }
            }
      
        }



        // int setExposure(double exp) {
        //     if (!_capture || !_capture->isOpened()) {
        //         std::cout << "Camera not opened, cannot set exposure." << std::endl;
        //         return -1;
        //     }
        //     // 先切换到手动曝光
        //     _capture->set(cv::CAP_PROP_AUTO_EXPOSURE, 0.75);
        //     bool ok = _capture->set(cv::CAP_PROP_EXPOSURE, exp);
        //     if (!ok) {
        //         std::cout << "Set exposure failed." << std::endl;
        //         return -1;
        //     }
        //     std::cout << "Set exposure to " << exp << std::endl;
        //     return 0;
        // }
};



void migong2vector(int n0,int ipts0[][2], int ipts1[][2],int n1)
{
    LeftPoint.clear();
    RightPoint.clear();
    for(int i = 0; i < n0; i++ )
    {
        LeftPoint.push_back(Point(ipts0[i][0],ipts0[i][1]));
    }
    for(int i = 0; i < n1; i++ )
    {
        RightPoint.push_back(Point(ipts1[i][0],ipts1[i][1]));
    }


}

void vector2migong(int ipts0[][2], int ipts1[][2]) 
{

    for (int i = 0; i < LeftPoint.size() ; ++i)
    {
        ipts0[i][0] = LeftPoint[i].x;
        ipts0[i][1] = LeftPoint[i].y;
    }
    for (int i = 0; i < RightPoint.size() ; ++i)
    {
        ipts1[i][0] = RightPoint[i].x;
        ipts1[i][1] = RightPoint[i].y;
    }
    // // 如果 points.size() < size，剩余的数组元素可以初始化为默认值
    // for (int i = points.size(); i < size; ++i) {
    //     ipts[i][0] = 0;
    //     ipts[i][1] = 0;
    // }
}


void getwith()
{
    trackwidth.clear();
    int n = getsmall(RightPoint.size(),LeftPoint.size());
    for(int i = 0; i < n; i++ )
    {
        trackwidth.push_back(RightPoint[i].x-LeftPoint[i].x);
    }  
}

vector<Point> interpolatePoints(const Point& p1, const Point& p2, int n) {  //补线函数
    vector<Point> points; 
    // if (n <= 0 || p1 == p2) return points; 
    double dx = static_cast<double>(p2.x - p1.x) / (n + 1);  
  
    for (int i = 0; i <= n; ++i) {  
        points.emplace_back(static_cast<int>(p1.x + i * dx), static_cast<int>(p1.y - i));  
    }  
    return points;  
} 

float my_min(float a ,float b)
{
    if (a<b) return a;
    else return b;
}

float my_max(float a,float b)
{
    if(a>b) return a;
    else return b;
}


void point_Cal_Line(float x1,float y1,float x2,float y2,float outline[][2],int *num)
{
//cout<<"补线"<<endl;
int delta= (int)(x2-x1);
if(delta==0)delta=1;
float k=(y2-y1)/(delta*1.0f);
float b=y1-k*x1;
float min_x=my_min(x1,x2);
float max_x=my_max(x1,x2);
for(int j=(int)min_x;j<(int)max_x;j++)
{
    int i;
    int zhi;
    zhi=k*j+b;

    if(zhi>MT9V03X_H)
    zhi=MT9V03X_H;
    else if(zhi<0)
    zhi=0;
    i=j-min_x;
    outline[i][0]=j;
    outline[i][1]=zhi;
}
*num=(int)max_x-(int)min_x;
}
//第一位为高位

void Splicing_array(float pts_in1[][2], int num1, float pts_in2[][2], int num2,float pts_out[][2], int *num3,int dir)
{
    int inv_num2 = num2-1;

    for(int i=0;i<num1;i++){
        pts_out[i][0] = pts_in1[i][0];
        pts_out[i][1] = pts_in1[i][1];
    }
    if(dir==0)
    {
        for(int i=num1;i<num1+num2;i++){
            pts_out[i][0] = pts_in2[i][0];
            pts_out[i][1] = pts_in2[i][1];
        }
    }
    else{

        for(int i=num1;i<num1+num2;i++){
            pts_out[i][0] = pts_in2[inv_num2][0];
            pts_out[i][1] = pts_in2[inv_num2][1];
            inv_num2--;
            if(inv_num2<=0)
                break;
        }
    }
    *num3 = (num1+num2);
}

void point_Cal_Line_2(float x1,float y1,float x2,float y2,float outline[][2],int *num)
{
    //cout<<"补线"<<endl;
    int delta= (int)(x2-x1);
    if(delta==0)delta=1;
    float k=(y2-y1)/(delta*1.0f);
    float b=y1-k*x1;
    float min_x=my_min(x1,x2);
    float max_x=my_max(x1,x2);
    for(int j=(int)min_x;j<(int)max_x;j++)
    {
        int i;
        int zhi;
        zhi=k*j+b;

        if(zhi>MT9V03X_H)
        zhi=MT9V03X_H;
        else if(zhi<0)
        zhi=0;
        i=max_x-j;
        outline[i][0]=j;
        outline[i][1]=zhi;
    }
    *num=(int)max_x-(int)min_x;
}
int range_limit(int x, int low, int up){
    return x > up ? up : x < low ? low : x;
}



//////////////////////////0 表示画在原图上     1表示画在逆透视上///////////////////////
void drawline(Mat &fame,int num,float matrix[][2],float inv_matrix[][2],int model)
{

if(model ==0)
{ 
    for(int i=0;i<num;i++)
    {
        inv_matrix[i][0] = Cal_inv_rot_x(matrix[i][0],matrix[i][1]);
        inv_matrix[i][1] = Cal_inv_rot_y(matrix[i][0],matrix[i][1]);
        cv::circle(fame, cv::Point((int)inv_matrix[i][0],(int)inv_matrix[i][1]), 1, cv::Scalar(0, 255, 0), -1);
    }
}

else
{
    for(int i=0;i<num;i++)
    {
        cv::circle(fame, cv::Point((int)matrix[i][0],(int)matrix[i][1]), 3, cv::Scalar(0, 255, 0), -1);
    }
}


}



void drawline_purple(Mat &fame,int num,float matrix[][2],float inv_matrix[][2],int model)
{

if(model ==0)
{ 
    for(int i=0;i<num;i++)
    {
        inv_matrix[i][0] = Cal_inv_rot_x(matrix[i][0],matrix[i][1]);
        inv_matrix[i][1] = Cal_inv_rot_y(matrix[i][0],matrix[i][1]);
        cv::circle(fame, cv::Point((int)inv_matrix[i][0],(int)inv_matrix[i][1]), 5, cv::Scalar(128, 0, 128), -1);
    }
}

else
{
    for(int i=0;i<num;i++)
    {
        cv::circle(fame, cv::Point((int)matrix[i][0],(int)matrix[i][1]), 5, cv::Scalar(128, 0, 128), -1);
    }
}


}


void image_draw_rectan(Mat &fame)
{
    //cout<<"黑框"<<endl;
    int i = 0;
    // 绘制左右黑边
    for (i = 0; i < fame.rows; i++)
    {
        // 左边两列
        fame.at<uchar>(i, 0) = 0;
        fame.at<uchar>(i, 1) = 0;
        fame.at<uchar>(i, 2) = 0;
        fame.at<uchar>(i, 3) = 0;
        // 右边两列
        fame.at<uchar>(i, fame.cols - 1) = 0;
        fame.at<uchar>(i, fame.cols - 2) = 0;
        fame.at<uchar>(i, fame.cols - 3) = 0;
        fame.at<uchar>(i, fame.cols - 4) = 0;
    }
    // 绘制上边两行
    for (i = 0; i < fame.cols; i++)
    {
        fame.at<uchar>(0, i) = 0;
        fame.at<uchar>(1, i) = 0;
    }
}


bool check_line_overlap(float line1[][2], float line2[][2], int line1_num, int line2_num) {
    // 确保两边都有足够的点数
    if (line1_num < 5 || line2_num < 5) {
        return false;
    }
    
    int check_points = min(line1_num, line2_num);
    int overlap_count = 0;
    int required_overlaps = (int)(check_points * 0.08f); // 需要30%的点重合
    
    for (int i = 0; i < check_points; i++) {
        float dx = fabs(line1[i][0] - line2[i][0]);
        float dy = fabs(line1[i][1] - line2[i][1]);
        
        if (dx < 10.0f && dy < 2.0f) {
            overlap_count++;
            // 如果已经达到所需的重合点数，直接返回true
            if (overlap_count >= required_overlaps) {
                return true;
            }
        }
    }
    
    return false;
}
bool check_line_overlap_far(float line1[][2], float line2[][2], int line1_num, int line2_num) {
    // 确保两边都有足够的点数
    if (line1_num < 5 || line2_num < 5) {
        return false;
    }
    
    int check_points = min(line1_num, line2_num);
    int overlap_count = 0;
    int required_overlaps = (int)(check_points * 0.03f); // 需要30%的点重合
    
    for (int i = 0; i < check_points; i++) {
        float dx = fabs(line1[i][0] - line2[i][0]);
        float dy = fabs(line1[i][1] - line2[i][1]);
        
        if (dx < 10.0f && dy < 2.0f) {
            overlap_count++;
            // 如果已经达到所需的重合点数，直接返回true
            if (overlap_count >= required_overlaps) {
                return true;
            }
        }
    }
    
    return false;
}


void findline_middle2side(cv::Mat &img, float pts_left[][2], float pts_right[][2], int *num_left, int *num_right)
{
    int start_y = img.rows - 10;  // 从底部往上扫描
    int end_y = 20;              // 扫描到顶部位置
    int mid_x = img.cols / 2;    // 图像中线
    int left_cnt = 0, right_cnt = 0;
    
    // 逐行扫描
    for(int y = start_y; y >= end_y; y--)
    {
        bool found_left = false, found_right = false;
        
        // 从中间向左扫描
        for(int x = mid_x; x > 1; x--)
        {
            // 检测白到黑的跳变点
            if(AT_IMAGE(img, x, y) == 255 && AT_IMAGE(img, x-1, y) == 0)
            {
                pts_left[left_cnt][0] = x;
                pts_left[left_cnt][1] = y;
                left_cnt++;
                found_left = true;
                break;
            }
        }
        
        // 从中间向右扫描
        for(int x = mid_x; x < img.cols-1; x++)
        {
            // 检测白到黑的跳变点
            if(AT_IMAGE(img, x, y) == 255 && AT_IMAGE(img, x+1, y) == 0)
            {
                pts_right[right_cnt][0] = x;
                pts_right[right_cnt][1] = y;
                right_cnt++;
                found_right = true;
                break;
            }
        }
        
        // 如果连续多行没有找到边线点，可以提前退出
        if(!found_left && !found_right)
        {
            if(left_cnt > 5 && right_cnt > 5) // 已经找到足够多的点
            {
                break;
            }
        }
    }
    
    *num_left = left_cnt;
    *num_right = right_cnt;
}