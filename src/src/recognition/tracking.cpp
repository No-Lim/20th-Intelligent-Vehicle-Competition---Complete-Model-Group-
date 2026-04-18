
#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
//#include "crossroad.cpp"
// #include "./_circle.cpp"
//#include "crossroad.cpp"
// #include "./_ramp.cpp"

using namespace cv;
using namespace std;

// extern int cross.far_Lpt0_rpts0s_id;
// extern int cross.far_Lpt1_rpts1s_id;

class Tracking
{

 public:
    Tracking(){};
    ~Tracking(){};

    uint8_t cross_in_route = 0;
    //int16_t max_route = 0,min_route = 0;
    bool have_burger = false, have_ramp = false, have_danger = false;
    bool have_layby = false;
    bool have_battery = false;
    bool have_car = true;


    uint8_t t_people_check = 0;

    float circle_route; //环岛积分

    bool is_line_overlap = false;  // 边线重合标志
    float overlap_threshold = 10.0f;  // 重合判定阈值（像素）
 public:
    vector<POINT> pointsEdgeLeft;     // 赛道左边缘点集
    vector<POINT> pointsEdgeRight;    // 赛道右边缘点集

 public:

    float cx ;
    float cy ;

    bool _is_result = false;  // 是否生成处理后的图像
    int del_id = 0;


    int aim_idx__far = 0;  // 远预锚点位置
    int aim_idx_near = 0;  // 近预锚点位置

    float mea_0 = 0.0f;  // 左直线拟合平均绝对误差
    float mea_1 = 0.0f;  // 右直线拟合平均绝对误差

    std::vector<POINT> bezier_line;  // 中线贝塞尔曲线拟合


    uint16_t speed_diff = 0;  // 减速率

   public:

    cv::Mat mat_bin;  // 原图转二值化图

    int begin_x_l = BEGIN_X;                // 巡线横坐标起始点 左  120
    int begin_x_r = IMAGE_WIDTH - BEGIN_X;  // 巡线横坐标起始点 右  300 - 120
    int begin_y_t = BEGIN_Y;                // 巡线纵坐标起始点  190
    int black_sum = 0;

    int16_t aim_speed = 0;        // 速度量
    int16_t aim_speed_shift = 0;  // 变速计数器
    float aim_angle = 0.0f;       // 偏差量
    float aim_angle_last = 0.0f;  // 偏差量 上一帧
    float aim_sigma = 0.0f;       // 偏差方差
    float aim_distance_f = 0.0f;  // 远锚点
    float aim_distance_n = 0.0f;  // 近锚点

    int element_begin_id = 0;         // 特殊元素中线起始点
    float element_over_route = 0.0f;  // 元素结束路程
    bool element_over = true;         // 元素结束标志
    bool element_no = true;         // 元素结束标志   
    uint8_t element_identify = 0;         // 识别元素

    int img_circlre_flag =0;   //IMG类中保存circle_flag 随便加的

    //int half_pianyi[2];    // 记录half中偏移量

    int left_cone_cnt =0;
    int right_cone_cnt =0;
   public:
    Encoder_t encoder = {0};

   public:
    //bool detection_new = false;
    // 偏差滤波
    float aim_angle_filter;
    float parker_aim_angle_filter;

    int8_t car_stop;

    // L 角点置信度 (角度)
    float Lpt0_found_conf, Lpt1_found_conf;
    // L 角点二次判断
    bool Lpt0_found_last = false, Lpt1_found_last = false;

   public:
    uint8_t ai_number = 0;
    // L 角点
    int Lpt0_rpts0s_id, Lpt1_rpts1s_id;
    bool Lpt0_found, Lpt1_found;
    // 长直道
    bool is_straight0, is_straight1;
    // 弯道 左 右 强制
    bool is_curve0, is_curve1;

   public:
    float l_border[IMAGE_HEIGHT+50] = {0.0};
    float r_border[IMAGE_HEIGHT+50] = {0.0};
    // 原图左右中线数据定义
    int ipts0[IMAGE_HEIGHT+50][2];  // 左: 0
    int ipts1[IMAGE_HEIGHT+50][2];  // 右: 1
    int iptsc[IMAGE_HEIGHT+50];     // 中: c
    int ipts0_num, ipts1_num, rptsc_num;


    // 透视变换后左右中线  滤波: b
    float rpts0b[IMAGE_HEIGHT+50][2];
    float rpts1b[IMAGE_HEIGHT+50][2];
    float rptscb[IMAGE_HEIGHT+50][2];



    // 透视变换后左右中线  等距采样: s
    float rpts0s[IMAGE_HEIGHT+50][2];
    float rpts1s[IMAGE_HEIGHT+50][2];
    float rptscs[IMAGE_HEIGHT+50][2];



    int rpts0s_num, rpts1s_num, rptscs_num;
    // 左右边线局部角度变化率: a
    float rpts0a[IMAGE_HEIGHT+50];
    float rpts1a[IMAGE_HEIGHT+50];
    int rpts0a_num, rpts1a_num;
    // 左右边线局部角度变化率非极大抑制: an
    float rpts0an[IMAGE_HEIGHT+50];
    float rpts1an[IMAGE_HEIGHT+50];
    int rpts0an_num, rpts1an_num;
    // 左右边线偏移中线: c
    float rptsc0[IMAGE_HEIGHT+50][2];
    float rptsc1[IMAGE_HEIGHT+50][2];

    int rptsc0_num, rptsc1_num;

   public:
    std::vector<POINT> edge_det;        // AI元素检测边缘点集
    std::vector<POINT> edge_left;       // 赛道左边缘点集   注意: 此点集 x 与 y 位置相反 !!!
    std::vector<POINT> edge_right;      // 赛道右边缘点集
    std::vector<POINT> last_edge_left;  // 记录上一场边缘点集 (丢失边)
    std::vector<POINT> last_edge_right;
    // 中线
    float (*rpts)[2];
    int rpts_num = 0;
    // 归一化中线
    float rptsn[IMAGE_HEIGHT+50][2];
    int rptsn_num;

    // 透视变换临时变量
    float trans[2];
    int tu0_x, tu0_y, tu1_x, tu1_y;
    




    // 变换后左右中线,原来在下面
    float rpts0[IMAGE_HEIGHT+50][2] ;
    float rpts1[IMAGE_HEIGHT+50][2] ;
    float rptsc[IMAGE_HEIGHT+50][2] ;
    // 原图边线转换数据定义
    int iptsc0[IMAGE_HEIGHT+50] ;
    int iptsc1[IMAGE_HEIGHT+50] ;


    float rightline[MT9V03X_H][2];

float rightline2[MT9V03X_H][2];

int rightline_num;

float Splicing_rightline[MT9V03X_H][2];

int Splicing_rightline_num;

float inv_Splicing_rightline[MT9V03X_H][2];

float Splicing_rightline_s0s[MT9V03X_H][2];

int Splicing_rightline_s0s_num;

float inv_rightline[MT9V03X_H][2];

float Splicing_rightline_center[MT9V03X_H][2];

int Splicing_rightline_center_num;

float leftline[MT9V03X_H][2];

float leftline2[MT9V03X_H][2];

float Splicing_leftline[MT9V03X_H][2];

float Splicing_leftline_s1s[MT9V03X_H][2];

int  Splicing_leftline_s1s_num;

float inv_Splicing_leftline[MT9V03X_H][2];

int  leftline_num;

int  Splicing_leftline_num;

float inv_leftline [MT9V03X_H][2];

float Splicing_leftline_center[MT9V03X_H][2];

int Splicing_leftline_center_num;


float layby_speed=0;

float inv_mix_rightline[MT9V03X_H][2];
float inv_mix_leftline[MT9V03X_H][2];

float mix_rightline[MT9V03X_H][2];
float mix_leftline[MT9V03X_H][2];
int mix_rightline_num;
int mix_leftline_num;

void check_line_overlap() {
    is_line_overlap = false;
    
    // 确保两边都有足够的点数
    if (rpts0s_num < 5 || rpts1s_num < 5) {
        return;
    }
    
    // 计算重合点的数量
    int overlap_count = 0;
    int check_points = min(rpts0s_num, rpts1s_num);
    
    for (int i = 0; i < check_points; i++) {
        // 计算左右边线在同一y坐标下的距离
        float dx = fabs(rpts0s[i][0] - rpts1s[i][0]);
        float dy = fabs(rpts0s[i][1] - rpts1s[i][1]);
        
        // 如果x方向距离小于阈值，且y方向大致相同
        if (dx < overlap_threshold && dy < 2.0f) {
            overlap_count++;
        }
    }
    
    // 如果重合点数量超过总点数的一定比例，认为边线重合
    if (overlap_count > check_points * 0.3) {  // 30%的点重合
        is_line_overlap = true;
    }
}


    void trackRecognition(bool isResearch, uint16_t rowStart);
    //元素后处理
    void trackprocess();
    /**
     * @brief 赛道线识别
     *
     * @param imageBinary 赛道识别基准图像
     */
    void trackRecognition(Mat &imageBinary)
    {
        mat_bin = imageBinary;
        trackRecognition(false, 0);
    }
    

    
};

void Tracking::trackRecognition(bool isResearch, uint16_t rowStart)
{

     //cout<<  "          " <<OSTU_thres<<endl;

     

    begin_y_t = BEGIN_Y;    //190
    if((flag_obstacle==OBSTACLE_MULTIPLE)&&!block_right_ai&&!block_left_ai)
    {
        begin_y_t=150;
    }
    else if(flag_parker==PARKER_END)
    {
        begin_y_t = 120;
    }
    {
        
        //x1,y1落在白点上
        int x1 = begin_x_l, y1 = begin_y_t;
        int hang_left = IMAGE_HEIGHT * 2 / 3;
        if(flag_50)
        {
            y1 = 130;
            hang_left = IMAGE_HEIGHT * 2 / 3 - 50;
        }
        ////////////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////

        // 向左寻找白点
        if (AT_IMAGE(mat_bin, x1, y1) == 0)
            for (x1--; x1 > 0; x1--)
                if (AT_IMAGE(mat_bin, x1, y1) == 255)
                    break;
        // 向左寻找黑点，找不到黑点则x1为0
        for (; x1 > 0; x1--)
            if (AT_IMAGE(mat_bin, x1 - 1, y1) == 0)
                break;
        // 向上寻找黑点
        if (x1 < BLOCK_SIZE / 2)   //x1 < 3，此时需要向上找黑点
        {
            x1 = BLOCK_SIZE / 2;
            for (; y1 > hang_left; y1--)     //从190或140行找到133行
                if (AT_IMAGE(mat_bin, x1, y1 - 1) == 0)
                    break;
        }

        if (AT_IMAGE(mat_bin, x1, y1) == 255) 
        {
            //ipts0_num = y1 + (IMAGE_HEIGHT - BEGIN_Y);   //如果起始行在190行，则为190 + 10 = 200

            if(flag_ramp==RAMP_UP||flag_ramp==RAMP_DOWN)
            {
                ipts0_num = 100;
            }
            else
                ipts0_num = 250;
            findline_lefthand_adaptive(mat_bin, BLOCK_SIZE, CLIP_VALUE, x1, y1, ipts0, &ipts0_num);  //左手迷宫
            begin_x_l = x1 + 50 > IMAGE_WIDTH - 1 ? IMAGE_WIDTH - 1 : x1 + 50;                                     
        } 
        else 
        {
            ipts0_num = 0;
            begin_x_l = BEGIN_X;
        }
    }
    // 原图找右边线 -------------------------------------------------------不是直道的情况下从190行  120向左  180向右
    {
        //x2,y2落在白点上
        int x2 = begin_x_r, y2 = begin_y_t;
        int hang = IMAGE_HEIGHT * 2 / 3;

        if(flag_50)
        {
            y2 = 130;
            hang = IMAGE_HEIGHT * 2 / 3 - 50;
        }
        // 直道从180列向右，特殊130，向右寻找白点  
        if (AT_IMAGE(mat_bin, x2, y2) == 0)  
            for (x2++; x2 < IMAGE_WIDTH - 1; x2++)
                if (AT_IMAGE(mat_bin, x2, y2) == 255)
                    break;
        // 向右寻找黑点
        for (; x2 < IMAGE_WIDTH - 1; x2++)
            if (AT_IMAGE(mat_bin, x2 + 1, y2) ==0)
                break;
        // 向上寻找黑点
        if (x2 > IMAGE_WIDTH - BLOCK_SIZE / 2 - 1)  //x2 > 296
        {
            x2 = IMAGE_WIDTH - BLOCK_SIZE / 2 - 1;
            for (; y2 > hang; y2--)
                if (AT_IMAGE(mat_bin, x2, y2 - 1) == 0)
                    break;
        }
        if (AT_IMAGE(mat_bin, x2, y2) == 255) 
        {
            //ipts1_num = y2 + (IMAGE_HEIGHT - BEGIN_Y);   //直道190 + 10 = 200
            if(flag_ramp==RAMP_UP||flag_ramp==RAMP_DOWN)
            {
                ipts1_num = 100;
            }
            else
                ipts1_num = 250;
            findline_righthand_adaptive(mat_bin, BLOCK_SIZE, CLIP_VALUE, x2, y2, ipts1, &ipts1_num);  //右手迷宫
            begin_x_r = x2 - 50 < 0 ? 0 : x2 - 50;                 
        } 
        else 
        {
            ipts1_num = 0;
            begin_x_r = IMAGE_WIDTH - BEGIN_X;    
        }
    }

    // 原图边线 -> 透视边线 左
    rptsc_num = 0;
    for (int i = 0; i < ipts0_num; i++, rptsc_num++) 
    {
        if (ipts0[i][1] < 10)           //图像最上侧10行不用
            break;
        if(flag_obstacle >= OBSTACLE_NONE)
        {    
            l_border[IMAGE_HEIGHT+50] = {0};          //初始化         
            l_border[ipts0[i][1]] = ipts0[i][0];   //存的原图左边线
        }
        map_perspective(ipts0[i][0], ipts0[i][1], rpts0[i], 0);  
        iptsc0[ipts0[i][1]] = ipts0[i][0];

    }
    //ipts0_num = rptsc_num;
    // 原图边线 -> 透视边线 右
    rptsc_num = 0;
    for (int i = 0; i < ipts1_num; i++, rptsc_num++) 
    {
        if (ipts1[i][1] < 10)
            break;
        if(flag_obstacle >= OBSTACLE_NONE)
        {   
            r_border[IMAGE_HEIGHT+50] = {0};          //初始化
            r_border[ipts1[i][1]] = ipts1[i][0];   //存的原图右边线
        }
        map_perspective(ipts1[i][0], ipts1[i][1], rpts1[i], 0);
        iptsc1[ipts1[i][1]] = ipts1[i][0];

    };
    //ipts1_num = rptsc_num;

    // 中线获取 图像顶部10个像素丢弃
    rptsc_num = 0;
    for (int ccy = IMAGE_HEIGHT - 1; ccy >= 10; ccy--) 
    {
        iptsc[ccy] = iptsc0[ccy] + iptsc1[ccy];

        if (iptsc[ccy] != 0) 
        {
            // if (iptsc1[ccy] == 0)
            //     iptsc[ccy] = (int)((IMAGE_WIDTH - iptsc[ccy]) / 2 + iptsc[ccy]);
            // else
                //iptsc[ccy] = (int)(iptsc[ccy] / 2);

            // 原图中线 -> 透视中线
            map_perspective(iptsc[ccy], ccy, rptsc[rptsc_num++], 0);
        }
    }

    // 滤波
    blur_points(rpts0, ipts0_num, rpts0b, (int)round(LINE_BLUR_KERNEL));
    blur_points(rpts1, ipts1_num, rpts1b, (int)round(LINE_BLUR_KERNEL));
    blur_points(rptsc, rptsc_num, rptscb, (int)round(LINE_BLUR_KERNEL));

    // 边线等距采样
    rpts0s_num = sizeof(rpts0s) / sizeof(rpts0s[0]);
    resample_points(rpts0b, ipts0_num, rpts0s, &rpts0s_num, SAMPLE_DIST * PIXEL_PER_METER);   //0.02 * 205 = 4
    rpts1s_num = sizeof(rpts1s) / sizeof(rpts1s[0]);
    resample_points(rpts1b, ipts1_num, rpts1s, &rpts1s_num, SAMPLE_DIST * PIXEL_PER_METER);
    rptscs_num = sizeof(rptscs) / sizeof(rptscs[0]);
    resample_points(rptscb, rptsc_num, rptscs, &rptscs_num, SAMPLE_DIST * PIXEL_PER_METER);

    // 边线局部角度变化率
    local_angle_points(rpts0s, rpts0s_num, rpts0a, (int)round(ANGLE_DIST / SAMPLE_DIST));   //   0.2/0.02
    rpts0a_num = rpts0s_num;
    local_angle_points(rpts1s, rpts1s_num, rpts1a, (int)round(ANGLE_DIST / SAMPLE_DIST));
    rpts1a_num = rpts1s_num;

    // 角度变化率非极大抑制
    nms_angle(rpts0a, rpts0s_num, rpts0an, (int)round(ANGLE_DIST / SAMPLE_DIST) * 2 + 1);
    rpts0an_num = rpts0a_num;
    nms_angle(rpts1a, rpts1s_num, rpts1an, (int)round(ANGLE_DIST / SAMPLE_DIST) * 2 + 1);
    rpts1an_num = rpts1a_num;

    // 左右中线跟踪
    if(flag_circle == CIRCLE_LEFT_IN)
    {
        track_leftline(rpts0s, rpts0s_num, rptsc0, (int)round(ANGLE_DIST / SAMPLE_DIST), PIXEL_PER_METER * ROAD_WIDTH / 2-3);  //205 * 0.45 / 2 = 46
        rptsc0_num = rpts0s_num;
    }
    else if(flag_circle == CIRCLE_RIGHT_RUNNING)
    {
        track_leftline(rpts0s, rpts0s_num, rptsc0, (int)round(ANGLE_DIST / SAMPLE_DIST), PIXEL_PER_METER * ROAD_WIDTH / 2);  //205 * 0.45 / 2 = 46
        rptsc0_num = rpts0s_num;
    }

    else if(( cone_left)&&(flag_obstacle==OBSTACLE_READY||flag_obstacle==OBSTACLE_MULTIPLE))
    {
        track_leftline(rpts0s, rpts0s_num, rptsc0, (int)round(ANGLE_DIST / SAMPLE_DIST), PIXEL_PER_METER * ROAD_WIDTH / 2 +16 );
        rptsc0_num = rpts0s_num;
    }

   else if((( block_left_ai))&&(flag_obstacle==OBSTACLE_READY||flag_obstacle == OBSTACLE_MULTIPLE))
   {
         track_leftline(rpts0s, rpts0s_num, rptsc0, (int)round(ANGLE_DIST / SAMPLE_DIST), PIXEL_PER_METER * ROAD_WIDTH / 2 +8 );
        rptsc0_num = rpts0s_num;
   }
    else if(layby_right)
    {
        track_leftline(rpts0s, rpts0s_num, rptsc0, (int)round(ANGLE_DIST / SAMPLE_DIST), PIXEL_PER_METER * ROAD_WIDTH / 2 + 17 );
        rptsc0_num = rpts0s_num;
    }
    else if(car_right&&move_flag)
    {
        track_leftline(rpts0s, rpts0s_num, rptsc0, (int)round(ANGLE_DIST / SAMPLE_DIST), PIXEL_PER_METER * ROAD_WIDTH / 2 + 15 );
        rptsc0_num = rpts0s_num;
    }

//汉堡在右
//进入餐饮区，强制内切
    else if(       ( flag_burger == BURGER_START||flag_burger == BURGER_DETECTION )&&burger_right)
    {
        track_leftline(rpts0s, rpts0s_num, rptsc0, (int)round(ANGLE_DIST / SAMPLE_DIST), PIXEL_PER_METER * ROAD_WIDTH / 2 -4);
        rptsc0_num = rpts0s_num;
     
    }
//离开餐饮区，强制外切    
//已经改回来了
    else if(flag_burger == BURGER_END&&burger_right)//汉堡在右, 左线右移
    {
        track_leftline(rpts0s, rpts0s_num, rptsc0, (int)round(ANGLE_DIST / SAMPLE_DIST), PIXEL_PER_METER * ROAD_WIDTH / 2/*+13*/);
        rptsc0_num = rpts0s_num;
    }


    else if(parker_near_moveFlag_L)
    {
        track_leftline(rpts0s, rpts0s_num, rptsc0, (int)round(ANGLE_DIST / SAMPLE_DIST), PIXEL_PER_METER * ROAD_WIDTH / 2 + 7 );
        rptsc0_num = rpts0s_num;
    }

    else 
    {
        track_leftline(rpts0s, rpts0s_num, rptsc0, (int)round(ANGLE_DIST / SAMPLE_DIST), PIXEL_PER_METER * ROAD_WIDTH / 2);
        rptsc0_num = rpts0s_num;
    }
    //右
    if(flag_circle == CIRCLE_RIGHT_IN)
    {
        track_rightline(rpts1s, rpts1s_num, rptsc1, (int)round(ANGLE_DIST / SAMPLE_DIST), PIXEL_PER_METER * ROAD_WIDTH / 2-7);
        rptsc1_num = rpts1s_num;
    }

    else if(flag_circle == CIRCLE_LEFT_RUNNING)
    {
        track_rightline(rpts1s, rpts1s_num, rptsc1, (int)round(ANGLE_DIST / SAMPLE_DIST), PIXEL_PER_METER * ROAD_WIDTH / 2-7);
        rptsc1_num = rpts1s_num;
    }
//汉堡在左
//进入餐饮区，强制内切
//之前是减7
    else if(( flag_burger == BURGER_START||flag_burger == BURGER_DETECTION )   &&burger_left)
    {
        track_rightline(rpts1s, rpts1s_num, rptsc1, (int)round(ANGLE_DIST / SAMPLE_DIST), PIXEL_PER_METER * ROAD_WIDTH / 2-7);
        rptsc1_num = rpts1s_num;
     
    }
//离开餐饮区，强制外切 
//已经改回来了
    else if(flag_burger == BURGER_END&&burger_left)//汉堡在左,右线左移
    {
       // cout<<"进了"<<endl;
        track_rightline(rpts1s, rpts1s_num, rptsc1, (int)round(ANGLE_DIST / SAMPLE_DIST), PIXEL_PER_METER * ROAD_WIDTH / 2/*+13 */);
        rptsc1_num = rpts1s_num;
    }

        else if(( cone_right)&&(flag_obstacle==OBSTACLE_READY||flag_obstacle == OBSTACLE_MULTIPLE))
    {
        track_rightline(rpts1s, rpts1s_num, rptsc1, (int)round(ANGLE_DIST / SAMPLE_DIST), PIXEL_PER_METER * ROAD_WIDTH / 2 +12);
        rptsc1_num = rpts1s_num;
    }
    else if((( block_right_ai))&&(flag_obstacle==OBSTACLE_READY||flag_obstacle == OBSTACLE_MULTIPLE))
    {
        track_rightline(rpts1s, rpts1s_num, rptsc1, (int)round(ANGLE_DIST / SAMPLE_DIST), PIXEL_PER_METER * ROAD_WIDTH / 2 +15);
        rptsc1_num = rpts1s_num;
    }
    else if(car_left&&move_flag)
    {
        track_rightline(rpts1s, rpts1s_num, rptsc1, (int)round(ANGLE_DIST / SAMPLE_DIST), PIXEL_PER_METER * ROAD_WIDTH / 2 +8 );
        rptsc1_num = rpts1s_num;
    }
    else if(layby_left)
    {
        track_rightline(rpts1s, rpts1s_num, rptsc1, (int)round(ANGLE_DIST / SAMPLE_DIST), PIXEL_PER_METER * ROAD_WIDTH / 2 + 12 );
        rptsc1_num = rpts1s_num;
    }
    else if(parker_near_moveFlag_R)
    {
        track_rightline(rpts1s, rpts1s_num, rptsc1, (int)round(ANGLE_DIST / SAMPLE_DIST), PIXEL_PER_METER * ROAD_WIDTH / 2 +1 );
        rptsc1_num = rpts1s_num;
    }


     else
    {
        track_rightline(rpts1s, rpts1s_num, rptsc1, (int)round(ANGLE_DIST / SAMPLE_DIST), PIXEL_PER_METER * ROAD_WIDTH / 2 );
        rptsc1_num = rpts1s_num;
    }
    if(block_qianzhan_flag)
    {
        aim_distance_f = 0.3;
        aim_distance_n = 0.8;
    }
    else //看的不是特别远
    {
        aim_distance_f = 0.75;
        aim_distance_n = 0.25;
        // aim_distance_f = 0.8;
        // aim_distance_n = 0.2;
    }

    // 标志位重置
    is_curve0 = is_curve1 = is_straight0 = is_straight1 =false;
    mea_0 = mea_1 = 10.0f;
    // 左线直线拟合
    if (rpts0s_num > 80) 
    {
        mea_0 = fit_line(rpts0s, rpts0s_num, 50);   //返回绝对平均绝对误差 60~200行
        if (mea_0 < 1.5f && rpts0s_num > 80)
            is_straight0 = true;
        else
            is_curve0 = true;
    }
    // 右线直线拟合
    if (rpts1s_num > 80) 
    {
        mea_1 = fit_line(rpts1s, rpts1s_num, 50);  //返回绝对平均绝对误差  60~200行
        if (mea_1 < 1.5f && rpts1s_num > 80)
            is_straight1 = true;
        else
            is_curve1 = true;
    }

    /* ***************************************************************** */
    /* **************************** 角点检测 **************************** */
    /* ***************************************************************** */

    // 角点重置
    Lpt0_found = Lpt1_found = false;
    // 左线角点
    for (int i = 0; i < rpts0s_num; i++) 
    {
        if (rpts0an[i] == 0)
            continue;
        int im1 = clip(i - (int)round(ANGLE_DIST / SAMPLE_DIST), 0, rpts0s_num - 1);         // im1 = (i - 10)限制在  0 - （rpts0s_num - 1） 
        int ip1 = clip(i + (int)round(ANGLE_DIST / SAMPLE_DIST), 0, rpts0s_num - 1);  
        //cout<<im1<<"            "<<ip1<<endl;       
        float conf = fabs(rpts0a[i]) - (fabs(rpts0a[im1]) + fabs(rpts0a[ip1])) / 2;
        
       //最大角 减去 （左右相邻两个角的和的二分之一）
        // L 角点阈值 0.6981317 < x < 2.44346
        if (Lpt0_found == false && 60. / 180. * PI < conf && conf < 170. / 180. * PI && i < 1.6 / SAMPLE_DIST)   //i < 40   ？？？？？？？？？？？？？？？？？？？？？？？？？？？？？
        {
            Lpt0_found_conf = conf;
            Lpt0_rpts0s_id = i;
            //cout<<"左角点:"<<Lpt0_rpts0s_id<<endl;
            Lpt0_found = true;
            if(conf*180/PI>90)
            {
                angle_check=true;
            }
            //cout<<"左角点角度"<<conf*180/PI<<endl;
            ///////////5.11新加
           // map_perspective(ipts1[i][0], ipts1[i][1], rpts1[i], 0);
        }
        if (Lpt0_found)
            break;
    }
    // 右线角点
    for (int i = 0; i < rpts1s_num; i++) 
    {
        if (rpts1an[i] == 0)
            continue;
        int im1 = clip(i - (int)round(ANGLE_DIST / SAMPLE_DIST), 0, rpts1s_num - 1);
        int ip1 = clip(i + (int)round(ANGLE_DIST / SAMPLE_DIST), 0, rpts1s_num - 1);
        float conf = fabs(rpts1a[i]) - (fabs(rpts1a[im1]) + fabs(rpts1a[ip1])) / 2;
        // L 角点阈值
        if (Lpt1_found == false && 65. / 180. * PI < conf && conf < 140. / 180. * PI && i < 1.6 / SAMPLE_DIST) 
        {
            Lpt1_found_conf = conf;
            Lpt1_rpts1s_id = i;
            //cout<<"右角点:"<<Lpt1_rpts1s_id<<endl;
            //cout<<"右角点角度"<<conf*180/PI<<endl;
            if(conf*180/PI>90)
            {
                angle_check=true;
            }
            Lpt1_found = true;
        }
        if (Lpt1_found)
            break;
    }

    if(Lpt0_found)  //画图
    {
        map_perspective(rpts0s[Lpt0_rpts0s_id][0], rpts0s[Lpt0_rpts0s_id][1], trans, 1);
        tu0_x = trans[0];
        tu0_y = trans[1];
    }
    if(Lpt1_found)  //画图
    {
        map_perspective(rpts1s[Lpt1_rpts1s_id][0], rpts1s[Lpt1_rpts1s_id][1], trans, 1);
        tu1_x = trans[0];
        tu1_y = trans[1];
    }
    if (is_straight0 && is_straight1)     //左右都是直线，  寻中线
        flag_track = TRACK_MIDDLE;
    else if (is_straight0 &&flag_ramp == RAMP_NONE)   //只有左侧是直道，  寻左中线
        flag_track = TRACK_LEFT;
    else if (is_straight1 &&flag_ramp == RAMP_NONE)   //只有右侧是直道，  寻右中线
        flag_track = TRACK_RIGHT;
    else if (is_curve0 && is_curve1 &&aim_angle_filter<-3&&flag_ramp == RAMP_NONE)   //右转弯道  ，寻左中线
        flag_track = TRACK_LEFT;
    else if (is_curve0 && is_curve1 && aim_angle_filter>3&&flag_ramp == RAMP_NONE)    //左转弯道  ，寻右中线
        flag_track = TRACK_RIGHT;
    else if (rpts0s_num == 0 && rpts1s_num != 0 &&flag_ramp == RAMP_NONE )   //左线丢失  ，寻右中线
        flag_track = TRACK_RIGHT;
    else if (rpts0s_num != 0 && rpts1s_num == 0 &&flag_ramp == RAMP_NONE)    //右线丢失  ，寻左中线
        flag_track = TRACK_LEFT;
    else if (rpts0s_num < rpts1s_num / 2 &&flag_ramp == RAMP_NONE)       //左线长度小于右线长度的二分之一，  寻右中线
        flag_track = TRACK_RIGHT;
    else if (rpts0s_num / 2 > rpts1s_num &&flag_ramp == RAMP_NONE)       //右线长度小于左线长度的二分之一，  寻左中线
        flag_track = TRACK_LEFT;
    else if (rpts0s_num < 10 && rpts0s_num < rpts1s_num &&flag_ramp == RAMP_NONE)      //左线点数小于10，且左线点数小于右线点数  ， 寻右中线
        flag_track = TRACK_RIGHT;
    else if (rpts1s_num < 10 && rpts0s_num > rpts1s_num&& flag_ramp == RAMP_NONE)      //右线点数小于10，且右线点数小于左线点数  ， 寻左中线
        flag_track = TRACK_LEFT;
    else                             //剩余情况默认  ， 寻中线
        flag_track = TRACK_MIDDLE;

    /* ***************************************************************** */
    /* *************************** 目标检测结果 ************************** */
    /* ***************************************************************** */  
    if(flag_obstacle <= OBSTACLE_NONE&& flag_parker <= PARKER_DETECTION && flag_ramp <= RAMP_DETECTION && flag_circle == CIRCLE_NONE && flag_cross == CROSS_NONE)
    { 
	    std::vector<PredictResult> _det_result;  // 目标检测结果
		_det_result = com_predictor_results;
        // 遍历检测结果
        for (size_t i = 0; i < _det_result.size(); i++) 
        {
            PredictResult r = _det_result[i];
            // 检测结果筛选
            // if (r.score < _config->threshold_detection)
            // 	continue;

            //防止误判
            if(r.label =="pedestrian"&&r.score<0.60)
            {
                continue;
            }
            else if(r.label =="school"&&r.score<0.60)
            {
                continue;
            }
            else if(r.label=="company"&&r.score<0.65)
            {
                continue;
            }
            else if(r.label == "block"&&r.score<0.65)
            {
                continue;
            }
            else if(r.label == "battery"&&r.score<0.80)
            {
                continue;
            }
            // else if(r.label == "burger"&&r.score<0.6)
            // {
            //     continue;
            // }
            else if (r.label !="pedestrian"&& r.label !="company" && r.label!="block"&&r.label!="battery"&&r.label != "school" &&/*r.label!="burger"&&*/ r.score < 0.80)
                continue;
      

            // 禁止在元素内判断其他元素
            if(flag_obstacle == OBSTACLE_NONE && flag_crosswalk == CROSSWALK_NONE &&flag_layby==LAYBY_NONE&& flag_ramp == RAMP_NONE&&flag_parker == PARKER_NONE) 
            {
                switch (r.type) 
                {
                    case 0: //充电桩
                            if(have_battery/*&&jump_parker*/)
                            {
                                flag_parker = PARKER_DETECTION;
                            }
                            break;


                    case 1:  //BLOCK

                            if(have_danger)
                            {
                                flag_obstacle = OBSTACLE_DETECTION;
                            }
                            break;     
                              
                                // ai_number = 0;
                            // }						
                                                   	        
                    case 2:  // 坡道
                            if(have_ramp&&r.y>50)
                            {
                                flag_ramp = RAMP_DETECTION;  
                            }
                            break;        
                                   
                   
                            
                    case 3:   //汉堡
                            
                            if(have_burger&&r.y>50/*&&jump_burger*/)
                            {
                                flag_burger = BURGER_DETECTION;
                            }
                            break;
                    case 4: //充电车
                            if(have_car)
                            {
                                
                            }
                    case 5:   //公司
                            
                            if(have_layby&&parker_first)
                            {
                                flag_layby = LAYBY_DETECTION;
                            }
                            break;	
                    case 6:   //锥桶
                            if(have_danger)
                            {
                                flag_obstacle = OBSTACLE_DETECTION;
                            }
                            break;
                    case 7:   //斑马线
                            if(1&&(r.width>80)&& (encoder.route > 500))
                            {
                                flag_crosswalk = CROSSWALK_DETECTION;
                                
                            }
                            break;						            

                    break;



                    case 8:   //行人
                    if(have_danger)
                    {
                        flag_obstacle = OBSTACLE_DETECTION;
                    }
                    break;

                    case 9:   //学校
                    if(have_layby)
                    {
                        flag_layby = LAYBY_DETECTION;
                    }
                    break;	

                    default:
                        break;
                }
            }                         
        }
    }    
}







void Tracking::trackprocess()
{  
//         if (flag_track == TRACK_LEFT) 
//         {
//             rpts = rptsc0;
//             rpts_num = rptsc0_num;
//         } 
//         else if (flag_track == TRACK_RIGHT) 
//         {
//             rpts = rptsc1;
//             rpts_num = rptsc1_num;
//         }     
//         else 
//         {
//             rpts = rptscs;
//             rpts_num = rptscs_num;
//         }

/////////////////////////////////////////////4.2 跟踪起点改变/////////////////////////////
    float H_zoom = 0.90f;
    float Half_width = MT9V03X_W/2;
    cx = (rot[1][0]*MT9V03X_H*H_zoom+rot[1][1]*Half_width+rot[1][2])/(rot[2][0]*MT9V03X_H*H_zoom+rot[2][1]*Half_width+rot[2][2]);
    cy = (rot[0][0]*MT9V03X_H*H_zoom+rot[0][1]*Half_width+rot[0][2])/(rot[2][0]*MT9V03X_H*H_zoom+rot[2][1]*Half_width+rot[2][2]);
/////////////////////////////////////////////////////////////////////////////////////////////
    // 找最近点(起始点中线归一化)
    float min_dist = 10000;
    int begin_id = 0;
    bool flag_rpts = false;

    //cout<<"rpts_num     "<<rpts_num <<endl;
    for (int i = 0; i < rpts_num; i++) 
    {
        float dx = rpts[i][0] - cx;
        float dy = rpts[i][1] - cy;
        float dist = sqrt(dx * dx + dy * dy);
        if (dist < min_dist) 
        {
            min_dist = dist;
            begin_id = i;
        }
    }

    begin_id = begin_id + element_begin_id >= rpts_num ? begin_id: begin_id + element_begin_id;
    // 中线有点, 同时最近点不是最后几个点
    //cout<<"rpts_num-begin_id"<<(rpts_num - begin_id)<<endl;
    if (begin_id >= 0 && rpts_num - begin_id >= 7) 
    {
        //cout<<"lose_points"<<lose_points<<endl;
        //清楚标志位
        lose_points = 0;
        // 找到中线
        flag_rpts = true;
        // 归一化中线
        rpts[begin_id][0] = cx;   //(150,199)
        rpts[begin_id][1] = cy;
        rptsn_num = sizeof(rptsn) / sizeof(rptsn[0]);
        //rptsn_num = 1000;

        //cout<<"fuckkk"<<endl;
        resample_points(rpts + begin_id, rpts_num - begin_id, rptsn, &rptsn_num,  SAMPLE_DIST * PIXEL_PER_METER);

        //cout<<"rptsn_num"<<rptsn_num<<endl;
        aim_idx__far = clip(round(aim_distance_f / SAMPLE_DIST), 0, rptsn_num - 1);
        aim_idx_near = clip(round(aim_distance_n / SAMPLE_DIST), 0, rptsn_num - 1);

        std::vector<POINT> v_center(4);  // 三阶贝塞尔曲线

        v_center[0] = {(int)cx, (int)cy};
        v_center[1] = {(int)rptsn[aim_idx_near][0], (int)(IMAGE_HEIGHT * (1 - aim_distance_n))};
        v_center[2] = {(int)rptsn[(int)((aim_idx__far + aim_idx_near) / 2)][0],  (int)(IMAGE_HEIGHT * (1 - (aim_distance_f + aim_distance_n) / 2))};
        v_center[3] = {(int)rptsn[aim_idx__far][0], (int)(IMAGE_HEIGHT * (1 - aim_distance_f))};



        //拟合出来的中线与远近瞄点有关
        //找四个点拟合出来中线
        bezier_line = bezier(0.03, v_center);

        // 计算远锚点偏差值***************************************************************************************/
        float dx;
        float dy;
        
        if (flag_ramp > RAMP_DETECTION)      
        {
            dx = bezier_line[bezier_line.size() / 3].x - cx;  // rptsn[aim_idx__far][0] - cx;
            dy = cy - bezier_line[bezier_line.size() /3].y;  // cy - rptsn[aim_idx__far][1];

        } 
        else if(flag_burger != BURGER_NONE)
        {
            dx = bezier_line[bezier_line.size() *9/ 10].x - cx;  // rptsn[aim_idx__far][0] - cx;
            dy = cy - bezier_line[bezier_line.size() *9/10].y;  // cy - rptsn[aim_idx__far][1];
        }
        else //三分之二处
        {
           dx = bezier_line[bezier_line.size() *  2/3].x - cx;  // rptsn[aim_idx__far][0] - cx;
           dy = cy - bezier_line[bezier_line.size() * 2/3  ].y;  // cy - rptsn[aim_idx__far][1];
        }
        float error_far = (-atan2f(dx, dy) * 180 / PI);

        //assert(!isnan(error_far));
        // // 计算近锚点偏差值************************************************************************************/
        float dx_near;
        float dy_near;
        // dx_near = rptsn[aim_idx_near][0] - cx;
        // dy_near = cy - rptsn[aim_idx_near][1];

        if (flag_ramp > RAMP_DETECTION)      
        {
            dx_near = bezier_line[1].x - cx;  // rptsn[aim_idx_near][0] - cx;
            dy_near = cy - bezier_line[1].y;  // cy - rptsn[aim_idx_near][1];
        } 
        else //四分之一处
        {
            dx_near = bezier_line[bezier_line.size()/4].x - cx;  // rptsn[aim_idx_near][0] - cx;
            dy_near = cy - bezier_line[bezier_line.size()/4].y;  // cy - rptsn[aim_idx_near][1];
        }

        float error_near = (-atan2f(dx_near, dy_near) * 180 / PI);
        aim_angle = error_far * 0.8f + error_near * 0.2f;         //近远点取权重

        if(flag_ramp > RAMP_DETECTION)
        {
            aim_angle = aim_angle/1.5;
        }
        //aim_angle = error_far;
        aim_sigma = sigma(rptsn + aim_idx_near, aim_idx__far - aim_idx_near);   //中线偏差方差
        rpts_num = 0;

    } 
    else 
    {
        if(flag_circle == CIRCLE_NONE && element_over&&flag_cross!=CROSS_HALF&&flag_cross!=CROSS_BEGIN)
        {
            lose_points++;
            //cout<<"lose_points"<<lose_points<<endl;
            if(lose_points >= 80&&stop==false)
            {
                cout<<"停车保护"<<endl;
                flag_protect = PROTECT_STOP; 
            }
        }

        // 中线点过少
        flag_rpts = false;
        aim_angle = aim_angle_last;
        aim_sigma = 100.0f;
        rpts_num = 0;

    else if(parker_right&&(flag_parker == PARKER_OUT))
    {
        aim_angle_filter = -14.5;  
    }
    else if(parker_left&&(flag_parker == PARKER_OUT))
    {
        aim_angle_filter = 14.5;  
    }
    else if(flag_parker==PARKER_FAR_OUT0)
    {
        aim_angle_filter = 0;  
    }
    else if(flag_parker==PARKER_FAR_OUT1&&parker_right)
    {
        aim_angle_filter = -23.5;  
    }
    else if(flag_parker==PARKER_FAR_OUT1&&parker_left)
    {
        aim_angle_filter = 23.5;  
    }
    else if(burger_left)
    {
        aim_angle_filter = aim_angle-2;  
    }
    else if(burger_right)
    {
        aim_angle_filter = aim_angle+2;   
    }
    else
       aim_angle_filter = aim_angle;
        
    aim_angle_last = aim_angle_filter;


}
