/**
 * 智能车竞赛 - 十字路口控制模块
 * 纯代码整理，未改动任何逻辑
 */

 #include <fstream>
 #include <iostream>
 #include <cmath>
 #include <opencv2/highgui.hpp>
 #include <opencv2/opencv.hpp>
 #include "../../include/common.hpp"
 #include "tracking.cpp"
 
 using namespace cv;
 using namespace std;
 
 class Crossroad
 {
 public:
     // 丢线计数器
     int if_lost_left_line = 0;
     int if_lost_right_line = 0;
     int left_lose_count;                    // 左边丢线计数器
     int right_lose_count;                   // 右边丢线计数器
 
     // 角点置信度
     float conf3, conf4;
     float conf3_max, conf4_max;
 
     // 圆环相关标志
     bool rightcir = false;
     bool leftcir = false;
     bool r_upflag = false;
     bool l_upflag = false;
     uint8_t rightpointi;
     uint8_t cirstate = 0;
 
     // 图像中心与巡线点
     int mid = COLSIMAGE / 2;                // 图像中点横坐标
     int lastmid = mid;                      // 巡线的中心
     int endline = cutimgup;                 // 巡线截至行
 
     // 角点坐标
     Point l_down_point, l_up_point, l_mid_point;
     Point r_down_point, r_up_point, r_mid_point;
     Point leftpoint;                        // 巡线点：左
     Point rightpoint;                       // 巡线点：右
     int whitenum = 0;
 
     float dx0[IMAGE_HEIGHT];
     float dx1[IMAGE_HEIGHT] = {0.0};
 
     // ========== 测试变量 ==========
     float trans_cross[2];
     int tu0_x_cross, tu0_y_cross, tu1_x_cross, tu1_y_cross;
     int far_point0[2] = {0, 0};
     int far_point1[2] = {0, 0};
     int far_point0_xie[2] = {0, 0};
     int far_point1_xie[2] = {0, 0};
     int far_point0_ring[2] = {0, 0};
     int far_point1_ring[2] = {0, 0};
     float test_ring1[2] = {0, 0};
 
     // 远线搜索参数
     int far_x1 = 45, far_x2 = 275, far_y1, far_y2;
     int far_x11 = 70, far_x22 = 100;
 
     float test0[2], test1[2];
 
     // 逆透视坐标
     float inv_Lpt0_found[2], inv_Lpt1_found[2];
     float inv_far_Lpt0_found[2], inv_far_Lpt1_found[2];
 
     // 远线角点标志
     bool far_Lpt0_found, far_Lpt1_found;
     int far_Lpt0_rpts0s_id, far_Lpt1_rpts1s_id;
 
     // 远线图像坐标
     int far_ipts0[IMAGE_HEIGHT][2];
     int far_ipts1[IMAGE_HEIGHT][2];
     int far_ipts0_num, far_ipts1_num;
 
     // 远线俯视坐标
     float far_rpts0[IMAGE_HEIGHT][2];
     float far_rpts1[IMAGE_HEIGHT][2];
     int far_rpts0_num, far_rpts1_num;
 
     // 远线滤波后坐标
     float far_rpts0b[IMAGE_HEIGHT][2];
     float far_rpts1b[IMAGE_HEIGHT][2];
     int far_rpts0b_num, far_rpts1b_num;
 
     // 远线等距采样坐标
     float far_rpts0s[IMAGE_HEIGHT][2];
     float far_rpts1s[IMAGE_HEIGHT][2];
     int far_rpts0s_num, far_rpts1s_num;
 
     // 远线角度变化率
     float far_rpts0a[IMAGE_HEIGHT];
     float far_rpts1a[IMAGE_HEIGHT];
     int far_rpts0a_num, far_rpts1a_num;
 
     // 远线角度非极大抑制
     float far_rpts0an[IMAGE_HEIGHT];
     float far_rpts1an[IMAGE_HEIGHT];
     int far_rpts0an_num, far_rpts1an_num;
 
     // 直道标志
     bool is_straight_far_0, is_straight_far_1;
 
     // 十字状态变量
     float cross_route = 0, cout_route = 0;  // 十字编码器积分
     int not_have_line = 0, not_have_L = 0;  // 十字丢线计次
     bool rectangle_flag = false;
     uint8_t out_cout = 0, cross_cout = 0, cross_jishu = 0;
 
     int none_l_line = 0, none_r_line = 0;
     int have_l_line = 0, have_r_line = 0;
 
     bool flag1 = false;
     bool cross_brake = true;
     uint8_t flag_50_route = 0;
 
     bool left_flag = false;
     bool right_flag = false;
     int block_route = 0;
 
     // ========== 函数声明 ==========
     void run_buxian_cross(Tracking &IMG);                           // 补线十字
     void check_cross(Tracking &IMG, Mat &imgBinary);                // 检测十字（none && 左右角点都存在）
     void cross_fill(float l_border[IMAGE_HEIGHT][2], float r_border[IMAGE_HEIGHT][2], Tracking &IMG);
     void run_cross(Tracking &IMG);                                  // 执行十字
     void cross_farline(Tracking &IMG);                              // 寻远线和远线角点
     void cross_farline_R(Tracking &IMG);                            // 只处理右线（half时使用）
     void cross_farline_L(Tracking &IMG);                            // 只处理左线
     float limit_a_b(float value, float min, float max);
     void calculate_s_i(uint16_t start, uint16_t end, float border[IMAGE_HEIGHT][2], float* slope_rate, float* intercept);
     float Slope_Calculate(uint16_t begin, uint16_t end, float border[IMAGE_HEIGHT][2]);
 
     void check_Half(Tracking &IMG);                                 // 斜入十字检测
     bool bai_col(Mat &frame);
     void check_Half_right(Tracking &IMG);
     void check_Half_left(Tracking &IMG);
     void cross_fill_old(float l_border[IMAGE_HEIGHT][2], float r_border[IMAGE_HEIGHT][2], Tracking &IMG);
     void crossdeal(Tracking &IMG);
     void Detectionline(Mat &roi_img, Tracking &IMG);
     void drawimg(string window_name, Mat& frame);
     void cross_farline_old(Tracking &IMG);
     void Double_check_cross_L(Tracking &IMG);
     void Double_check_cross_R(Tracking &IMG);
     void check_block(Tracking &IMG, Mat &imgBingary);
     void run_block(Tracking &IMG);
 };
 
 // ========== 显示函数 ==========
 void Crossroad::drawimg(string window_name, Mat& frame)
 {
     for (uint8_t i = 0; i < LeftPoint.size(); i++)
     {
         circle(frame, LeftPoint[i], 2, Scalar(0, 255, 0), FILLED);
         circle(frame, RightPoint[i], 2, Scalar(0, 255, 0), FILLED);
         circle(frame, Point((LeftPoint[i].x + RightPoint[i].x) / 2, LeftPoint[i].y), 2, Scalar(255, 0, 0), FILLED);
     }
     circle(frame, l_down_point, 3, Scalar(0, 0, 255), FILLED);
     circle(frame, l_up_point, 3, Scalar(0, 0, 255), FILLED);
     circle(frame, r_down_point, 3, Scalar(0, 0, 255), FILLED);
     circle(frame, r_up_point, 3, Scalar(0, 0, 255), FILLED);
     if (r_upflag || l_upflag)
     {
         putText(frame, "crossing", Point(80, 30), FONT_HERSHEY_PLAIN, 2, Scalar(255, 0, 0), 1);
     }
     imshow(window_name, frame);
 }
 
 // ========== 正入十字检测 ==========
 void Crossroad::check_cross(Tracking &IMG, Mat &imgBinary)
 {
     // 检测到双角点
     if (flag_cross == CROSS_NONE && (IMG.Lpt0_found && IMG.Lpt1_found))
     {
         IMG.element_identify = 1;
         flag_cross = CROSS_BEGIN;
         cout << "CROSS_IN:正入十字" << endl;
         cross_route = IMG.encoder.route;
         IMG.element_over = false;
     }
 }
 
 // ========== 二次检测（转弯接十字补救） ==========
 void Crossroad::Double_check_cross_R(Tracking &IMG)
 {
     cross_farline_R(IMG);
 }
 
 void Crossroad::Double_check_cross_L(Tracking &IMG)
 {
     cross_farline_L(IMG);
 }
 
 // ========== 十字执行状态机 ==========
 void Crossroad::run_cross(Tracking &IMG)
 {
     if (flag_cross == CROSS_HALF)
     {
         // 左半十字
         if (Lpt0_found_flag)
         {
             cross_farline_L(IMG);
             flag_track = TRACK_LEFT;
             if (IMG.rpts0s_num < 5)
             {
                 not_have_line++;
             }
             if ((not_have_line > 1 && IMG.rpts1s_num > 10 && IMG.rpts0s_num > 10) &&
                 (IMG.encoder.route - cross_route >= 30))
             {
                 flag_cross = CROSS_NONE;
                 if_lost_left_line = 0;
                 not_have_line = 0;
                 Lpt0_found_flag = 0;
                 Lpt1_found_flag = 0;
                 IMG.element_over = true;
                 cout << "左十字结束" << endl;
             }
         }
         
         // 右半十字
         if (Lpt1_found_flag)
         {
             cross_farline_R(IMG);
             flag_track = TRACK_RIGHT;
             if (IMG.rpts1s_num < 5)
             {
                 not_have_line++;
             }
             if ((not_have_line > 1 && IMG.rpts0s_num > 10 && IMG.rpts1s_num > 10) &&
                 (IMG.encoder.route - cross_route >= 30))
             {
                 flag_cross = CROSS_NONE;
                 if_lost_right_line = 0;
                 not_have_line = 0;
                 Lpt0_found_flag = 0;
                 Lpt1_found_flag = 0;
                 IMG.element_over = true;
                 cout << "右十字结束" << endl;
             }
         }
     }
 }
 
 // ========== 左远线搜索 ==========
 void Crossroad::cross_farline_L(Tracking &IMG)
 {
     // 根据当前状态确定搜索起点
     if (IMG.Lpt0_found && !if_lost_left_line && !IMG.Lpt1_found)
     {
         int idx = clip(IMG.Lpt0_rpts0s_id, 0, IMG.rpts0s_num - 1);
         if (IMG.rpts0s_num > 0 && idx >= 0 && idx < IMG.rpts0s_num)
         {
             inv_Lpt0_found[0] = Cal_inv_rot_x(IMG.rpts0s[idx][0] - 1, IMG.rpts0s[idx][1]);
             inv_Lpt0_found[1] = Cal_inv_rot_y(IMG.rpts0s[idx][0], IMG.rpts0s[idx][1]) - 14;
             IMG.rptsc0_num = IMG.rpts0s_num = std::max(0, IMG.Lpt0_rpts0s_id - 1);
         }
     }
     else if (IMG.Lpt0_found && IMG.rpts0s_num >= 10 && !if_lost_left_line && IMG.Lpt1_found)
     {
         int idx = clip(IMG.Lpt0_rpts0s_id, 0, IMG.rpts0s_num - 1);
         if (IMG.rpts0s_num > 0 && idx >= 0 && idx < IMG.rpts0s_num)
         {
             inv_Lpt0_found[0] = Cal_inv_rot_x(IMG.rpts0s[idx][0] - 5, IMG.rpts0s[idx][1]);
             inv_Lpt0_found[1] = Cal_inv_rot_y(IMG.rpts0s[idx][0], IMG.rpts0s[idx][1]) - 24;
             IMG.rptsc0_num = IMG.rpts0s_num = std::max(0, IMG.Lpt0_rpts0s_id - 1);
         }
     }
     else if (flag_circle == CIRCLE_LEFT_IN || flag_circle == CIRCLE_RIGHT_OUT || flag_circle == CIRCLE_RIGHT_END)
     {
         // 圆环阶段处理
         if (flag_circle == CIRCLE_LEFT_IN)
         {
             if (IMG.ipts0_num > 5 && !if_lost_left_line)
             {
                 inv_Lpt0_found[0] = IMG.ipts0[IMG.ipts0_num - 3][0] + 68;
                 inv_Lpt0_found[1] = IMG.ipts0[IMG.ipts0_num - 3][1] - 5;
                 test0[0] = inv_Lpt0_found[0];
                 test0[1] = inv_Lpt0_found[1];
             }
             else
             {
                 if_lost_left_line = 1;
                 inv_Lpt0_found[0] = 40;
                 inv_Lpt0_found[1] = begin_y * 0.85;
                 far_point0_ring[0] = inv_Lpt0_found[0];
                 far_point0_ring[1] = inv_Lpt0_found[1];
             }
         }
         else
         {
             inv_Lpt0_found[0] = 150;
             inv_Lpt0_found[1] = begin_y * 0.85;
         }
     }
     else if (parker_left && move_flag)
     {
         if (IMG.Lpt0_found)
         {
             inv_Lpt0_found[0] = Cal_inv_rot_x(IMG.rpts0s[clip(IMG.Lpt0_rpts0s_id, 0, IMG.rpts0s_num - 1)][0] - 2,
                                               IMG.rpts0s[clip(IMG.Lpt0_rpts0s_id, 0, IMG.rpts0s_num - 1)][1]);
             inv_Lpt0_found[1] = Cal_inv_rot_y(IMG.rpts0s[clip(IMG.Lpt0_rpts0s_id, 0, IMG.rpts0s_num - 1)][0],
                                               IMG.rpts0s[clip(IMG.Lpt0_rpts0s_id, 0, IMG.rpts0s_num - 1)][1]) - 20;
             IMG.rptsc0_num = IMG.rpts0s_num = IMG.Lpt0_rpts0s_id;
         }
         else
         {
             inv_Lpt0_found[0] = 20;
             inv_Lpt0_found[1] = 150;
             IMG.rptsc0_num = IMG.rpts0s_num = IMG.Lpt0_rpts0s_id;
         }
     }
     else if (IMG.rpts0s_num < 2 && !move_flag)
     {
         if_lost_left_line = 1;
         inv_Lpt0_found[0] = 95;
         inv_Lpt0_found[1] = 130;
     }
 
     // 处理右线截断
     if (IMG.Lpt1_found && IMG.Lpt1_rpts1s_id > 1)
     {
         IMG.rptsc1_num = IMG.rpts1s_num = std::max(0, IMG.Lpt1_rpts1s_id - 2);
     }
 
     // 向上搜索边线起点
     int cross_width = 4;
     far_y1 = 0, far_y2 = 0;
     int y1 = inv_Lpt0_found[1];
     far_x11 = inv_Lpt0_found[0];
     bool white_found = false;
     far_ipts0_num = sizeof(far_ipts0) / sizeof(far_ipts0[0]);
     int local_thres_left_up = 0;
 
     far_point0_xie[0] = far_x11;
     far_point0_xie[1] = y1;
 
     // 先找黑边
     for (; y1 > block_size / 2; y1--)
     {
         if (AT_IMAGE(IMG.mat_bin, far_x11, y1 - 1) < OSTU_thres)
         {
             break;
         }
     }
 
     // 自适应二值化向上搜索
     for (; y1 > block_size / 2; y1--)
     {
         for (int dy = -block_size / 2; dy <= block_size / 2; dy++)
         {
             for (int dx = -block_size / 2; dx <= block_size / 2; dx++)
             {
                 local_thres_left_up += AT_IMAGE(IMG.mat_bin, far_x11 + dx, y1 + dy);
             }
         }
         local_thres_left_up /= block_size * block_size;
         local_thres_left_up -= clip_value;
 
         if (AT_IMAGE(IMG.mat_bin, far_x11, y1 - 1) < local_thres_left_up)
         {
             far_y1 = y1;
             break;
         }
     }
 
     // 从起点开始寻找远端边线
     if (AT_IMAGE(IMG.mat_bin, far_x11, far_y1 - 1) < local_thres_left_up)
     {
         findline_lefthand_adaptive(IMG.mat_bin, block_size, clip_value, far_x11, far_y1, far_ipts0, &far_ipts0_num);
     }
     else
     {
         far_ipts0_num = 0;
     }
 
     // 透视变换
     for (int i = 0; i < far_ipts0_num; i++)
     {
         map_perspective(far_ipts0[i][0], far_ipts0[i][1], far_rpts0[i], 0);
     }
     far_rpts0_num = far_ipts0_num;
 
     // 边线滤波
     blur_points(far_rpts0, far_rpts0_num, far_rpts0b, (int)round(line_blur_kernel));
     far_rpts0b_num = far_rpts0_num;
 
     // 边线等距采样
     far_rpts0s_num = sizeof(far_rpts0s) / sizeof(far_rpts0s[0]);
     resample_points(far_rpts0b, far_rpts0b_num, far_rpts0s, &far_rpts0s_num, sample_dist * pixel_per_meter);
 
     // 边线局部角度变化率
     local_angle_points(far_rpts0s, far_rpts0s_num, far_rpts0a, (int)round(angle_dist / sample_dist));
     far_rpts0a_num = far_rpts0s_num;
 
     // 角度变化率非极大抑制
     nms_angle(far_rpts0a, far_rpts0a_num, far_rpts0an, (int)round(angle_dist / sample_dist) * 2 + 1);
     far_rpts0an_num = far_rpts0a_num;
 
     // 找远线上的L角点
     far_Lpt0_found = far_Lpt1_found = false;
     for (int i = 0; i < far_rpts0s_num; i++)
     {
         if (far_rpts0an[i] == 0) continue;
         int im1 = clip(i - (int)round(angle_dist / sample_dist), 0, far_rpts0s_num - 1);
         int ip1 = clip(i + (int)round(angle_dist / sample_dist), 0, far_rpts0s_num - 1);
         conf3 = fabs(far_rpts0a[i]) - (fabs(far_rpts0a[im1]) + fabs(far_rpts0a[ip1])) / 2;
         if (30. / 180. * PI < conf3 && conf3 < 120. / 180. * PI && i < 100 &&
             far_rpts0s[i][0] <= far_rpts0s[ip1][0] && i > 1 &&
             far_rpts0s[i][1] > far_rpts0s[ip1][1] && far_rpts0s[im1][0] < far_rpts0s[ip1][0])
         {
             far_Lpt0_rpts0s_id = i;
             far_Lpt0_found = true;
             inv_far_Lpt0_found[0] = Cal_inv_rot_x(far_rpts0s[clip(far_Lpt0_rpts0s_id, 0, far_rpts0s_num - 1)][0],
                                                   far_rpts0s[clip(far_Lpt0_rpts0s_id, 0, far_rpts0s_num - 1)][1]);
             inv_far_Lpt0_found[1] = Cal_inv_rot_y(far_rpts0s[clip(far_Lpt0_rpts0s_id, 0, far_rpts0s_num - 1)][0],
                                                   far_rpts0s[clip(far_Lpt0_rpts0s_id, 0, far_rpts0s_num - 1)][1]);
             break;
         }
         if (conf3 > conf3_max) conf3_max = conf3;
     }
 
     if (far_Lpt0_found)
     {
         map_perspective(far_rpts0s[far_Lpt0_rpts0s_id][0], far_rpts0s[far_Lpt0_rpts0s_id][1], trans_cross, 1);
         tu0_x_cross = trans_cross[0];
         tu0_y_cross = trans_cross[1];
     }
 }
 
 // ========== 右远线搜索 ==========
 void Crossroad::cross_farline_R(Tracking &IMG)
 {
     // 处理左线截断
     if (IMG.Lpt0_found)
     {
         IMG.rptsc0_num = IMG.rpts0s_num = std::max(0, IMG.Lpt0_rpts0s_id - 2);
     }
 
     // 根据当前状态确定搜索起点
     if (IMG.Lpt1_found && IMG.Lpt1_rpts1s_id > 2 && !if_lost_right_line && !move_flag)
     {
         int idx = clip(IMG.Lpt1_rpts1s_id, 0, IMG.rpts1s_num - 1);
         if (IMG.rpts1s_num > 0 && idx >= 0 && idx < IMG.rpts1s_num)
         {
             inv_Lpt1_found[0] = Cal_inv_rot_x(IMG.rpts1s[idx][0], IMG.rpts1s[idx][1]);
             inv_Lpt1_found[1] = Cal_inv_rot_y(IMG.rpts1s[idx][0], IMG.rpts1s[idx][1]) - 20;
             IMG.rptsc1_num = IMG.rpts1s_num = IMG.Lpt1_rpts1s_id;
         }
     }
     else if ((flag_circle == CIRCLE_RIGHT_IN || flag_circle == CIRCLE_LEFT_OUT || flag_circle == CIRCLE_LEFT_END))
     {
         // 圆环阶段处理
         if (flag_circle == CIRCLE_RIGHT_IN)
         {
             if (IMG.ipts1_num > 12 && !if_lost_right_line)
             {
                 inv_Lpt1_found[0] = IMG.ipts1[IMG.ipts1_num - 3][0] - 20;
                 inv_Lpt1_found[1] = IMG.ipts1[IMG.ipts1_num - 3][1] - 1;
                 test1[0] = inv_Lpt1_found[0];
                 test1[1] = inv_Lpt1_found[1];
             }
             else
             {
                 if_lost_right_line = 1;
                 inv_Lpt1_found[0] = 240;
                 inv_Lpt1_found[1] = begin_y * 0.85;
                 far_point1_ring[0] = inv_Lpt1_found[0];
                 far_point1_ring[1] = inv_Lpt1_found[1];
             }
         }
         else
         {
             inv_Lpt1_found[0] = 60;
             inv_Lpt1_found[1] = begin_y * 0.85;
             far_point1_ring[0] = inv_Lpt1_found[0];
             far_point1_ring[1] = inv_Lpt1_found[1];
         }
     }
     else if (parker_right && move_flag)
     {
         if (IMG.Lpt1_found)
         {
             inv_Lpt1_found[0] = Cal_inv_rot_x(IMG.rpts1s[clip(IMG.Lpt1_rpts1s_id, 0, IMG.rpts1s_num - 1)][0] + 2,
                                               IMG.rpts1s[clip(IMG.Lpt1_rpts1s_id, 0, IMG.rpts1s_num - 1)][1]);
             inv_Lpt1_found[1] = Cal_inv_rot_y(IMG.rpts1s[clip(IMG.Lpt1_rpts1s_id, 0, IMG.rpts1s_num - 1)][0],
                                               IMG.rpts1s[clip(IMG.Lpt1_rpts1s_id, 0, IMG.rpts1s_num - 1)][1]) - 20;
             IMG.rptsc1_num = IMG.rpts1s_num = IMG.Lpt1_rpts1s_id;
         }
         else
         {
             inv_Lpt1_found[0] = 250;
             inv_Lpt1_found[1] = 120;
             IMG.rptsc1_num = IMG.rpts1s_num = IMG.Lpt1_rpts1s_id;
         }
     }
     else if (IMG.rpts1s_num < 2)
     {
         if_lost_right_line = 1;
         inv_Lpt1_found[0] = 270;
         inv_Lpt1_found[1] = 80;
     }
 
     // 向上搜索边线起点
     int cross_width = 4;
     far_y1 = 0, far_y2 = 0;
     int y1 = inv_Lpt1_found[1];
     far_x11 = inv_Lpt1_found[0];
     bool white_found = false;
     far_ipts1_num = sizeof(far_ipts1) / sizeof(far_ipts1[0]);
     int local_thres_right_up;
 
     far_point1_xie[0] = inv_Lpt1_found[0];
     far_point1_xie[1] = inv_Lpt1_found[1];
 
     // 先找黑边
     for (; y1 > block_size / 2; y1--)
     {
         if (AT_IMAGE(IMG.mat_bin, far_x11, y1 - 1) < OSTU_thres)
         {
             far_y1 = y1;
             break;
         }
     }
 
     test_ring1[0] = far_x11;
     test_ring1[1] = far_y1;
 
     // 从找到角点位置开始寻找
     if (AT_IMAGE(IMG.mat_bin, far_x11, far_y1 + 1) >= thres)
     {
         findline_righthand_adaptive(IMG.mat_bin, 7, CLIP_VALUE, far_x11, far_y1, far_ipts1, &far_ipts1_num);
     }
     else
     {
         far_ipts1_num = 0;
     }
 
     // 透视变换
     for (int i = 0; i < far_ipts1_num; i++)
     {
         map_perspective(far_ipts1[i][0], far_ipts1[i][1], far_rpts1[i], 0);
     }
     far_rpts1_num = far_ipts1_num;
 
     // 边线滤波
     blur_points(far_rpts1, far_rpts1_num, far_rpts1b, (int)round(line_blur_kernel));
     far_rpts1b_num = far_rpts1_num;
 
     // 边线等距采样
     far_rpts1s_num = sizeof(far_rpts1s) / sizeof(far_rpts1s[0]);
     resample_points(far_rpts1b, far_rpts1b_num, far_rpts1s, &far_rpts1s_num, sample_dist * pixel_per_meter);
 
     // 边线局部角度变化率
     local_angle_points(far_rpts1s, far_rpts1s_num, far_rpts1a, (int)round(angle_dist / sample_dist));
     far_rpts1a_num = far_rpts1s_num;
 
     // 角度变化率非极大抑制
     nms_angle(far_rpts1a, far_rpts1a_num, far_rpts1an, (int)round(angle_dist / sample_dist) * 2 + 1);
     far_rpts1an_num = far_rpts1a_num;
 
     // 找远线上的L角点
     far_Lpt1_found = far_Lpt0_found = false;
     for (int i = 0; i < far_rpts1s_num; i++)
     {
         if (far_rpts1an[i] == 0) continue;
         int im1 = clip(i - (int)round(angle_dist / sample_dist), 0, far_rpts1s_num - 1);
         int ip1 = clip(i + (int)round(angle_dist / sample_dist), 0, far_rpts1s_num - 1);
         conf4 = fabs(far_rpts1a[i]) - (fabs(far_rpts1a[im1]) + fabs(far_rpts1a[ip1])) / 2;
         if (30. / 180. * PI < conf4 && conf4 < 170. / 180. * PI && i < 100 &&
             far_rpts1s[i][0] >= far_rpts1s[ip1][0] && i > 1 &&
             far_rpts1s[i][1] >= far_rpts1s[ip1][1] && far_rpts1s[im1][0] >= far_rpts1s[ip1][0])
         {
             far_Lpt1_rpts1s_id = i;
             far_Lpt1_found = true;
             inv_far_Lpt1_found[0] = Cal_inv_rot_x(far_rpts1s[clip(far_Lpt1_rpts1s_id, 0, far_rpts1s_num - 1)][0],
                                                   far_rpts1s[clip(far_Lpt1_rpts1s_id, 0, far_rpts1s_num - 1)][1]);
             inv_far_Lpt1_found[1] = Cal_inv_rot_y(far_rpts1s[clip(far_Lpt1_rpts1s_id, 0, far_rpts1s_num - 1)][0],
                                                   far_rpts1s[clip(far_Lpt1_rpts1s_id, 0, far_rpts1s_num - 1)][1]);
             break;
         }
         if (conf4 > conf4_max) conf4_max = conf4;
     }
 
     if (far_Lpt1_found)
     {
         map_perspective(far_rpts1s[far_Lpt1_rpts1s_id][0], far_rpts1s[far_Lpt1_rpts1s_id][1], trans_cross, 1);
         tu1_x_cross = trans_cross[0];
         tu1_y_cross = trans_cross[1];
     }
 }
 
 // ========== 远线搜索（双线版本） ==========
 void Crossroad::cross_farline(Tracking &IMG)
 {
     // 确定搜索起点
     if (IMG.Lpt1_found && IMG.Lpt1_rpts1s_id > 2 && !if_lost_right_line)
     {
         inv_Lpt1_found[0] = Cal_inv_rot_x(IMG.rpts1s[clip(IMG.Lpt1_rpts1s_id, 0, IMG.rpts1s_num - 1)][0] + 14,
                                           IMG.rpts1s[clip(IMG.Lpt1_rpts1s_id, 0, IMG.rpts1s_num - 1)][1]);
         inv_Lpt1_found[1] = Cal_inv_rot_y(IMG.rpts1s[clip(IMG.Lpt1_rpts1s_id, 0, IMG.rpts1s_num - 1)][0],
                                           IMG.rpts1s[clip(IMG.Lpt1_rpts1s_id, 0, IMG.rpts1s_num - 1)][1]) - 9;
         IMG.rptsc1_num = IMG.rpts1s_num = IMG.Lpt1_rpts1s_id;
     }
     else
     {
         inv_Lpt1_found[0] = 260;
         inv_Lpt1_found[1] = begin_y * 0.85;
     }
 
     // 向上搜索边线起点
     int cross_width = 4;
     far_y1 = 0, far_y2 = 0;
     int y1 = inv_Lpt1_found[1];
     far_x11 = inv_Lpt1_found[0];
     bool white_found = false;
     far_ipts1_num = sizeof(far_ipts1) / sizeof(far_ipts1[0]);
     int local_thres_right_up;
 
     far_point1_xie[0] = inv_Lpt1_found[0];
     far_point1_xie[1] = inv_Lpt1_found[1];
 
     // 先找黑边
     for (; y1 > block_size / 2; y1--)
     {
         if (AT_IMAGE(IMG.mat_bin, far_x11, y1 - 1) < OSTU_thres)
         {
             far_y1 = y1;
             break;
         }
     }
 
     test_ring1[0] = far_x11;
     test_ring1[1] = far_y1;
 
     // 从找到角点位置开始寻找
     if (AT_IMAGE(IMG.mat_bin, far_x11, far_y1 + 1) >= thres)
     {
         findline_righthand_adaptive(IMG.mat_bin, 7, CLIP_VALUE, far_x11, far_y1, far_ipts1, &far_ipts1_num);
     }
     else
     {
         far_ipts1_num = 0;
     }
 
     // 透视变换
     for (int i = 0; i < far_ipts1_num; i++)
     {
         map_perspective(far_ipts1[i][0], far_ipts1[i][1], far_rpts1[i], 0);
     }
     far_rpts1_num = far_ipts1_num;
 
     // 边线滤波
     blur_points(far_rpts1, far_rpts1_num, far_rpts1b, (int)round(line_blur_kernel));
     far_rpts1b_num = far_rpts1_num;
 
     // 边线等距采样
     far_rpts1s_num = sizeof(far_rpts1s) / sizeof(far_rpts1s[0]);
     resample_points(far_rpts1b, far_rpts1b_num, far_rpts1s, &far_rpts1s_num, sample_dist * pixel_per_meter);
 
     // 边线局部角度变化率
     local_angle_points(far_rpts1s, far_rpts1s_num, far_rpts1a, (int)round(angle_dist / sample_dist));
     far_rpts1a_num = far_rpts1s_num;
 
     // 角度变化率非极大抑制
     nms_angle(far_rpts1a, far_rpts1a_num, far_rpts1an, (int)round(angle_dist / sample_dist) * 2 + 1);
     far_rpts1an_num = far_rpts1a_num;
 
     // 找远线上的L角点
     far_Lpt0_found = far_Lpt1_found = false;
     for (int i = 0; i < far_rpts1s_num; i++)
     {
         if (far_rpts1an[i] == 0) continue;
         int im1 = clip(i - (int)round(angle_dist / sample_dist), 0, far_rpts1s_num - 1);
         int ip1 = clip(i + (int)round(angle_dist / sample_dist), 0, far_rpts1s_num - 1);
         float conf = fabs(far_rpts1a[i]) - (fabs(far_rpts1a[im1]) + fabs(far_rpts1a[ip1])) / 2;
         if (30. / 180. * PI < conf && conf < 140. / 180. * PI && i < 50)
         {
             far_Lpt1_rpts1s_id = i;
             far_Lpt1_found = true;
             break;
         }
     }
 
     if (far_Lpt0_found)
     {
         map_perspective(far_rpts0s[far_Lpt0_rpts0s_id][0], far_rpts0s[far_Lpt0_rpts0s_id][1], trans_cross, 1);
         tu0_x_cross = trans_cross[0];
         tu0_y_cross = trans_cross[1];
     }
     if (far_Lpt1_found)
     {
         map_perspective(far_rpts1s[far_Lpt1_rpts1s_id][0], far_rpts1s[far_Lpt1_rpts1s_id][1], trans_cross, 1);
         tu1_x_cross = trans_cross[0];
         tu1_y_cross = trans_cross[1];
     }
 }
 
 // ========== 斜入十字检测入口 ==========
 void Crossroad::check_Half(Tracking &IMG)
 {
     if (IMG.Lpt1_found && IMG.rpts1s_num > IMG.rpts0s_num) check_Half_right(IMG);
     if (IMG.Lpt0_found && IMG.rpts0s_num > IMG.rpts1s_num) check_Half_left(IMG);
 }
 
 // ========== 左斜入十字检测 ==========
 void Crossroad::check_Half_left(Tracking &IMG)
 {
     // 远端边线提取
     cross_farline_L(IMG);
 
     // L点二次检查
     if (far_Lpt0_found && IMG.Lpt0_found)
     {
         float dx = far_rpts0s[far_Lpt0_rpts0s_id][0] - IMG.rpts0s[IMG.Lpt0_rpts0s_id][0];
         float dy = far_rpts0s[far_Lpt0_rpts0s_id][1] - IMG.rpts0s[IMG.Lpt0_rpts0s_id][1];
         float dn = sqrtf(dx * dx + dy * dy);
         if (fabs(dn - 0.35 * pixel_per_meter) > 0.35 * pixel_per_meter) far_Lpt0_found = false;
     }
     is_straight_far_0 = far_rpts0s_num > (0.55 / sample_dist);
 
     if (far_Lpt0_found && flag_circle == CIRCLE_NONE && !IMG.is_straight1 && IMG.Lpt0_rpts0s_id < 30)
     {
         flag_cross = CROSS_HALF;
         Lpt0_found_flag = 1;
         cout << "左斜入" << endl;
         cross_route = IMG.encoder.route;
         IMG.element_over = false;
     }
 }
 
 // ========== 右斜入十字检测 ==========
 void Crossroad::check_Half_right(Tracking &IMG)
 {
     cross_farline_R(IMG);
 
     // L点二次检查
     if (far_Lpt1_found && IMG.Lpt1_found)
     {
         float dx = far_rpts1s[far_Lpt1_rpts1s_id][0] - IMG.rpts1s[IMG.Lpt1_rpts1s_id][0];
         float dy = far_rpts1s[far_Lpt1_rpts1s_id][1] - IMG.rpts1s[IMG.Lpt1_rpts1s_id][1];
         float dn = sqrtf(dx * dx + dy * dy);
         if (fabs(dn - 0.35 * pixel_per_meter) > 0.30 * pixel_per_meter) far_Lpt1_found = false;
     }
     is_straight_far_1 = far_rpts1s_num > (0.45 / sample_dist);
 
     if (far_Lpt1_found && flag_circle == CIRCLE_NONE && !IMG.is_straight0 && IMG.Lpt1_rpts1s_id < 30)
     {
         flag_cross = CROSS_HALF;
         Lpt1_found_flag = 1;
         cout << "            右斜入" << endl;
         cross_route = IMG.encoder.route;
         IMG.element_over = false;
     }
 }
 
 // ========== 砖块检测 ==========
 void Crossroad::check_block(Tracking &IMG, Mat &imgBinary)
 {
     int black_num = 0;
     if (IMG.Lpt0_found && IMG.Lpt0_rpts0s_id < 28 && IMG.tu0_x < 140 && IMG.tu0_x > 70)
     {
         for (int i = 0; i < 10; i++)
         {
             for (int j = 0; j < 5; j++)
             {
                 if (AT_IMAGE(imgBinary, IMG.tu0_x - 1 - i, IMG.tu0_y - 5 - j) == 0)
                 {
                     black_num++;
                 }
             }
         }
     }
 
     if (black_num > 25)
     {
         block_route = IMG.encoder.route;
         cout << "砖块在左" << endl;
         flag_block = BLOCK_VOID;
         left_flag = true;
     }
 }
 
 // ========== 砖块执行 ==========
 void Crossroad::run_block(Tracking &IMG)
 {
     if (left_flag)
     {
         flag_track = TRACK_RIGHT;
         block_move_flag = true;
         if ((IMG.encoder.route - block_route) >= 40)
         {
             block_move_flag = false;
             flag_block = BLOCK_NONE;
         }
     }
 }