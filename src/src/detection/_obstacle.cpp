/**
 * 智能车竞赛 - 障碍物控制模块
 */

 #include <algorithm>
 #include <fstream>
 #include <iostream>
 #include <cmath>
 #include "../recognition/tracking.cpp"
 #include "../../include/common.hpp"
 #include "../../include/detection.hpp"
 
 class Obstacle
 {
 public:
     // 变量定义
     std::vector<POINT> bezier_left, bezier_right;  // 左右边线贝塞尔曲线拟合
     uint16_t vector_size = 0;
     int16_t route = 0;                    // 障碍物检测起始里程
     int block_route = 0;                  // 砖块检测起始里程
     int blockaa = 0;                      // 砖块检测标志
     bool out_flag = false;
     bool block_left = false;
     bool block_right = false;
     bool block_flag = true;
 
     int16_t block_ready_route = 0;
     vector<PredictResult> resultsObs;     // 锥桶AI检测数据
     
     // 传统视觉检测参数
     static const int SCAN_HEIGHT = IMAGE_HEIGHT / 3;    // 扫描区域高度
     static const int MIN_BLOCK_AREA = 200;              // 最小砖块面积
     static const int MAX_BLOCK_AREA = 2000;             // 最大砖块面积
     static const int BLACK_THRESHOLD = 50;              // 黑色阈值
     bool traditional_detection_enabled = true;          // 是否启用传统检测
     int SCAN_START = 50;                    // 从底部往上30行开始扫描
     int SCAN_LINES = 120;                   // 扫描15行
     static const int MIN_BLACK_WIDTH = 25;  // 最小黑色宽度
     static const int MAX_BLACK_WIDTH = 60;  // 最大黑色宽度
     static const int MIN_WHITE_WIDTH = 5;   // 最小白色宽度
     int SCAN_X_LEFT = 100;                  // 左侧扫描的x坐标
     int SCAN_X_RIGHT = 200;                 // 右侧扫描的x坐标
     int scan_counter = 0;                   // 扫描计数器
 
     // 函数声明
     void check_obstacle(Tracking &IMG, Mat &frame);
     void run_obstacle(Tracking &IMG, Mat &img);
 
 private:
     int last_center_x = -1;                 // 记录上一帧检测到的中点位置
 };
 
 // 检测障碍物类型和数量
 void Obstacle::check_obstacle(Tracking &IMG, Mat &frame)
 {
     if (flag_obstacle == OBSTACLE_DETECTION && !com_predictor_results.empty())
     {
         // 根据检测到的障碍物数量决定状态
         if (com_predictor_results.size() == 1) 
         {
             flag_obstacle = OBSTACLE_READY;
             cout << "一个障碍物" << endl;
         } 
         else if (com_predictor_results.size() > 1)
         {
             flag_obstacle = OBSTACLE_MULTIPLE;
             cout << "多个障碍物" << endl;
         }
         
         IMG.element_over = false;
     }
 }
 
 // 障碍物区域导航状态机
 void Obstacle::run_obstacle(Tracking &IMG, Mat &img)
 {
     // ========== 障碍物检测与分类 ==========
     if (flag_obstacle == OBSTACLE_READY || flag_obstacle == OBSTACLE_MULTIPLE)
     {
         resultsObs.clear();
         PredictResult resultObs;
         
         // 筛选有效检测结果
         for (size_t i = 0; i < com_predictor_results.size(); i++)
         {
             if ((com_predictor_results[i].label == "cone" && 
                  com_predictor_results[i].y > IMAGE_HEIGHT * 0.35) ||
                 ((com_predictor_results[i].y + com_predictor_results[i].height) > IMAGE_HEIGHT * 0.3 && 
                  com_predictor_results[i].label == "block") ||
                 ((com_predictor_results[i].y + com_predictor_results[i].height) > IMAGE_HEIGHT * 0.35 && 
                  com_predictor_results[i].label == "pedestrian"))
             {
                 resultsObs.push_back(com_predictor_results[i]);
             }
         }
 
         // 选取Y坐标最大的检测结果（距离最近）
         if (resultsObs.size() > 0)
         {
             int areaMax = 0;
             int index = 0;
             for (size_t i = 0; i < resultsObs.size(); i++)
             {
                 int area = resultsObs[i].y;
                 if (area >= areaMax)
                 {
                     index = i;
                     areaMax = area;
                 }
             }
             resultObs = resultsObs[index];
         }
 
         int disLeft = 0;
         int disRight = 0;
         
         // 计算障碍物到左右边线的距离
         if (resultObs.label == "cone" || 
             resultObs.label == "pedestrian" || 
             resultObs.label == "block")
         {
             disLeft = resultObs.x + resultObs.width / 2 - IMG.ipts0[30][0];
             disRight = IMG.ipts1[30][0] - (resultObs.x + resultObs.width / 2);
         }
 
         // 障碍物位置判断
         if (resultObs.label == "cone" || 
             resultObs.label == "pedestrian" || 
             resultObs.label == "block")
         {
             // 障碍物靠左
             if (((resultObs.x + resultObs.width / 2) > IMG.ipts0[10][0]) && 
                 (IMG.ipts1[10][0] > (resultObs.x + resultObs.width / 2)) && 
                 (disLeft <= disRight))
             {
                 flag_track = TRACK_LEFT;
                 route = IMG.encoder.route;
 
                 if (resultObs.label == "block")
                 {
                     block_route = IMG.encoder.route;
                     cout << "砖块在左" << endl;
                     flag_track = TRACK_LEFT;
                     block_left_ai = true;
                     block_qianzhan_flag = true;
                     blockaa = 1;
                 } 
                 else 
                 {
                     if (!cone_left)
                     {
                         cout << "锥桶在左" << endl;
                         cone_left = true;
                     }
                 }
             }
             // 障碍物靠右
             else if (((resultObs.x + resultObs.width / 2) > IMG.ipts0[10][0]) &&
                      (IMG.ipts1[10][0] > (resultObs.x + resultObs.width / 2)) &&
                      (disLeft > disRight))
             {
                 flag_track = TRACK_RIGHT;
                 route = IMG.encoder.route;
                 
                 if (resultObs.label == "block") 
                 {
                     block_route = IMG.encoder.route;
                     cout << "砖块在右" << endl;
                     flag_track = TRACK_RIGHT;
                     blockaa = 1;
                     block_right_ai = true;
                     block_qianzhan_flag = true;
                 } 
                 else 
                 {
                     if (!cone_right) 
                     {
                         cone_right = true;
                         cout << "锥桶在右" << endl;
                     }
                 }
             }
         }
     }
 
     // ========== 单个障碍物退出条件 ==========
     if (((((IMG.encoder.route - route) >= 1 && (cone_left || cone_right))) ||
          ((IMG.encoder.route - block_route) >= 60 && blockaa == 1)) &&
         flag_obstacle == OBSTACLE_READY))
     {
         if (blockaa == 1)
         {
             cout << "block" << endl;
         }
         cout << "单个障碍物强制退出路障" << endl;
         
         route = 0;
         flag_obstacle = OBSTACLE_NONE;
         cone_left = false;
         cone_right = false;
         block_qianzhan_flag = false;
         blockaa = 0;
         block_left = false;
         block_right = false;
         block_right_ai = false;
         block_left_ai = false;
         IMG.element_over = true;
     }
 
     // ========== 多个障碍物快速退出 ==========
     if (((IMG.encoder.route - route) >= 1) && 
         (flag_obstacle == OBSTACLE_MULTIPLE))
     {
         cout << "多个障碍物强制退出" << endl;
         
         route = 0;
         flag_obstacle = OBSTACLE_NONE;
         cone_left = false;
         cone_right = false;
         block_qianzhan_flag = false;
         block_left = false;
         block_right = false;
         block_right_ai = false;
         block_left_ai = false;
         IMG.element_over = true;
         cout << "退                 123456               出" << endl;
     }
 
     // ========== BUG保护：强制退出 ==========
     if ((flag_obstacle == OBSTACLE_READY && 
          (!cone_left && !cone_right) && 
          (blockaa != 1)) && 
         IMG.encoder.route - route > 1)
     {
         cout << "发生BUG强制退出" << endl;
         
         route = 0;
         flag_obstacle = OBSTACLE_NONE;
         cone_left = false;
         cone_right = false;
         block_qianzhan_flag = false;
         block_left = false;
         block_right = false;
         block_right_ai = false;
         block_left_ai = false;
         IMG.element_over = true;
         cout << "退                 123456               出" << endl;
     }
 }