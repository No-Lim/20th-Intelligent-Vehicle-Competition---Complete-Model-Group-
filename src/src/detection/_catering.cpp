/**
 * @file catering.cpp
 * @brief 智能车竞赛 - 餐饮区（汉堡）控制模块
 */

 #include <algorithm>
 #include <fstream>
 #include <iostream>
 #include <cmath>
 #include "../recognition/tracking.cpp"
 #include "../../include/common.hpp"
 #include "../../include/detection.hpp"
 
 class Catering
 {
 public:
     int16_t burger_count = 0;    // 停车计时计数器
     bool burger_zuo = 0;         // 汉堡在左侧标志
     bool burger_you = 0;         // 汉堡在右侧标志
     int route;                   // 记录进入元素时的编码器里程
 
     void check_burger(Tracking &IMG);  // 检测汉堡元素入口
     void run_burger(Tracking &IMG);    // 汉堡区域状态机主逻辑
 };
 
 /**
  * @brief 检测是否进入汉堡区域
  */
 void Catering::check_burger(Tracking &IMG)
 {
     if (flag_burger == BURGER_DETECTION) 
     {
         cout << "BURGER_要开始了" << endl; 
         flag_burger = BURGER_START;     
         IMG.element_identify = 3;
         IMG.element_over = false;
     }
 }
 
 /**
  * @brief 汉堡区域导航状态机
  */
 void Catering::run_burger(Tracking &IMG)
 {
     PredictResult resultObs; 
     
     // ========== 汉堡检测 ==========
     for (size_t i = 0; i < com_predictor_results.size(); i++)
     {
         if (com_predictor_results[i].label == "burger" && 
             com_predictor_results[i].y > IMAGE_HEIGHT * 0.3)       
         {
             resultObs = com_predictor_results[i];
         }
     }
 
     // ========== 汉堡位置判断与入口初始化 ==========
     if (resultObs.label == "burger")
     {
         cout << "总路程" << IMG.encoder.route << "元素路程" << route << endl;
         
         if (flag_burger == BURGER_DETECTION)
         {
             IMG.aim_distance_f = 0.3;
             IMG.aim_distance_n = 0.7;
             
             // 状态1：汉堡在左，循右线进入
             if (resultObs.x < 150)
             { 
                 cout << "汉堡在左" << endl;
                 flag_track = TRACK_RIGHT;
                 route = IMG.encoder.route;
                 burger_zuo = 1;
                 flag_burger = BURGER_START;
             }
             
             // 汉堡在右，循左线进入
             if (resultObs.x > 150)
             {
                 cout << "汉堡在右" << endl;
                 flag_track = TRACK_LEFT;
                 route = IMG.encoder.route;
                 burger_you = 1;
                 flag_burger = BURGER_START;
             }
         }
     }
 
     // ========== 餐饮区导航状态机 ==========
     if (flag_burger == BURGER_START || flag_burger == BURGER_STOP)
     {
         IMG.aim_distance_f = 0.3;
         IMG.aim_distance_n = 0.7;
         
         // 状态2：打角进入（路程15~65）
         if (((IMG.encoder.route - route) >= 15) && 
             ((IMG.encoder.route - route) <= 65) && 
             (burger_zuo == 1))
         {   
             cout << "总路程" << IMG.encoder.route << "元素路程" << route << endl;
             flag_track = TRACK_RIGHT;
             burger_left = true;
         }
         else if (((IMG.encoder.route - route) >= 15) && 
                  ((IMG.encoder.route - route) <= 65) && 
                  (burger_you == 1))
         { 
             cout << "总路程" << IMG.encoder.route << "元素路程" << route << endl;
             flag_track = TRACK_LEFT;
             burger_right = true;
         }
         
         // 过渡状态（路程65~90）
         if (((IMG.encoder.route - route) >= 65) && 
             ((IMG.encoder.route - route) < 90))
         {   
             cout << "总路程" << IMG.encoder.route << "元素路程" << route << endl;
             cout << "过渡" << endl;
             flag_burger = BURGER_START;
         }
 
         // 状态3：停车（路程≥90，且计时未满）
         if (((IMG.encoder.route - route) >= 90) && (burger_count < 25))
         { 
             cout << "总路程" << IMG.encoder.route << "元素路程" << route << endl;
             flag_burger = BURGER_STOP;       
             burger_count++;
             cout << "STOP!!!!!!!!!!!!!" << endl;
         }
         // 状态4：停车结束，准备驶出
         else if ((IMG.encoder.route - route) > 90 && (burger_count >= 25))
         { 
             cout << "总路程" << IMG.encoder.route << "元素路程" << route << endl;
             cout << "出去" << endl;
             flag_burger = BURGER_END;       
         }
     }
     
     // ========== 驶出餐饮区 ==========
     if (flag_burger == BURGER_END)
     {
         // 驶出打角阶段（路程≤295）
         if ((IMG.encoder.route - route) <= 295) 
         {
             cout << burger_you << endl;
 
             if (burger_zuo)
             {
                 cout << "右打角" << endl;
                 flag_track = TRACK_RIGHT;
             }
             else if (burger_you)
             {
                 cout << "左打角" << endl;
                 flag_track = TRACK_LEFT;
             }
         }
         // 驶出完成，复位所有状态
         else if ((IMG.encoder.route - route) > 295)
         {    
             cout << "burger_over" << "路程" << IMG.encoder.route - route << endl;
             
             burger_zuo = 0;
             burger_you = 0;
             burger_left = false;
             burger_right = false;
             burger_count = 0;
             route = 0;
             
             IMG.element_over = true;
             flag_burger = BURGER_NONE;
         }
     }
 }