/**
 * 智能车竞赛 - 临时停车区控制模块
 */

 #include <algorithm>
 #include <fstream>
 #include <iostream>
 #include <cmath>
 #include "../recognition/tracking.cpp"
 #include "../../include/common.hpp"
 #include "../../include/detection.hpp"
 
 class LAYBY
 {
 private:
     vector<PredictResult> layby_resultsObs; // 锥桶AI检测数据
     int route = 0;                          // 开始检测时的路程记录
     int layby_count = 0;                    // 停车计数器
     int miss_count = 0;                     // 路牌消失计数器
     bool miss_layby = false;                // 路牌是否消失标志
     int layby_count0 = 0;
     int route_safe = 0;                     // 安全距离起始点
     int first_detect = 0;                   // 首次检测标志
     int noSignCounter = 0;                  // 连续未检测到路牌的计数器
 
     const int TARGET_DISTANCE = 30;         // 目标停车距离(cm)
     const int MISS_THRESHOLD = 10;          // 路牌消失确认阈值(帧数)
     
 public:
     void run_layby(Tracking &IMG);          // 主运行函数
     void check_layby(Tracking &IMG);        // 检查临时停车状态
 };
 
 // 检查临时停车状态
 void LAYBY::check_layby(Tracking &IMG) 
 {
     if (flag_layby == LAYBY_DETECTION) 
     {
         cout << "检测到临时停车区域" << endl; 
         flag_layby = LAYBY_READY;     
         IMG.element_identify = 4;           // 设置元素识别类型
         IMG.element_over = false;           // 重置元素完成标志
     }
 }
 
 // 临时停车区状态机主逻辑
 void LAYBY::run_layby(Tracking &IMG)
 {
     bool signDetected = false;              // 当前帧是否检测到路牌
 
     // ========== 1. 检测路牌位置 ==========
     for (size_t i = 0; i < com_predictor_results.size(); i++)
     {
         if ((com_predictor_results[i].label == "school" || 
              com_predictor_results[i].label == "company") && 
             (!com_predictor_results.empty()))
         {
             signDetected = true;
             miss_count = 0;                 // 重置消失计数器
             
             // 如果已经在RUNING状态但又检测到路牌，需要退出RUNING状态
             if (flag_layby == LAYBY_RUNING)
             {
                 flag_layby = LAYBY_READY;
                 cout << "重新检测到路牌,退出RUNING状态" << endl;
             }
             
             // 路牌在左侧
             if (com_predictor_results[i].x + com_predictor_results[i].width < 150)
             { 
                 cout << "路牌在左侧" << endl;
                 flag_track = TRACK_RIGHT;
                 first_detect++;
                 if (!layby_left)
                 {
                     layby_left = true;
                     flag_layby = LAYBY_READY;
                 }
             }
             // 路牌在右侧
             else if (com_predictor_results[i].x > 150)
             {
                 cout << "路牌在右侧" << endl;
                 flag_track = TRACK_LEFT;
                 first_detect++;
                 if (!layby_right)
                 {
                     layby_right = true;
                     flag_layby = LAYBY_READY;
                 }
             }
         }
     }
 
     // 记录安全距离起始点
     if (first_detect == 1 && flag_layby == LAYBY_READY)
     {
         route_safe = IMG.encoder.route;
     }
     
     // ========== 2. 路牌消失处理 ==========
     if (!signDetected && flag_layby == LAYBY_READY)
     {
         miss_count++;
         if (miss_count > MISS_THRESHOLD)
         {
             flag_layby = LAYBY_RUNING;
             cout << "路牌消失进RUNING" << endl;
             route = IMG.encoder.route;      // 重置起始位置
         }
     }
     else if (signDetected)
     {
         miss_count = 0;                     // 重置消失计数器
     }
 
     // ========== 3. 行驶状态控制 ==========
     if (flag_layby == LAYBY_RUNING) 
     {
         // 再次检查是否意外检测到路牌
         if (signDetected) 
         {
             noSignCounter = 0;              // 重置计数器
             flag_layby = LAYBY_READY;
             cout << "RUNING状态下又检测到路牌，返回READY状态" << endl;
             return;
         }
         else 
         {
             noSignCounter++;                // 增加未检测计数
             
             // 连续10帧未检测到路牌且行驶超过85
             if (noSignCounter >= 10 && (IMG.encoder.route - route_safe > 85)) 
             {
                 noSignCounter = 0;          // 重置计数器
                 flag_layby = LAYBY_STOP;
                 cout << "安全距离" << IMG.encoder.route - route_safe;
                 cout << "连续50帧未检测到路牌,切换至STOP状态" << endl;
             }
         }
     }
     
     // ========== 4. 停车状态 ==========
     if (flag_layby == LAYBY_STOP)
     {
         layby_count++;
         
         if (layby_count >= 40)
         {
             flag_layby = LAYBY_OUT;
             route = IMG.encoder.route;      // 记录离开起点
         }
     }
     
     // ========== 5. 离开状态 ==========
     if (flag_layby == LAYBY_OUT)
     {
         int current_distance = IMG.encoder.route - route;
         
         if (current_distance >= 1)
         {
             // 重置所有状态
             flag_layby = LAYBY_NONE;
             layby_count = 0;
             route = 0;
             layby_right = false;
             layby_left = false;
             IMG.element_over = true;
             miss_count = 0;
             route_safe = 0;
             first_detect = 0;
             noSignCounter = 0;
             cout << "临时停车流程完成" << endl;
         }
     }
 }