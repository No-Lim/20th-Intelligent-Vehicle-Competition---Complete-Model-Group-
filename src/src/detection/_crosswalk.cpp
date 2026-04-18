/**
 * @file crosswalk.cpp
 * @brief 智能车竞赛 - 斑马线控制模块
 */

 #include <algorithm>
 #include "../recognition/tracking.cpp"
 #include "../../include/common.hpp"
 #include "../../include/detection.hpp"
 
 class CROSSWALK
 {
 public:     
     // 变量定义
     int16_t crosswalk_route = 0;    // 记录检测到斑马线时的编码器里程
 
     // 函数声明
     void check_crosswalk(Tracking &IMG);  // 检测斑马线入口
     void run_crosswalk(Tracking &IMG);    // 斑马线区域状态机
 };
 
 /**
  * @brief 检测是否进入斑马线区域
  */
 void CROSSWALK::check_crosswalk(Tracking &IMG) 
 {
     if ((flag_crosswalk == CROSSWALK_DETECTION)) 
     {
         IMG.element_identify = 7;
         flag_crosswalk = CROSSWALK_START;
         IMG.element_over = false;
     }
 }
 
 /**
  * @brief 斑马线区域导航状态机
  */
 void CROSSWALK::run_crosswalk(Tracking &IMG)
 {
     PredictResult crosswalk_resultObs;
     
     // ========== 状态1：检测斑马线 ==========
     if ((flag_crosswalk == CROSSWALK_START) && (IMG.encoder.route > 500))
     {   
         cout << "进入了斑马线" << endl;
         
         for (size_t i = 0; i < com_predictor_results.size(); i++)
         {
             if (com_predictor_results[i].label == "crosswalk")
             {
                 crosswalk_resultObs = com_predictor_results[i];
                 crosswalk_route = IMG.encoder.route;
                 flag_crosswalk = CROSSWALK_END;
             }
         }
         
         // 检测框宽度过小，视为误检，直接结束
         if (crosswalk_resultObs.width < 80)
         {
             flag_crosswalk = CROSSWALK_NONE;
             IMG.element_over = true;
         }
     }
     
     // ========== 状态2：斑马线通过中 ==========
     else if ((flag_crosswalk == CROSSWALK_END))
     {
         // 持续检查宽度，防止误检
         if (crosswalk_resultObs.width < 80)
         {
             flag_crosswalk = CROSSWALK_NONE;
             IMG.element_over = true;
         }
         
         // 行驶距离超过120，触发停车
         if (IMG.encoder.route - crosswalk_route > 120)
         {
             flag_crosswalk = CROSSWALK_STOP;
             IMG.element_over = true;
         }
     }
 }