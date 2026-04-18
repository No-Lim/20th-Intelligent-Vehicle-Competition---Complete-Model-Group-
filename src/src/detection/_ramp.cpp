/**
 * 智能车竞赛 - 坡道控制模块
 */

 #include <algorithm>
 #include "../recognition/tracking.cpp"
 #include "../../include/common.hpp"
 #include "../../include/detection.hpp"
 
 class RAMP 
 {
 public:
     // 变量定义
     float ramp_route = 0;       // 坡道起始里程
     float ramp_over_route = 0;  // 坡道结束里程
 
     // 函数声明
     void check_ramp(Tracking &IMG);
     void run_ramp(Tracking &IMG, int ramp_up_route, int ramp_down_route);
     void run_ramp2(Tracking &IMG);
 };
 
 // 检测坡道入口
 void RAMP::check_ramp(Tracking &IMG) 
 {
     if (flag_ramp == RAMP_DETECTION) 
     {
         IMG.element_identify = 2;
         ramp_route = IMG.encoder.route;
         flag_ramp = RAMP_UP;
         IMG.element_over = false;
         cout << "ramp begin" << endl;
     }
 }
 
 // 坡道状态机（带参数版本）
 // 参数说明：ramp_up_route - 上坡距离，ramp_down_route - 下坡距离
 void RAMP::run_ramp(Tracking &IMG, int ramp_up_route, int ramp_down_route)
 {
     // 上坡完成，切换下坡
     if (flag_ramp == RAMP_UP && (IMG.encoder.route - ramp_route > ramp_up_route))
     {          
         cout << "该下坡了" << endl;
         flag_ramp = RAMP_DOWN;
         ramp_route = IMG.encoder.route;
     }
     // 下坡完成，坡道结束
     else if (flag_ramp == RAMP_DOWN && (IMG.encoder.route - ramp_route > ramp_down_route))
     {
         cout << "坡道结束" << endl;
         flag_ramp = RAMP_NONE;
         IMG.element_over = true;
     }    
 }
 
 // 坡道状态机（简化版本，带刹车控制）
 void RAMP::run_ramp2(Tracking &IMG)
 {
     // 超时保护：150距离后强制退出
     if (IMG.encoder.route - ramp_route > 150 && flag_ramp == RAMP_UP)
     {
         flag_ramp = RAMP_NONE;
         IMG.element_over = true;
     }
 
     // 上坡状态触发刹车并立即结束
     if (flag_ramp == RAMP_UP)
     {
         flag_brake = BRAKE_START;
         flag_ramp = RAMP_NONE;
         IMG.element_over = true;
     }
 }