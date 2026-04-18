 #include <fstream>
 #include <iostream>
 #include <cmath>
 #include "../include/common.hpp"
 #include "../include/json.hpp"
 #include "../include/uart.hpp"
 
 using namespace std;
 using namespace cv;
 
 /**
  * @brief 运动控制器
  *
  */
 class Motion
 {
  private:
    // int countShift = 0; // 变速计数器
    // int stopCounter = 0; // 静态计数器，保持计数状态
  public:
     /**
      * @brief 初始化：加载配置文件
      *
      */
     Motion()
     {
         string jsonPath = "../src/config/config.json";
         std::ifstream config_is(jsonPath);
         if (!config_is.good())
         {
             std::cout << "Error: Params file path:[" << jsonPath
                       << "] not find .\n";
             exit(-1);
         }
 
         nlohmann::json js_value;
         config_is >> js_value;
 
         try
         {
             params = js_value.get<Params>();
         }
         catch (const nlohmann::detail::exception &e)
         {
             std::cerr << "Json Params Parse failed :" << e.what() << '\n';
             exit(-1);
         }
         int PFF_value[7] = {-params.PFF_0 , -params.PFF_1 , -params.PFF_2 , 0 , params.PFF_2 , params.PFF_1 , params.PFF_0};
         int DFF_value[7] = {-params.DFF_0 , -params.DFF_1 , -params.DFF_2 , 0 , params.DFF_2 , params.DFF_1 , params.DFF_0};
         float UFF_P_value[7] = {params.UFF_P_0 , params.UFF_P_1 , params.UFF_P_2 , params.UFF_P_3 , params.UFF_P_4 , params.UFF_P_5 , params.UFF_P_6};
         float UFF_D_value[7] = {params.UFF_D_0 , params.UFF_D_1 , params.UFF_D_2 , params.UFF_D_3 , params.UFF_D_4 , params.UFF_D_5 , params.UFF_D_6};
         for (int i = 0; i < 7; ++i) 
             {
                 PFF[i] = PFF_value[i];
                 DFF[i] = DFF_value[i];
                 UFF_P[i] = UFF_P_value[i];
                 UFF_D[i] = UFF_D_value[i];
             }
         cout << "--- PFF_0:" <<PFF[0] << " | PFF_1:" << PFF[1] << " | PFF_2:" << PFF[2] << endl;
         cout << "--- DFF_0:" << params.DFF_0 << " | DFF_1:" << params.DFF_1 << " | DFF_2:" << params.DFF_2 << endl;
         cout << "--- UFF_P:" << params.UFF_P_0 << "  |  UFF_D:" << params.UFF_D_0 << endl;
         cout << "--- aim_distance_f:" << params.aim_distance_f << "  |  aim_distance_n:" << params.aim_distance_n << endl;
     };
 
     /**
      * @brief 控制器核心参数
      *
      */
     struct Params
     {
        //config文件里面修改就可以

         float speedLow = 0.8;                              // 智能车最低速
         float speedHigh = 0.8;                             // 智能车最高速                                //停车区，避障区速度
         float speed_obstacle = 0;
         float speedBridge = 0.6;                           // 坡道速度
         float speedDown = 0.5;                             // 特殊区域降速速度
         float speedXieru = 0;
         float speedMid = 0;
         float speedMidHigh;

         float speedTwo=0;
         float speedBattery=0;
         float speed_battery_out=0;
         float speed_buger_start=0;
         float speed_burger_end=0;
         float speed_layby_in = 0;
         float speed_layby_running = 0;
         float speed_layby_out = 0;
         float speed_circle_in = 0;
         float speedCircle_Runing = 0;
         float speedCircle_Out = 0;

         int parker_far_route_in = 0;
         int parker_far_route_turn = 0;

         int ramp_up_route = 0;
         int ramp_down_route = 0;

         bool debug = false;                                // 调试模式使能
         bool saveImg = false;                              // 存图使能
      
         bool bridge = true;                                // 坡道区使能
         bool danger = true;                                // 避障使能
         bool burger = true;                                // 餐饮区使能
         bool ring = true;                                  // 环岛使能
         bool cross = true;                                 // 十字道路使能
         bool layby = true;
         bool battery = true;
         uint8_t obstacle_p = 0;                               //避障p增益
         uint8_t ring_p = 0 ;                                  //环岛p增益
         uint8_t ring_in_p = 30;
         bool parker_far = false;
         bool parker_near = false;
 
          int8_t aim_angle_filter = 0;                        //停车区打角
      
 
         float score = 0.5;                                 // AI检测置信度
         string model = "../res/model/yolov3_mobilenet_v1"; // 模型路径
         int16_t PFF_0 = 0;                                 //模糊pid PFF[7]
         int16_t PFF_1 = 0;
         int16_t PFF_2 = 0; 
         int16_t DFF_0 = 0;                                 //模糊pid DFF[7]
         int16_t DFF_1 = 0;
         int16_t DFF_2 = 0;
         float UFF_P_0 = 0;                                 //模糊pid UFF_P[7]
         float UFF_P_1 = 0;
         float UFF_P_2 = 0;
         float UFF_P_3 = 0;
         float UFF_P_4 = 0;
         float UFF_P_5 = 0;
         float UFF_P_6 = 0;
         float UFF_D_0 = 0;                                 //模糊pid UFF_D[7]
         float UFF_D_1 = 0;
         float UFF_D_2 = 0;
         float UFF_D_3 = 0;
         float UFF_D_4 = 0;
         float UFF_D_5 = 0;
         float UFF_D_6 = 0;
         float aim_distance_f = 0;                          //远锚点
         float aim_distance_n = 0;                          //近锚点

         //int burger_countc = 0;
         // int parker_countc = 0;
         // int layby_countc = 0; 

         NLOHMANN_DEFINE_TYPE_INTRUSIVE(Params, speedLow, speedHigh,speed_obstacle, speedBridge, speedDown,
         speedXieru, 
         speedMid,speedMidHigh,
         speedTwo,
         speedBattery,speed_battery_out,
         speed_battery_out,
         speed_buger_start,
         speed_burger_end,
         speed_layby_in,
         speed_layby_running,
         speed_layby_out,
         speed_circle_in,
         speedCircle_Runing,
         speedCircle_Out,
         parker_far_route_in,
         parker_far_route_turn,
         ramp_up_route,
         ramp_down_route,
         debug, saveImg,  bridge, danger,
         burger, layby,battery,ring, cross,
         ring_p,ring_in_p,
         obstacle_p,
         parker_far,parker_near,
         aim_angle_filter,
         score, model,PFF_0,PFF_1,PFF_2,DFF_0,DFF_1,DFF_2,UFF_P_0,
         UFF_P_1,UFF_P_2,UFF_P_3,UFF_P_4,UFF_P_5,UFF_P_6,UFF_D_0,UFF_D_1,UFF_D_2,UFF_D_3,UFF_D_4,UFF_D_5,UFF_D_6,aim_distance_f,aim_distance_n); // 添加构造函数
     };
 
     Params params;                   // 读取控制参数
     uint16_t servoPwm = PWMSERVOMID; // 发送给舵机的PWM  
     /**
      * @brief 姿态PD控制器
      *
      * @param controlCenter 智能车控制中心
      */
     void poseCtrl(float controlCenter, Payload_t &payload)
     {
         static float errorLast = 0;       // 记录前一次的偏差
         controlCenter = controlCenter > 36 ? 36 : controlCenter;
         controlCenter = controlCenter < -36 ? -36 : controlCenter;
 
         float error = controlCenter ; // 图像控制中心转换偏差
                       
         //std::cout << "偏差 :"<< error<<std::endl;
 
         // if (abs(error - errorLast) > COLSIMAGE / 10)
         // {
         //     error = error > errorLast ? errorLast + COLSIMAGE / 10 : errorLast - COLSIMAGE / 10;  //误差变化范围限制在+-30
         // }
 
         turnP = Fuzzy_Kp(error, (error - errorLast));
         turnD = Fuzzy_Kd(error, (error - errorLast));
         //std::cout <<"P :"<<turnP<<" D :"<<turnD<<std::endl;
         float pwmDiff;

         //环岛
         //响应速度更快
         if(( flag_circle == CIRCLE_RIGHT_RUNNING || flag_circle == CIRCLE_LEFT_RUNNING))
         {
             pwmDiff = (error * (turnP + params.ring_p)) + (error - errorLast) * turnD;
         }
         else if(flag_circle==CIRCLE_RIGHT_OUT||flag_circle==CIRCLE_LEFT_OUT)
         {
            pwmDiff = (error * (turnP + 15)) + (error - errorLast) * turnD;
         }
         else if((flag_circle==CIRCLE_LEFT_IN||flag_circle==CIRCLE_RIGHT_IN))
         {
            pwmDiff = (error * (turnP +params.ring_in_p )) + (error - errorLast) * turnD;
         }
         //避障控制增益
         else if( (flag_obstacle != OBSTACLE_NONE))
         {
             pwmDiff = (error * (turnP + params.obstacle_p)) + (error - errorLast) * (turnD+1);
         }
         if(flag_parker==PARKER_DETECTION||flag_parker == PARKER_READY||flag_parker==PARKER_RUNING)
         {
            pwmDiff = (error * (turnP+3)) + (error - errorLast) * turnD;  
         }
         else //正常情况
         {
             pwmDiff = (error * turnP) + (error - errorLast) * turnD;
         }
         //std::cout<<error << "转换后 :"<< pwmDiff<<std::endl;
         
         errorLast = error;
         //限幅
         if(pwmDiff > 400)
             pwmDiff = 400;
         if(pwmDiff < -400)
             pwmDiff = -400;
         
         servoPwm = (uint16_t)(PWMSERVOMID - pwmDiff); // PWM转换
         if (servoPwm > PWMSERVOMAX)
             servoPwm = PWMSERVOMAX;  
         if (servoPwm < PWMSERVOMIN)
             servoPwm = PWMSERVOMIN;   
 
         //std::cout <<"舵机："<<servoPwm<<std::endl;
          //payload.tAngle = (servoPwm + 20)*2;//舵机
         
        //  payload.tAngle = (servoPwm + 10.2)*6.6;//舵机
            payload.tAngle = servoPwm *6.6;
         //payload.tAngle = 1510*6.6;
         //std::cout <<"舵机："<<payload.tAngle<<std::endl;
     }
 
/**
 * @brief 变加速控制函数，根据不同的标志位和条件动态调整目标速度。
 *
 * @param payload 用于存储速度控制结果的结构体。
 * @param IMG 用于获取图像识别信息的结构体。
 */
 void speedCtrl(Payload_t &payload, Tracking IMG)
 {


     //障碍物 
     if(flag_obstacle == OBSTACLE_MULTIPLE)
     {
        payload.tSpeed = params.speedTwo;
     }
     else if(flag_obstacle == OBSTACLE_READY)
     {
      payload.tSpeed = params.speed_obstacle;
     }
     
     else if (flag_protect == PROTECT_STOP)
     {
        payload.tSpeed = 1501;  // 设置目标速度为0
     }
   else if(flag_crosswalk == CROSSWALK_END)
   {
      payload.tSpeed = 344; 

   }
   else if (flag_ramp == RAMP_UP)  // 上坡状态
  {
      payload.tSpeed = params.speedBridge;  // 使用上坡速度参数
  }
  else if (flag_ramp == RAMP_DOWN)  // 下坡状态
  {
      payload.tSpeed = params.speedDown;  // 使用下坡速度参数
  }
      //充电停车区
     else if(flag_parker==PARKER_DETECTION||flag_parker == PARKER_READY||flag_parker==PARKER_RUNING||flag_parker==PARKER_IN)
     {
        payload.tSpeed=params.speedBattery;
     }
     else if(flag_parker == PARKER_STOP)
     {
        payload.tSpeed=5000;
     }
     else if(flag_parker == PARKER_OUT||flag_parker==PARKER_FAR_OUT0||flag_parker==PARKER_FAR_OUT1)
     {
        payload.tSpeed=1700;
     }
     else if(flag_parker ==PARKER_END)
     {
        payload.tSpeed=params.speed_battery_out;
     }
     //餐饮区//
     else if(flag_burger == BURGER_DETECTION || flag_burger == BURGER_START )
     {
      payload.tSpeed = params.speed_buger_start;
     }
     else if(flag_burger == BURGER_STOP)
     {
        payload.tSpeed = 0;
     }
     else if(flag_burger==BURGER_END)
     {

        payload.tSpeed = params.speed_burger_end;
     }

     //////临时停车区///////
     else if(flag_layby == LAYBY_STOP)
     {
      payload.tSpeed = 3000;
     }
     else if((flag_layby == LAYBY_DETECTION )|| (flag_layby == LAYBY_READY ))
     {
        payload.tSpeed = params.speed_layby_in;
     }
     else if((flag_layby == LAYBY_RUNING))
     {
      payload.tSpeed = params.speed_layby_running;
     }
     else if((flag_layby == LAYBY_OUT))
     {
      payload.tSpeed = 300;
     }
     else if(flag_crosswalk == CROSSWALK_STOP)
     {
      payload.tSpeed = 1501 ;
      //cout<<"斑马线stop"<<endl;
     }
     else if(flag_circle == CIRCLE_LEFT_IN|| flag_circle == CIRCLE_RIGHT_IN)
     {
         payload.tSpeed = params.speed_circle_in;
     }
     else if(flag_circle == CIRCLE_LEFT_RUNNING || flag_circle == CIRCLE_RIGHT_RUNNING )
     {
        payload.tSpeed = params.speedCircle_Runing;
     }
     else if( flag_circle == CIRCLE_LEFT_OUT||flag_circle == CIRCLE_RIGHT_OUT)
     {
      payload.tSpeed = params.speedCircle_Out;
     }
     //斜入十字 加速冲进去
     else if(((Lpt0_found_flag == 1)||(Lpt1_found_flag == 1))&&flag_circle==0)
     {
        payload.tSpeed = params.speedXieru ;
     }
     //直道加速
     else if((tracking.is_straight0||tracking.is_straight1)&&(flag_circle==CIRCLE_NONE))
     {
        payload.tSpeed = params.speedHigh ;
     }
     //小弯道大速度
     else if
     (
      ((!tracking.is_straight0)&&(!tracking.is_straight1))
      &&
      (IMG.aim_angle_filter<10&IMG.aim_angle_filter>(-10))
     )
     {
        payload.tSpeed = params.speedMidHigh;
     }
     //特大弯道强制减速
     else if(IMG.aim_angle_filter>19||IMG.aim_angle_filter<(-19))
     {
      payload.tSpeed = params.speedLow;
     }
     //正常弯道速度
     else
     {
       payload.tSpeed = params.speedMid;
     }


     payload.element = IMG.element_identify;
     // 输出目标速度（可选，用于调试）
     // std::cout << "电机速度：" << payload.tSpeed << std::endl;
 }
int pdControl(int kp,int kd,int error)
{
      // 上一次的误差
      static double error_last = 0;  
      // 计算微分项
      double derivative = (error - error_last);
      // 更新控制量
      double control = kp * error  + kd * derivative;
      // 更新上一次的误差
      error_last = error;

      return control;
}
     /**
      * @brief 模糊pid
      *
      * @param P 误差
      * @param D 误差与上次误差的差值
      */
  public:
 
     uint8_t rule_p[7][7]={
          {6, 5, 4, 3, 2, 0, 0,},   //   -3
          {5, 4, 3, 2, 1, 0, 1,},   //   -2
          {4, 3, 2, 1, 0, 1, 2,},   //   -1
          {3, 2, 1, 0, 1, 2, 3,},   //    0
          {2, 1, 0, 1, 2, 3, 4,},   //    1
          {1, 0, 1, 2, 3, 4, 5,},   //    2
          {0, 0, 2, 3, 4, 5, 6}};   //    3
     uint8_t rule_d[7][7]={
          {2, 2, 6, 5, 6, 4, 2,},   //   -3
          {1, 2, 5, 4, 3, 1, 0,},   //   -2
          {0, 1, 3, 3, 1, 1, 0,},   //   -1
          {0, 1, 1, 1, 1, 1, 0,},   //    0
          {0, 0, 0, 0, 0, 0, 0,},   //    1
          {5, 1, 1, 1, 1, 1, 1,},   //    2
          {6, 4, 4, 3, 3, 1, 1}};   //    3
          
 
     int16_t PFF[7] = {-36, -20, -10, 0, 10, 20, 36};
     int16_t DFF[7] = {-15, -10, -5, 0, 5, 10, 15};
 
     float UFF_P[7] = {5.0,7.0,9.0,11.0,13.0,15.0,17.0};//50帧率  0.45
 
     float UFF_D[7] = {3.26,3.19,4.20,5.22,5.24,5.26,5.28};  //0.45
 
 
     float Fuzzy_Kp(float P, float D) ;
     float Fuzzy_Kd(float P, float D) ;
 
     // FUZZY_PID()
     // {
     //     ;
     // }
     // ~FUZZY_PID()
     // {
     //     ;
     // }
 
     float kp = 0;
     float kd = 0;
     //int error = 0;
     //int last_error = 0;
     //void control_base(int pt);
 
     //void duty_control();
     //uint16_t duty = 0;
 
     int16_t PMAX = 36;
     int16_t PMIN = -36;
     int16_t DMAX = 15;
     int16_t DMIN = -15;
     uint8_t FMAX = 1;  
 
     // protected:
 
 
 };
 
 float Motion::Fuzzy_Kp(float P, float D) 
 {
   float PF[2];
   float DF[2];
   uint8_t Pn = 0, Dn = 0;
   uint8_t Un[4];
   float out=0 ; 
   if (P < PMIN)
     P = PMIN;
   if (P > PMAX)
     P = PMAX;
   if (D < DMIN)
     D = DMIN;
   if (D > DMAX)
     D = DMAX;
   if (P > PFF[0] && P < PFF[6])
   {
       if(P<PFF[1])
       {
          Pn=0;
          PF[0]=(FMAX*((float)(PFF[1]-P)/(PFF[1]-PFF[0])));
       }
 
       else if(P<PFF[2])
       {
          Pn=1;
          PF[0]=(FMAX*((float)(PFF[2]-P)/(PFF[2]-PFF[1])));
       }
 
       else if(P<PFF[3])
       {
          Pn=2;
          PF[0]=(FMAX*((float)(PFF[3]-P)/(PFF[3]-PFF[2])));
       }
 
 
       else if(P<PFF[4])
       {
          Pn=3;
          PF[0]=(FMAX*((float)(PFF[4]-P)/(PFF[4]-PFF[3])));
       }
 
 
       else if(P<PFF[5])
       {
          Pn=4;
          PF[0]=(FMAX*((float)(PFF[5]-P)/(PFF[5]-PFF[4])));
       }
 
       else if(P<PFF[6])
       {
          Pn=5;
          PF[0]=(FMAX*((float)(PFF[6]-P)/(PFF[6]-PFF[5])));
       }
 
   }
   else if(P<=PFF[0]) 
   {
       Pn=0;
       PF[0]=FMAX;
   }
   else if(P>=PFF[6])
   {
       Pn=6;
       PF[0]=FMAX;
   }
   PF[1] = (FMAX - PF[0]);
 
 
   if (D > DFF[0] && D < DFF[6])
   {
       if(D<DFF[1])
       {
          Dn=0;
          DF[0]=(FMAX*((float)(DFF[1]-D)/(DFF[1]-DFF[0])));
       }
 
       else if(D<DFF[2])
       {
          Dn=1;
          DF[0]=(FMAX*((float)(DFF[2]-D)/(DFF[2]-DFF[1])));
       }
 
       else if(D<DFF[3])
       {
          Dn=2;
          DF[0]=(FMAX*((float)(DFF[3]-D)/(DFF[3]-DFF[2])));
       }
 
 
       else if(D<DFF[4])
       {
          Dn=3;
          DF[0]=(FMAX*((float)(DFF[4]-D)/(DFF[4]-DFF[3])));
       }
 
 
       else if(D<DFF[5])
       {
          Dn=4;
          DF[0]=(FMAX*((float)(DFF[5]-D)/(DFF[5]-DFF[4])));
       }
 
       else if(D<DFF[6])
       {
          Dn=5;
          DF[0]=(FMAX*((float)(DFF[6]-D)/(DFF[6]-DFF[5])));
       }
 
   }
   else if(D<=DFF[0]) 
   {
       Dn=0;
       DF[0]=FMAX;
   }
   else if(D>=DFF[6])
   {
       Dn=6;
       DF[0]=FMAX;
   }
   DF[1] = (FMAX - DF[0]);
 
   if(Pn<6&&Dn<6)
    {  Un[0]=rule_p[Pn][Dn];
       Un[1]=rule_p[Pn+1][Dn];
       Un[2]=rule_p[Pn][Dn+1];
       Un[3]=rule_p[Pn+1][Dn+1];
 
       out= UFF_P[Un[0]]*(DF[0]*PF[0])+ UFF_P[Un[1]]*(DF[0]*PF[1])+UFF_P[Un[2]]*(DF[1]*PF[0])+UFF_P[Un[3]]*(DF[1]*PF[1]);
    }
   else if(Pn==6&&Dn<6)
   {
       Un[0]=rule_p[Pn][Dn];
       Un[2]=rule_p[Pn][Dn+1];
       out= UFF_P[Un[0]]*(DF[0])+UFF_P[Un[2]]*(DF[1]);
   }
   else if(Dn==6&&Pn<6)
   {
       Un[0]=rule_p[Pn][Dn];
       Un[1]=rule_p[Pn+1][Dn];
       out= UFF_P[Un[0]]*(PF[0])+ UFF_P[Un[1]]*(PF[1]);
   }
   else
   {
       Un[0]=rule_p[Pn][Dn];
       out= UFF_P[Un[0]];
   }
   //std::cout <<"Un[0]" << Un[0] << "Un[1]" << Un[1] << "Un[2]" << Un[2] << "Un[3]" << Un[3] << std::endl;
   //   std::cout <<"KP" << out << std::endl;
   return out;
 
 
 
 
 };
 
 
 
 float Motion::Fuzzy_Kd(float P, float D)  
 {
   float PF[2];
   float DF[2];
     // uint16 UF[4];
   uint8_t Pn = 0, Dn = 0;
   uint8_t Un[4];
     //  int32 temp1, temp2;
   float out=0 ;  
     //  float Un_out[4];
 
   if (P < PMIN)
     P = PMIN;
   if (P > PMAX)
     P = PMAX;
   if (D < DMIN)
     D = DMIN;
   if (D > DMAX)
     D = DMAX;
 
    if (P > PFF[0] && P < PFF[6])
    {
        if(P<PFF[1])
        {
           Pn=0;
           PF[0]=(FMAX*((float)(PFF[1]-P)/(PFF[1]-PFF[0])));
        }
 
        else if(P<PFF[2])
        {
           Pn=1;
           PF[0]=(FMAX*((float)(PFF[2]-P)/(PFF[2]-PFF[1])));
        }
 
        else if(P<PFF[3])
        {
           Pn=2;
           PF[0]=(FMAX*((float)(PFF[3]-P)/(PFF[3]-PFF[2])));
        }
 
 
        else if(P<PFF[4])
        {
           Pn=3;
           PF[0]=(FMAX*((float)(PFF[4]-P)/(PFF[4]-PFF[3])));
        }
 
 
        else if(P<PFF[5])
        {
           Pn=4;
           PF[0]=(FMAX*((float)(PFF[5]-P)/(PFF[5]-PFF[4])));
        }
 
        else if(P<PFF[6])
        {
           Pn=5;
           PF[0]=(FMAX*((float)(PFF[6]-P)/(PFF[6]-PFF[5])));
        }
 
    }
    else if(P<=PFF[0]) 
    {
        Pn=0;
        PF[0]=FMAX;
    }
    else if(P>=PFF[6])
    {
        Pn=6;
        PF[0]=FMAX;
    }
    PF[1] = (FMAX - PF[0]);
 
 
    if (D > DFF[0] && D < DFF[6])
    {
        if(D<DFF[1])
        {
           Dn=0;
           DF[0]=(FMAX*((float)(DFF[1]-D)/(DFF[1]-DFF[0])));
        }
 
        else if(D<DFF[2])
        {
           Dn=1;
           DF[0]=(FMAX*((float)(DFF[2]-D)/(DFF[2]-DFF[1])));
        }
 
        else if(D<DFF[3])
        {
           Dn=2;
           DF[0]=(FMAX*((float)(DFF[3]-D)/(DFF[3]-DFF[2])));
        }
 
 
        else if(D<DFF[4])
        {
           Dn=3;
           DF[0]=(FMAX*((float)(DFF[4]-D)/(DFF[4]-DFF[3])));
        }
 
 
        else if(D<DFF[5])
        {
           Dn=4;
           DF[0]=(FMAX*((float)(DFF[5]-D)/(DFF[5]-DFF[4])));
        }
 
        else if(D<DFF[6])
        {
           Dn=5;
           DF[0]=(FMAX*((float)(DFF[6]-D)/(DFF[6]-DFF[5])));
        }
 
    }
    else if(D<=DFF[0])
    {
        Dn=0;
        DF[0]=FMAX;
    }
    else if(D>=DFF[6])
    {
        Dn=6;
        DF[0]=FMAX;
    }
    DF[1] = (FMAX - DF[0]);
 
 
    if(Pn<6&&Dn<6)
     {  Un[0]=rule_d[Pn][Dn];
        Un[1]=rule_d[Pn+1][Dn];
        Un[2]=rule_d[Pn][Dn+1];
        Un[3]=rule_d[Pn+1][Dn+1];
 
        out= UFF_D[Un[0]]*(DF[0]*PF[0])+ UFF_D[Un[1]]*(DF[0]*PF[1])+UFF_D[Un[2]]*(DF[1]*PF[0])+UFF_D[Un[3]]*(DF[1]*PF[1]);
     }
 
    else if(Pn==6&&Dn<6)
    {
        Un[0]=rule_d[Pn][Dn];
        Un[2]=rule_d[Pn][Dn+1];
        out= UFF_D[Un[0]]*(DF[0])+UFF_D[Un[2]]*(DF[1]);
    }
 
    else if(Dn==6&&Pn<6)
    {
        Un[0]=rule_d[Pn][Dn];
        Un[1]=rule_d[Pn+1][Dn];
        out= UFF_D[Un[0]]*(PF[0])+ UFF_D[Un[1]]*(PF[1]);
    }
    else
    {
        Un[0]=rule_d[Pn][Dn];
        out= UFF_D[Un[0]];
    }
    return out;
 }
 
 
 