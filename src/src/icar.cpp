#include "../include/common.hpp"     //公共类方法文件
#include "../include/detection.hpp"  //百度Paddle框架移动端部署
#include "../include/uart.hpp"       //串口通信驱动
#include "motion.cpp"                //智能车运动控制类
#include "preprocess.cpp"            //图像预处理类
#include "recognition/crossroad.cpp" //十字道路识别与路径规划类
#include "recognition/ring.cpp"      //环岛道路识别与路径规划类
#include "recognition/tracking.cpp"  //赛道识别基础类
#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include <signal.h>
#include <unistd.h> 
#include <chrono>
#include <vector>
#include "detection/_obstacle.cpp"      //AI检测：障碍物
#include "detection/_catering.cpp"//AI检测：人行道（没有使用）
#include "detection/_ramp.cpp"      //AI检测：坡道
#include "detection/_layby.cpp"      //AI检测：坡道
#include "detection/_parker.cpp"     //AI检测：停车区
#include "detection/_crosswalk.cpp"//AI检测：人行道（没有使用）
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
using namespace std;
using namespace cv;
using namespace std::chrono;

std::shared_ptr<SerialDriver> serial = nullptr;       // 线程 -> 串口接收及发送
Tracking tracking;        // 赛道识别类
void check_garage(Mat &IMG,Tracking &A) ;
void draw_on_original(Mat &bianxian,Tracking &tracking,Crossroad &cross,Obstacle &obstacle);
void draw_on_perspective(Mat &bianxian,Tracking &tracking,Crossroad &cross);
// 设置终端为非阻塞模式
void set_nonblocking_input(bool enable) {
  termios tty;
  tcgetattr(STDIN_FILENO, &tty);
  if (enable) {
      tty.c_lflag &= ~(ICANON | ECHO); // 关闭缓冲和回显
      tcsetattr(STDIN_FILENO, TCSANOW, &tty);
      fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK); // 非阻塞
  } else {
      tty.c_lflag |= ICANON | ECHO;
      tcsetattr(STDIN_FILENO, TCSANOW, &tty);
      fcntl(STDIN_FILENO, F_SETFL, 0);
  }
}
// 检查是否按了 q
bool check_quit_key() {
  char ch;
  if (read(STDIN_FILENO, &ch, 1) == 1) {
      if (ch == 'q' || ch == 'Q') {
          return true;
      }
  }
  return false;
}

int main(int argc, char const *argv[]) 
{
  //uint8_t brake_cout = 0;
  char fps_buffer[40];  // 帧率图像显示缓存
  char fps_buffer1[40];   //角度
  char fps_buffer_l[40]; 
  char fps_buffer_r[40];
  char fps_buffer_mode[40];
  char pid_P[40], pid_D[40];

  Preprocess preprocess;    // 图像预处理类
  Motion motion;            // 运动控制类
  AI_flag = motion.params.debug;  //是否显示ai画框


  //AI元素
  Obstacle obstacle;      //避障检测类
  RAMP ramp;              // 坡道区检测类
  Catering catering;      //餐饮区检测类
  LAYBY layby;

  PARKER parker;
  CROSSWALK crosswalk;

  //十字，圆环
  Crossroad cross;          // 十字道路识别类
  Ring circle;              // 环岛识别类
  tracking.have_danger = motion.params.danger;    //是否检测避障
  tracking.have_ramp  =  motion.params.bridge;      //是否检测坡道
  tracking.have_burger =  motion.params.burger;   //是否检测餐饮区
  tracking.have_layby = motion.params.layby;
  tracking.have_battery = motion.params.battery;
  tracking.parker_aim_angle_filter = motion.params.aim_angle_filter;

  //目标检测类(AI模型文件)
  shared_ptr<DetectionPredictor> detection = make_shared<DetectionPredictor>();
  detection->init(motion.params.model);

  /* 串口 */
  Payload_t payload;      // 数据包 
  serial = std::make_shared<SerialDriver>("/dev/ttyUSB0", BaudRate::BAUD_115200);
  int ret = serial->open();
  if (ret != 0) 
  {
   printf("[Error] Uart Open failed!\n");
   return -1;
  }
    /* 摄像头 */
  std::shared_ptr<CaptureInterface> capture = std::make_shared<CaptureInterface>();   // 线程 -> 摄像头
  capture->init();
  //capture->setExposure(-6); // 设置曝光值为-6，具体数值可根据实际摄像头调整
  capture->start();  
  detection->start();
  // 初始化参数
  Mat imgCorrect;//矫正的图像，目前未使用
  Mat imgBinary;//二值图像
  //cv::Mat black = cv::Mat::zeros(200, 300, CV_8UC3);

  tracking.aim_distance_f = motion.params.aim_distance_f;
  tracking.aim_distance_n = motion.params.aim_distance_n;

  
    for(int j = 0;j<3;j++)
    {
      waitKey(500);
    }
  cout <<" 启动 "<<endl;
int cnt=0;
set_nonblocking_input(true); // 程序开始时调用一次
  long preTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
  while (1) 
  {
    //cnt++;

    //cout<<cnt<<"    "<<flag_burger<<"     "<<flag_obstacle<<endl;
    //cout<<flag_burger<<endl;
    // 获取图像
    capture->get(imgCorrect);
    // 目标检测
    detection->run(imgCorrect);
    // //[02] 图像预处理
    imgBinary = preprocess.binaryzation(imgCorrect); // 图像二值化
    image_filter(imgBinary);                      //图像滤波

    //图像画黑边
    //image_draw_rectan(imgBinary);

    //巡线 元素前处理
    tracking.trackRecognition(imgBinary);



    if (check_quit_key()) {
      flag_protect = PROTECT_STOP;
      cout << "[INFO] 收到退出指令，停车！" << endl;
  }
    
    // 重置识别元素开始标志
    tracking.element_identify = 0;
    if(tracking.element_over) 
    { 
      if(flag_burger == BURGER_NONE&&flag_circle== CIRCLE_NONE&&flag_cross==CROSS_NONE)
      {
       obstacle.check_obstacle(tracking,imgBinary);//检测障碍物
      }         
      if(flag_obstacle == OBSTACLE_NONE &&flag_circle== CIRCLE_NONE )
      {
        catering.check_burger(tracking);

      }
      if(flag_obstacle == OBSTACLE_NONE )
      {
        layby.check_layby(tracking);

      }

      if(flag_obstacle == OBSTACLE_NONE &&flag_layby == LAYBY_NONE&& (flag_circle < CIRCLE_LEFT_IN)/*&&jump_parker*/)
      {
        parker.check_parker(tracking);   //检测停车区  
 
      }
   
      if(flag_obstacle == OBSTACLE_NONE &&flag_layby == LAYBY_NONE&& flag_parker == PARKER_NONE && (flag_circle < CIRCLE_LEFT_IN)&&flag_block==BLOCK_NONE)   //检测坡道
      {
        ramp.check_ramp(tracking); 
      
      }
        if(flag_obstacle == OBSTACLE_NONE &&flag_layby == LAYBY_NONE&& flag_parker == PARKER_NONE && (flag_circle < CIRCLE_LEFT_IN)&&flag_block==BLOCK_NONE&&tracking.encoder.route>200)   //检测坡道)   //检测斑马线停车
        {
          crosswalk.check_crosswalk(tracking);

        }

        //添加flag_cross == CROSS_NONE后不会再crossin里判断circle这里添加了flag_cross == CROSS_NONE判断条件                                                 //cross是十字                                    
        if(motion.params.ring &&flag_layby == LAYBY_NONE&& flag_obstacle == OBSTACLE_NONE &&  flag_parker == PARKER_NONE  && flag_ramp == RAMP_NONE && flag_crosswalk == CROSSWALK_NONE 
          && flag_cross == CROSS_NONE&&flag_burger == BURGER_NONE &&tracking.encoder.route>150&&circle_count<=1&&flag_block==BLOCK_NONE)   
        {
          circle.check_circle(tracking, imgBinary);  // 环岛路 - 识别角点特征

        }
      
        if(motion.params.cross &&flag_layby == LAYBY_NONE&& flag_obstacle == OBSTACLE_NONE && flag_ramp == RAMP_NONE  && flag_parker == PARKER_NONE &&  flag_crosswalk == CROSSWALK_NONE && flag_circle== CIRCLE_NONE &&flag_block==BLOCK_NONE)  
        {  
          cross.check_Half(tracking);
        }

    }
/***********************************元素执行函数****************************************/   
    if(flag_obstacle != OBSTACLE_NONE&&flag_obstacle!=OBSTACLE_DETECTION)      //出现障碍                 
    {
      obstacle.run_obstacle(tracking, imgBinary);
      //obstacle.run_block(tracking, imgBinary);
      //cout<<"ooo"<<endl;
    }  

    else if (flag_ramp != RAMP_NONE&&flag_ramp!=RAMP_DETECTION)       //坡道
    {
       ramp.run_ramp(tracking,motion.params.ramp_up_route,motion.params.ramp_down_route);   
       //cout<<"dasd"<<endl;
    }
 
    else if(flag_burger != BURGER_NONE/*&&flag_burger!=BURGER_DETECTION*/)
    {
      //cout<<cnt<<endl;

      catering.run_burger(tracking);
      
    }
   
    else if(flag_layby != LAYBY_NONE&&flag_layby!=LAYBY_DETECTION)
    {
      layby.run_layby(tracking);
    }
    else if(flag_parker != PARKER_NONE&&flag_parker!=PARKER_DETECTION)     //停车区
    {
      parker.run_parker(tracking,motion.params.parker_far,motion.params.parker_near,cross,motion.params.parker_far_route_in,motion.params.parker_far_route_turn);
    }

    if (flag_circle != CIRCLE_NONE&&!flag_cross&&!flag_block)   //圆环
    {
      circle.run_circle(tracking, imgBinary,cross);
    }
    if (flag_cross != CROSS_NONE&&!flag_circle&&!flag_block)    //十字
    {
      //cross.run_buxian_cross(tracking) ;
      cross.run_cross(tracking);
      //cross.Detectionline(imgBinary,tracking);
    }
    // if(!flag_cross&&!flag_circle&&flag_block!=BLOCK_NONE)
    // {
    //   cout<<"执行v="<<endl;
    //   cross.run_block(tracking);
    // }

    
    if(flag_crosswalk != CROSSWALK_NONE)    //斑马线停车
    {
      crosswalk.run_crosswalk(tracking);
    }



//中线跟踪
   // 十字的CROSS_IN和CROSS_HALF和车库的GARAGE_FOUND_LEFT和GARAGE_FOUND_RIGHT都是使用远线控制通过，这样的方法可以使得十字不需要补线，车库不需要对斑马线进行任何操作就能通过
    if (flag_cross != CROSS_IN&&flag_cross != CROSS_HALF /*garage_type !=  GARAGE_FOUND_LEFT/&&garage_type !=GARAGE_FOUND_RIGHT*/ ) 
  {
    if(flag_cross==CROSS_NONE&&!start_bu)
    {
        //一般情况下的左右线切换到中线
        if (flag_track == TRACK_LEFT) {
          tracking.rpts = tracking.rptsc0;
            tracking.rpts_num = tracking.rptsc0_num;
        } else {
          tracking.rpts = tracking.rptsc1;
            tracking.rpts_num = tracking.rptsc1_num;
        }
    }

    else if(parker_right&&move_flag&&start_bu)
    {
      tracking.rpts = tracking.Splicing_leftline_center;
      tracking.rpts_num = tracking.Splicing_leftline_center_num;
    }
    else if(parker_left&&move_flag&&start_bu)
    {
      tracking.rpts = tracking.Splicing_rightline_center;
      tracking.rpts_num = tracking.Splicing_rightline_center_num;
    }

  }

  else {
    //十字和车库根据远线控制
    if (flag_track == TRACK_LEFT) {
        if (cross.far_rpts0s_num > 0 && cross.far_Lpt0_rpts0s_id >= 0 && cross.far_Lpt0_rpts0s_id < cross.far_rpts0s_num) {
            //对拐点id之前的点做截断处理舍弃掉
            int points_count = range_limit(cross.far_rpts0s_num - cross.far_Lpt0_rpts0s_id - 1, 0, cross.far_rpts0s_num);
            if (points_count > 0) {
                tracking.rpts = tracking.rptsc0;  // 使用已分配的数组
                track_leftline(cross.far_rpts0s + cross.far_Lpt0_rpts0s_id, points_count, tracking.rptsc0,
                          (int) round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
                tracking.rpts_num = points_count;
            }
        }
    }

    if(flag_track == TRACK_RIGHT) {
        if (cross.far_rpts1s_num > 0 && cross.far_Lpt1_rpts1s_id >= 0 && cross.far_Lpt1_rpts1s_id < cross.far_rpts1s_num) {
            int points_count = cross.far_rpts1s_num - cross.far_Lpt1_rpts1s_id;
            if (points_count > 0) {
                tracking.rpts = tracking.rptsc1;  // 使用已分配的数组
                track_rightline(cross.far_rpts1s + cross.far_Lpt1_rpts1s_id, points_count, tracking.rptsc1,
                            (int) round(angle_dist / sample_dist), pixel_per_meter * ROAD_WIDTH / 2);
                tracking.rpts_num = points_count;
            }
        }
    }
}
    //巡线 元素后处理      
    tracking.trackprocess();
    //计算舵机值将数据传入下位机
    motion.poseCtrl(tracking.aim_angle_filter, payload);
    //发送速度
   motion.speedCtrl(payload,tracking);//电机
   serial->sendPack(payload);//发送数据




    if(motion.params.debug)
    {
      Mat bianxian;
      bianxian = imgCorrect.clone();
      Mat test;
      test = imgCorrect.clone();
      cv::Mat black = cv::Mat::zeros(200, 300, CV_8UC3);
      draw_on_perspective(black,tracking,cross);
      draw_on_original(bianxian,tracking,cross,obstacle);
      //打印帧率
      long startTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
      // printf(">> FrameTime: %ldms | %.2ffps \n", startTime - preTime, 1000.0 / (startTime - preTime));
      double fps = (double)1000.0 / (startTime - preTime);
      sprintf(fps_buffer, "FPS: %.2f", fps);
      cv::putText(bianxian, (std::string)fps_buffer, Point(5, 10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255));  
      preTime = startTime;
      //打印转角
      sprintf(fps_buffer1, "angle: %.2f", tracking.aim_angle_filter);
      cv::putText(bianxian,  (std::string)fps_buffer1, Point(5, 30),  cv::FONT_HERSHEY_SIMPLEX, 0.4 , cv::Scalar(0, 0, 255));
      //左值线
      sprintf(fps_buffer_l, "is_straight0: %d", tracking.is_straight0);
      cv::putText(bianxian,  (std::string)fps_buffer_l, Point(5, 50),  cv::FONT_HERSHEY_SIMPLEX, 0.4 , cv::Scalar(0, 0, 255));
      //右直线
      sprintf(fps_buffer_r, "is_straight1: %d", tracking.is_straight1);
      cv::putText(bianxian,  (std::string)fps_buffer_r, Point(5, 70),  cv::FONT_HERSHEY_SIMPLEX, 0.4 , cv::Scalar(0, 0, 255));
      //巡线模式
      sprintf(fps_buffer_mode, "flag_track: %d", flag_track);
      cv::putText(bianxian,  (std::string)fps_buffer_mode, Point(5, 90),  cv::FONT_HERSHEY_SIMPLEX, 0.4 , cv::Scalar(0, 0, 255));
      cv::imshow("b",imgBinary);
    int canvas_width = bianxian.cols + black.cols; // 水平拼接
    int canvas_height = std::max(bianxian.rows, black.rows); // 垂直方向最大高度
    cv::Mat canvas = cv::Mat::zeros(canvas_height, canvas_width, bianxian.type());

    // 将图像复制到画布的不同位置
    bianxian.copyTo(canvas(cv::Rect(0, 0, bianxian.cols, bianxian.rows))); // 第一幅图像在左侧
    black.copyTo(canvas(cv::Rect(bianxian.cols, 0, black.cols, black.rows))); // 第二幅图像在右侧

    // 创建窗口并设置大小和位置
    std::string _win_name = "fuck car";
    cv::namedWindow(_win_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(_win_name, 600, 400);
    cv::moveWindow(_win_name, 0, 0);
    //cv::imshow("as",detection->_result->org_frame);
    // 显示合成后的图像
    cv::imshow(_win_name, canvas);
      waitKey(10);
    }    

  }
  // 结束线程
  detection->stop();
  serial->close();
  capture->stop();
  return 0;
}

void check_garage(Mat &tracking,Tracking &A) 
{
    int16_t black_blocks = 0;
    int16_t cursor = 0;
    int16_t times = 0;
    static int8_t flag = 0;
    static int8_t flag_1 = 0;

    for (int16_t y = 70; y < 190; y++) 
    {
        black_blocks = 0;
        cursor = 0;  // 指向栈顶的游标
        for (int16_t x = COLSIMAGE / 2 - 150; x < COLSIMAGE / 2 + 150; x++) 
        {
            if (tracking.at<uint8_t>(y,x) == 0) //黑
            {
                if (cursor < 20)
                    cursor++;
            } 
            else 
            {
                if (cursor >= 4 && cursor <= 8)
                    black_blocks++;
                cursor = 0;
            }
        }
        if (black_blocks >= 5 && black_blocks <= 9)
            times++;
    }
    if (times >= 5 && flag_1 !=1) 
    {
      flag_crosswalk = CROSSWALK_DETECTION;
      flag = A.car_stop;
      A.element_identify = 1;
      
      cout <<"准备停车"<<endl;
    }
    if(flag_crosswalk == CROSSWALK_DETECTION && flag !=0)
    {
      flag-=1;
      if(flag == 1)
      {
        flag_crosswalk = CROSSWALK_START;
      }
    }
}


void draw_on_original(Mat &bianxian,Tracking &tracking,Crossroad &cross,Obstacle &obstacle)
{
      cv::circle(bianxian, cv::Point(tracking.ipts0[60][0], tracking.ipts0[60][1]), 4, cv::Scalar(0, 255, 0), -1);  

      cv::circle(bianxian, cv::Point(tracking.ipts0[180][0], tracking.ipts0[180][1]), 4, cv::Scalar(0, 255, 0), -1);  

      cv::circle(bianxian, cv::Point(tracking.ipts1[60][0], tracking.ipts1[60][1]), 4, cv::Scalar(0, 255, 0), -1);  

      cv::circle(bianxian, cv::Point(tracking.ipts1[180][0], tracking.ipts1[180][1]), 4, cv::Scalar(0, 255, 0), -1);  


      //左中线
      for (int i = 0; i < tracking.ipts0_num; i++)  
      {
        cv::circle(bianxian, cv::Point(tracking.ipts0[i][0], tracking.ipts0[i][1]), 1, cv::Scalar(255, 0, 255), -1);  
        
      }
      for (int i = 0; i < cross.far_ipts0_num; i++)  
      {
        cv::circle(bianxian, cv::Point(cross.far_ipts0[i][0], cross.far_ipts0[i][1]), 1, cv::Scalar(255, 0, 255), -1);  
        
      }
      //右中线
      for (int i = 0; i < tracking.ipts1_num; i++) 
      {
        cv::circle(bianxian, cv::Point(tracking.ipts1[i][0], tracking.ipts1[i][1]), 1, cv::Scalar(0, 0, 255), -1);
      }
      for (int i = 0; i < cross.far_ipts1_num; i++) 
      {
        cv::circle(bianxian, cv::Point(cross.far_ipts1[i][0], cross.far_ipts1[i][1]), 1, cv::Scalar(0, 0, 255), -1);
      }

      //左角点
      if(tracking.Lpt0_found)
      {
        cv::circle(bianxian, cv::Point(tracking.tu0_x, tracking.tu0_y), 8, cv::Scalar(255, 0, 255), 4);
      }

      //右角点  
      if(tracking.Lpt1_found)
      {
        cv::circle(bianxian, cv::Point(tracking.tu1_x, tracking.tu1_y), 8, cv::Scalar(255, 0, 255), 4);
      }
      cv::circle(bianxian, cv::Point(cross.far_point0_xie[0], cross.far_point0_xie[1]), 8, cv::Scalar(0,69,255), -1);
      cv::circle(bianxian, cv::Point(cross.far_point1_xie[0], cross.far_point1_xie[1]), 8, cv::Scalar(139,139,0), -1);
      cv::circle(bianxian, cv::Point(cross.test0[0], cross.test0[1]), 1, cv::Scalar(139,139,0), -1);//深绿
          //左角点
      if(cross.far_Lpt0_found)
      {
        cv::circle(bianxian, cv::Point(cross.inv_far_Lpt0_found[0], cross.inv_far_Lpt0_found[1]), 8, cv::Scalar(255, 255, 255), -1);
      }

      //右角点  
      if(cross.far_Lpt1_found)
      {
        cv::circle(bianxian, cv::Point(cross.tu1_x_cross, cross.tu1_y_cross), 4, cv::Scalar(255, 255, 255), -1);
      }

      //贝塞尔拟合远锚点 2/3
      cv::circle(bianxian, cv::Point(tracking.bezier_line[tracking.bezier_line.size()*2/3].x, tracking.bezier_line[tracking.bezier_line.size()*2/3].y), 4, cv::Scalar(255, 0, 255), 4);


      //圆环补线
      if(flag_circle==CIRCLE_RIGHT_IN)
      {
          if(cross.far_Lpt1_found) {
              drawline(bianxian,tracking.Splicing_leftline_num,tracking.Splicing_leftline,tracking.inv_Splicing_leftline,0);
          }
      }
      else if(flag_circle==CIRCLE_RIGHT_OUT||flag_circle==CIRCLE_RIGHT_END)
      {
          if(tracking.Lpt0_found&&cross.far_rpts0s_num>0) {
              drawline(bianxian,tracking.leftline_num,tracking.leftline,tracking.inv_leftline,0);
          }
          else{
              if (tracking.rpts0s_num < 0.6 / sample_dist){
                  drawline(bianxian,tracking.leftline_num,tracking.leftline,tracking.inv_leftline,0);
              }
          }
      }
      else if (flag_circle==CIRCLE_LEFT_IN)
      {
          if(cross.far_Lpt0_found) {
              drawline(bianxian,tracking.Splicing_rightline_num,tracking.Splicing_rightline,tracking.inv_Splicing_rightline,0);
          }
      }
      else if(flag_circle==CIRCLE_LEFT_OUT||flag_circle==CIRCLE_LEFT_END)
      {
          if(tracking.Lpt1_found) {
              drawline(bianxian,tracking.Splicing_rightline_center_num,tracking.Splicing_rightline_center,tracking.inv_Splicing_rightline,0);
              drawline(bianxian,tracking.rightline_num,tracking.rightline,tracking.inv_rightline,0);
          }
          else{
              if (tracking.rpts1s_num < 0.6 / sample_dist){
                  drawline(bianxian,tracking.rightline_num,tracking.rightline,tracking.inv_rightline,0);
              }

          }
      }
}



void draw_on_perspective(Mat &black,Tracking &tracking,Crossroad &cross)
{


  for(int i=0;i<tracking.leftline_num;i++)
  {
    cv::circle(black,cv::Point(tracking.leftline[i][0],tracking.leftline[i][1]),1, cv::Scalar(0, 255, 0), -1);
  }

  for(int i=0;i<tracking.rightline_num;i++)
  {
    cv::circle(black,cv::Point(tracking.rightline[i][0],tracking.rightline[i][1]),1, cv::Scalar(0, 255, 0), -1);
  }


        //贝塞尔中线
        for (size_t i = 0; i < tracking.bezier_line.size(); i++)
        {
          cv::circle(black, cv::Point(tracking.bezier_line[i].x, tracking.bezier_line[i].y), 1, cv::Scalar(0, 0, 255), 4);  //红色
        }




cv::circle(black, cv::Point(tracking.rpts0s[60][0], tracking.rpts0s[60][1]), 4, cv::Scalar(0,0,255), -1);  //粉色
cv::circle(black, cv::Point(tracking.rpts1s[80][0], tracking.rpts1s[80][1]), 4, cv::Scalar(0,0,255), -1);  //粉色
  for(int i = 0;i<tracking.rpts0s_num;i++)
  {
    cv::circle(black, cv::Point(tracking.rpts0s[i][0], tracking.rpts0s[i][1]), 2, cv::Scalar(255,255,255), -1);  //粉色
    //cout<<"yes1"<<endl;

  }
  for(int i = 0;i<tracking.rpts1s_num;i++)
  {
    cv::circle(black, cv::Point(tracking.rpts1s[i][0], tracking.rpts1s[i][1]), 1, cv::Scalar(255, 255, 255), -1);  //黄色
    //cout<<"yes2"<<endl;

  }

  //左角点
  if(tracking.Lpt0_found)
  {
    //cout<<"yes4"<<endl;

    cv::circle(black, cv::Point(tracking.rpts0s[tracking.Lpt0_rpts0s_id][0], tracking.rpts0s[tracking.Lpt0_rpts0s_id][1]), 3, cv::Scalar(255, 0, 255), -1);
  }

  //右角点  
  if(tracking.Lpt1_found)
  {
    cv::circle(black, cv::Point(tracking.rpts1s[tracking.Lpt1_rpts1s_id][0], tracking.rpts1s[tracking.Lpt1_rpts1s_id][1]), 3, cv::Scalar(255, 0, 255), -1);
  }


  //远线
  for (int i = 0; i < cross.far_rpts0_num; i++)  
  {
    cv::circle(black, cv::Point(cross.far_rpts0[i][0], cross.far_rpts0[i][1]), 1, cv::Scalar(0, 0, 255), -1);  
    //cout<<cross.far_rpts0_num<<endl;

  }
  for (int i = 0; i < cross.far_rpts1_num; i++)  
  {
    cv::circle(black, cv::Point(cross.far_rpts1[i][0], cross.far_rpts1[i][1]), 1, cv::Scalar(0, 0, 255), -1);  
    //cout<<"yes6"<<endl;

  }

  //左角点
  if(cross.far_Lpt0_found)
  {
    //cout<<"yes7"<<endl;

    cv::circle(black, cv::Point(cross.far_rpts0s[cross.far_Lpt0_rpts0s_id][0], cross.far_rpts0s[cross.far_Lpt0_rpts0s_id][1]), 3, cv::Scalar(255, 0, 255), -1);
  }

  //右角点  
  if(cross.far_Lpt1_found)
  {
    //cout<<"yes8"<<endl;

    cv::circle(black, cv::Point(cross.far_rpts1s[cross.far_Lpt1_rpts1s_id][0], cross.far_rpts1s[cross.far_Lpt1_rpts1s_id][1]), 3, cv::Scalar(255, 0, 255), -1);
  }
  //圆环补线
  if(flag_circle==CIRCLE_RIGHT_IN)
  {
      if(cross.far_Lpt1_found) {
          drawline(black,tracking.Splicing_leftline_num,tracking.Splicing_leftline,tracking.inv_Splicing_leftline,1);
          drawline(black,tracking.Splicing_leftline_center_num,tracking.Splicing_leftline_center,tracking.inv_Splicing_leftline,1);
      }
  }
  else if(flag_circle==CIRCLE_RIGHT_OUT||flag_circle==CIRCLE_RIGHT_END)
  {
      if(tracking.Lpt0_found&&cross.far_rpts0s_num>0) {
          drawline(black,tracking.leftline_num,tracking.leftline,tracking.inv_leftline,1);
          drawline_purple(black,tracking.Splicing_leftline_center_num,tracking.Splicing_leftline_center,tracking.inv_Splicing_leftline,1);
      }
      else{
          if (tracking.rpts0s_num < 0.6 / sample_dist){
              drawline(black,tracking.leftline_num,tracking.leftline,tracking.inv_leftline,1);
          }
      }
  }
  else if (flag_circle==CIRCLE_LEFT_IN)
  {
      if(cross.far_Lpt0_found) {
          drawline(black,tracking.Splicing_rightline_center_num,tracking.Splicing_rightline_center,tracking.inv_Splicing_rightline,1);
          drawline(black,tracking.Splicing_rightline_num,tracking.Splicing_rightline,tracking.inv_Splicing_rightline,1);
      }
  }
  else if(flag_circle==CIRCLE_LEFT_OUT||flag_circle==CIRCLE_LEFT_END)
  {
      if(tracking.Lpt1_found) {
          drawline_purple(black,tracking.Splicing_rightline_center_num,tracking.Splicing_rightline_center,tracking.inv_Splicing_rightline,1);
          drawline(black,tracking.rightline_num,tracking.rightline,tracking.inv_rightline,1);
      }
      else{
          if (tracking.rpts1s_num < 0.6 / sample_dist){
              drawline(black,tracking.rightline_num,tracking.rightline,tracking.inv_rightline,1);
          }

      }
 }
}




