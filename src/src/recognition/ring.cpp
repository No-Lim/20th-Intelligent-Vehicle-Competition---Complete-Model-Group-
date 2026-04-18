#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "tracking.cpp"
#include "crossroad.cpp"

using namespace cv;
using namespace std;

extern Mat  imgBinary;
class Ring
{
    public:
    // 变量定义
    uint8_t left_cout = 0, right_cout = 0;
    float circle_route; //环岛积分
    bool left_ring = false, right_ring = false;
    int none_left_line = 0, none_right_line = 0;
    int have_left_line = 0, have_right_line = 0;
    // int circle_count = 0;
    int Count_dis_Flag=0;

    int stop_cout = 0;

    // bool quanbai_col(Mat frame);
    // bool quanbai_row(Mat frame);

    bool quanhei_row_right(Mat frame);
    bool quanhei_row_left(Mat frame);

    //函数
    void check_circle(Tracking &IMG,Mat &imgBinary);

    void run_circle(Tracking &IMG, Mat &imgBinary,Crossroad &cross);
    void run_circle_old(Tracking &IMG, Mat &imgBinary);
    double check_circle(int line1[][2], int num);
};





void Ring::check_circle(Tracking &IMG, Mat &imgBinary) 
{

    // 非环岛, 单边 L 角点, 单边长直道
    //左圆环
    if (IMG.Lpt0_rpts0s_id<25 && flag_circle == CIRCLE_NONE && (IMG.Lpt0_found && !IMG.Lpt1_found && IMG.is_straight1 )/*&&AT_IMAGE(imgBinary, clip(IMG.tu0_x - 5, 0, imgBinary.cols - 1), IMG.tu1_y-5) != 0*/ /*&& left_cout == 0*/ ) 
    {
        // if(quanhei_row_right(imgBinary))
        // {
            //&&IMG.Lpt0_rpts0s_id>18
            if(AT_IMAGE(imgBinary, IMG.tu0_x-5, IMG.tu0_y-5) == 0)
            {
                cout<<"二值退出环岛"<<endl;
                flag_circle=CIRCLE_NONE;
            }
            stop_cout=0;
            none_left_line = 0;
            have_left_line = 0;
            circle_route = IMG.encoder.route;//开始记路程
            flag_circle = CIRCLE_LEFT_BEGIN;
            cout<<"CIRCLE_LEFT_BEGIN"<<endl;
    }  
 


//右圆环
    if (cirle_1 && flag_circle == CIRCLE_NONE && (!IMG.Lpt0_found && IMG.Lpt1_found && IMG.is_straight0) &&IMG.Lpt1_rpts1s_id<15&&AT_IMAGE(imgBinary, IMG.tu1_x+5, IMG.tu1_y-5) != 0 ) 
    {
    
        // if(AT_IMAGE(imgBinary, IMG.tu1_x+5, IMG.tu1_y+5) == 0)
        // {
        //     cout<<"二值退出环岛"<<endl;
        //     flag_circle=CIRCLE_NONE;
        // }
        stop_cout=0;
            none_right_line = 0;
            have_right_line = 0;
            circle_route = IMG.encoder.route;//开始记路程
            flag_circle = CIRCLE_RIGHT_BEGIN;
            cout<<"CIRCLE_RIGHT_BEGIN"<<endl;   
   
    }

double Ring::check_circle(int line1[][2], int num)
{
    if (num < 5)  // 如果边线点太少，直接返回1000表示边线不可靠
    {
        return 1000;
    }
    vector<int> v_slope;
    int step = 10; // 每隔10个点计算一次斜率
    for (int i = 10; i < num; i += step)
    {
        if (line1[i][0] - line1[i - step][0])
            v_slope.push_back((line1[i][1] - line1[i - step][1]) * 100 / (line1[i][0] - line1[i - step][0]));
    }
    if (v_slope.size() > 1)
    {
        double sum = accumulate(begin(v_slope), end(v_slope), 0.0);
        double mean = sum / v_slope.size(); // 均值
        double accum = 0.0;
        for_each(begin(v_slope), end(v_slope), [&](const double d)
                 { accum += (d - mean) * (d - mean); });

        return sqrt(accum / (v_slope.size() - 1)); // 方差
    }
    else
        return 0;
}

void Ring::run_circle(Tracking &IMG,Mat &imgBinary,Crossroad &cross) 
{
    if (flag_circle == CIRCLE_LEFT_BEGIN) 
    {
        if(!angle_check&&stop_cout==0)
        {
            stop_cout++;
            flag_circle=CIRCLE_NONE;
        }
        //cout<<"总路程:"<<IMG.encoder.route<<"环岛积分"<<circle_route<<endl;
        flag_track = TRACK_RIGHT;
        //先丢左线后有线
        if (IMG.rpts0s_num  <2&&!IMG.Lpt0_found) { Count_dis_Flag=1;none_left_line++; have_left_line = 0;}  //丢线标志位开启，开始记录编码器距离
        if (IMG.rpts0s_num >30&&none_left_line )  have_left_line ++;                                    //经历了一个先丢先再有线的过程，再次出现边线标志位标志位开启
        if (flag_circle == CIRCLE_LEFT_BEGIN&&IMG.rpts0s_num  <  36 &&/*total_distence>circle_in_distance&&*/(have_left_line>2) &&( IMG.encoder.route-circle_route >= 35)) //开始记路程) 
        {
            //当搜到内环&&内环边线长度小于某个长度（小于的意思是虽然之前我可能搜到了内环，过早切换到内环会非常切内，单轮出界
            //但是这时我仍可以巡外侧长直道行进，但边线长度会随着车往前跑而逐渐变短，当短到一定地步时入环时机合适）&&编码累积了一定的行进距离
            //这些条件可多可少，最好根据自身车速和摄像头视野灵活修改
                flag_circle = CIRCLE_LEFT_IN;cross.if_lost_right_line =0;
                none_left_line = 0;
                have_left_line = 0;
                //if_clean_pid = 1;//变积分PID开启标志位
                Count_dis_Flag=0;
                //还原一些边线标志位，并跳转到CIRCLE_LEFT_IN状态
                cout<< "CIRCLE_LEFT_IN" <<endl;
        }



    }
    //入环，寻内圆左线
    else if (flag_circle == CIRCLE_LEFT_IN) {
        flag_track = TRACK_LEFT;
       IMG.aim_distance_f=0.3;
       IMG.aim_distance_n=0.7;
        if(IMG.rpts0s_num<25)Count_dis_Flag=1;
        if(IMG.rpts1s_num < 5)none_right_line++;         //右侧长直道丢失
        if(flag_circle == CIRCLE_LEFT_IN&&IMG.rpts1s_num >10&&none_right_line>1){                       //右侧经历一个先丢线再有线的过程，表示车身已经进入圆环内了，跳转至CIRCLE_LEFT_RUNNING，清理还原标志位
            flag_circle = CIRCLE_LEFT_RUNNING;
            circle_route = IMG.encoder.route;//开始记路程
            Count_dis_Flag=0;
            none_right_line = 0;
            cout<<"CIRCLE_LEFT_RUNNING"<<endl;
        }
        
    }
    //正常巡线，寻外圆右线
    else if (flag_circle == CIRCLE_LEFT_RUNNING) {
        flag_track = TRACK_RIGHT;
        if (IMG.Lpt1_found) {
            //接近出环时,拐点截断处理
            IMG.rpts1s_num = IMG.Lpt1_rpts1s_id-2;
            IMG.rptsc1_num = IMG.Lpt1_rpts1s_id-2;
        }
        //满足拐点足够靠近近点时切换到内环循迹
        if ( IMG.Lpt1_found&&((IMG.Lpt1_rpts1s_id < 0.3/ sample_dist))&&(IMG.encoder.route-circle_route >= 200)) {
            flag_circle = CIRCLE_LEFT_OUT;
            Count_dis_Flag=0;
            cross.if_lost_right_line =0;
            //if_clean_pid = 1;
            cout<<"CIRCLE_LEFT_OUT"<<endl;
        }
    }
    //出环，寻内圆
    else if (flag_circle == CIRCLE_LEFT_OUT) {
       // cout<<"总路程:"<<IMG.encoder.route<<"环岛积分"<<circle_route<<endl;
        //小圆容易看不到内侧的环，因此使用cross_farline_R去找赛道对侧的长直道，即可进行补线操作
        //cross.cross_farline_R(IMG);
        flag_track = TRACK_LEFT;
        Count_dis_Flag=1;
        if(IMG.rpts1s_num < 5)           none_right_line++; 
        //cout<<"总路程:"<<IMG.encoder.route<<"环岛积分"<<circle_route<<endl;                                         //右侧经历拐点消失后丢线标志位启动
        if((IMG.rpts1s_num>20&&!IMG.Lpt1_found&&none_right_line>1)/*&&(IMG.encoder.route-circle_route >= 50)*/)                   //右侧重新出现长直道边线或者编码器累计一定的长度强制跳出
        {
            flag_circle = CIRCLE_LEFT_END;
            none_right_line= 0;
            Count_dis_Flag=0;
            cout<<"CIRCLE_LEFT_END"<<endl;
            circle_route = IMG.encoder.route;
        }
    }
        //走过圆环，寻右线
    else if (flag_circle == CIRCLE_LEFT_END) {
        flag_track = TRACK_RIGHT;
        //broadcast_flag=1;
        Count_dis_Flag=1;
        if(IMG.encoder.route - circle_route > 150)//再次记录100
        {
        //此时的条件非常简单，寻外环长直道记录一定的长度防止再次触发圆环标志位，清理还原标志位
        flag_circle = CIRCLE_NONE;
        //road_type = ROAD_NORMAL;
        begin_y=BEGIN_Y;
        Count_dis_Flag=0;
        //aim_distance = AIM_DISTENCE;
        //is_large_circle = is_small_circle = 0;
        cross.if_lost_right_line =0;
        cross.if_lost_left_line = 0;
        circle_count++;
        none_right_line = 0;
        have_right_line = 0;
        none_left_line = 0;
        have_left_line = 0;
        circle_route = 0;
        angle_check=false;
        }
    }
        //右圆环同理
    else if (flag_circle == CIRCLE_RIGHT_BEGIN) {

        if(IMG.is_straight1)
        {
            flag_circle=CIRCLE_NONE;
        }

        if(!angle_check&&stop_cout==0)
        {
            cout<<angle_check<<endl;
            flag_circle=CIRCLE_NONE;
            
            stop_cout++;
        }
        flag_track = TRACK_LEFT;
        if (IMG.rpts1s_num < 10&&!IMG.Lpt1_found) { Count_dis_Flag=1;none_right_line++; have_right_line = 0;}
        if (IMG.rpts1s_num>18&&none_right_line)have_right_line++;
        if (flag_circle == CIRCLE_RIGHT_BEGIN&&have_right_line && (IMG.encoder.route-circle_route >=  95))//&&IMG.rpts1s_num  <  70)//(0.2 / sample_dist )<rpts1s_num&&rpts1s_num < (0.4 / sample_dist )&&
        {
            flag_circle = CIRCLE_RIGHT_IN;
            cross.if_lost_left_line = 0;
            none_right_line = 0;
            have_right_line = 0;
            //circle_encoder = current_encoder;
            Count_dis_Flag=0;
            //if_clean_pid = 1;
            cout<<"CIRCLE_RIGHT_IN"<<endl;
        }
    }
    else if (flag_circle == CIRCLE_RIGHT_IN)
     {
        if(IMG.is_straight1)
        {
            flag_circle=CIRCLE_NONE;
        }
        flag_track = TRACK_RIGHT;
        IMG.aim_distance_f=0.3;
        IMG.aim_distance_n=0.7;
        if(IMG.rpts1s_num<35)Count_dis_Flag=1;
        if(IMG.rpts0s_num < 5)none_left_line++;
        if(flag_circle == CIRCLE_RIGHT_IN&&IMG.rpts0s_num >25&&none_left_line>1){
            flag_circle = CIRCLE_RIGHT_RUNNING; Count_dis_Flag=0;
            circle_route = IMG.encoder.route;//开始记路程
            //begin_y=BEGIN_Y;
            none_left_line = 0;
            cout<<"CIRCLE_RIGHT_RUNNING"<<endl;
        }
    }
    else if (flag_circle == CIRCLE_RIGHT_RUNNING) {
        flag_track = TRACK_LEFT;
        if (IMG.Lpt0_found) {
            IMG.rpts0s_num = IMG.Lpt0_rpts0s_id-2;
            IMG.rptsc0_num = IMG.Lpt0_rpts0s_id-2;
        }
        if (IMG.Lpt0_found && ((IMG.Lpt0_rpts0s_id < 0.5 / sample_dist))&&(IMG.encoder.route-circle_route>=200)) 
        {//||rpts1s_num>=5
            flag_circle = CIRCLE_RIGHT_OUT;
            cross.if_lost_left_line = 0;
            //if_clean_pid = 1;
            cout<<"CIRCLE_RIGHT_OUT"<<endl;
            circle_angle_flage=false;
        }
    }
    else if (flag_circle == CIRCLE_RIGHT_OUT) {
        //cross.cross_farline_L(IMG);
        IMG.aim_distance_f=0.2;
        IMG.aim_distance_n=0.8;
        flag_track = TRACK_RIGHT;
        //broadcast_flag=1;
        if(IMG.rpts0s_num < 5){
            none_left_line++;
            Count_dis_Flag=1;
        }
        if(IMG.is_straight0)
        {
            flag_circle=CIRCLE_NONE;
        }
        if((IMG.rpts0s_num>30&&!IMG.Lpt0_found&&none_left_line>=1)/*&&(IMG.encoder.route-circle_route>=150)*/)
        {
            cout<<"end"<<endl;
            flag_circle = CIRCLE_RIGHT_END;
            none_left_line= 0;
            Count_dis_Flag=0;
        }
        circle_route = IMG.encoder.route;
    }
    else if (flag_circle == CIRCLE_RIGHT_END) {
        flag_track = TRACK_LEFT;
        Count_dis_Flag=1;
        if (IMG.rpts1s_num < 0.2 / sample_dist) { none_right_line++;Count_dis_Flag=1; }
        if(IMG.encoder.route - circle_route > 150)
        {
            cout<<"右圆环结束"<<endl;
            flag_circle = CIRCLE_NONE;
            //flag_circle= CIRCLE_NONE;
            //road_type = ROAD_NORMAL;

            begin_y=BEGIN_Y;
            Count_dis_Flag=0;
            //aim_distance = AIM_DISTENCE;
            //is_large_circle = is_small_circle = 0;
            cross.if_lost_right_line =0;
            cross.if_lost_left_line = 0;
            circle_count++;
            none_right_line = 0;
            have_right_line = 0;
            none_left_line = 0;
            have_left_line = 0;
            circle_route = 0;
            angle_check=false;
        }
    }
}





void Ring::run_circle_old(Tracking &IMG ,Mat &imgBinary) 
{
    if(left_ring)
    {        
        // 入左环, 寻内圆左线
        if (flag_circle == CIRCLE_LEFT_IN) 
        {
            //cout<<"CIRCLE_LEFT_IN"<<endl;
            bool running = false;
            flag_track = TRACK_LEFT;
            // 先丢右线后有线
            if (IMG.rpts1s_num < 0.2 / SAMPLE_DIST)   //10个点
                none_right_line++;
            
            if (IMG.rpts1s_num > 0.8 / SAMPLE_DIST && none_right_line > 0)   //40个点
            {
                    have_right_line++;
                if (have_right_line > 0)
                {
                    none_right_line = 0;
                    have_right_line = 0;
                    running = true;
                }
            }
            // 左侧丢线,右侧线少                           
            if (IMG.rpts0s_num < 20 && IMG.rpts1s_num > 50 && none_right_line > 0)
                running = true;

            if (running) 
            {
                none_right_line = 0;
                have_right_line = 0;
                flag_circle = CIRCLE_LEFT_RUNNING;
                cout<<"CIRCLE_LEFT_RUNNING"<<endl;
                running = false;
            }
        }


        // 正常巡线, 寻外圆右线
        if(flag_circle == CIRCLE_LEFT_RUNNING) 
        {

            flag_track = TRACK_RIGHT;
            // 外环存在拐点, 可再加拐点距离判据 (左L点)
            if(IMG.Lpt1_found)
                IMG.ipts1_num = IMG.rpts1s_num = IMG.rptsc1_num = IMG.Lpt1_rpts1s_id;
            if(IMG.Lpt1_found && IMG.Lpt1_rpts1s_id < 0.6 / SAMPLE_DIST  )   
            {
                flag_circle = CIRCLE_LEFT_OUT;
                cout<<"CIRCLE_LEFT_OUT"<<endl;
            }
        }



        // 出环, 寻内圆
        if(flag_circle == CIRCLE_LEFT_OUT) 
        {
            //cout<<"CIRCLE_LEFT_OUT"<<endl;
            flag_track = TRACK_LEFT;           //？？？？？？？？？？？？？？？？？？？？？？？？？

            // bool flag2 = false;
            // flag2 = quanbai_row(imgBinary);

            if(IMG.Lpt1_found)
                IMG.ipts1_num = IMG.rpts1s_num = IMG.rptsc1_num = IMG.Lpt1_rpts1s_id;

            // 先丢右线后有线
            if(IMG.rpts1s_num < 0.2 / SAMPLE_DIST)    //10
                none_right_line++;
            if(IMG.rpts1s_num > 1.0 / SAMPLE_DIST && none_right_line > 0)    //50
            {
                //7.1增加了第二次看到左角点结束out状态&& IMG.Lpt0_found
                if (have_right_line > 0) 
                {
                    none_right_line = 0;
                    have_right_line = 0;
                    flag_circle = CIRCLE_LEFT_OUT_NONE;                 //7.2添加---防止出左环二次看到角点进入begin状态
                    cout<<"CIRCLE_LEFT_OUT_NONE"<<endl;

                    flag_track = TRACK_RIGHT;                      
                    if(IMG.is_straight0 && IMG.is_straight1)
                    {
                        IMG.element_over = true;
                        flag_circle = CIRCLE_NONE;
                        cirle_1 = false;

                        left_cout = 1;
                        left_ring = false;
                    }

                }
                else 
                {
                    have_right_line++;
                }
            }
        }


        if(flag_circle == CIRCLE_LEFT_OUT_NONE && left_ring)        //在此状态内二次找角点
        {
            flag_track = TRACK_RIGHT;  
            //cout<<"CIRCLE_LEFT_OUT_NONE"<<endl;
            if(IMG.is_straight0 && IMG.is_straight1) 
            {
                IMG.element_over = true;
                flag_circle = CIRCLE_NONE;
                cout<<"CIRCLE_NONE"<<endl;


                cirle_1 = false;
                 // first = true;

                left_ring = false;
                left_cout = 1;
            }
        }
    }
/* ***************************** 右圆环************************************ */

    if(right_ring)
    {      
        // 入右环, 寻内圆右线
        if(flag_circle == CIRCLE_RIGHT_IN) 
        {
            bool running2 = false;
            //cout<<"CIRCLE_RIGHT_IN"<<endl;
            flag_track = TRACK_RIGHT;
           
            // 先丢左线后有线
            if(IMG.rpts0s_num < 0.2 / SAMPLE_DIST)
                none_left_line++;
            if(IMG.rpts0s_num > 0.8 / SAMPLE_DIST && none_left_line > 0) 
            {
                    have_left_line++;
                if(have_left_line > 0) 
                {
                    none_left_line = 0;
                    have_left_line = 0;
                    running2 = true;
                    flag_circle = CIRCLE_RIGHT_RUNNING;
                }
            }
            // 右侧丢线,左侧线少                                 
            if (IMG.rpts1s_num < 20 && IMG.rpts0s_num > 50 && none_left_line > 0)
                running2 = true;

            if (running2) 
            {
                none_left_line = 0;
                have_left_line = 0;
                flag_circle = CIRCLE_RIGHT_RUNNING;
                cout<<"CIRCLE_RIGHT_RUNNING"<<endl;
                running2 = false;
            }
        }

        // 正常巡线, 寻外圆左线
        else if(flag_circle == CIRCLE_RIGHT_RUNNING) 
        {
            // cout<<"CIRCLE_RIGHT_RUNNING"<<endl;
            flag_track = TRACK_LEFT;

            // 外环存在拐点, 可再加拐点距离判据 (左L点)
            if(IMG.Lpt0_found)
                IMG.ipts0_num = IMG.rpts0s_num = IMG.rptsc0_num = IMG.Lpt0_rpts0s_id;
            if(IMG.Lpt0_found && IMG.Lpt0_rpts0s_id < 0.6 / SAMPLE_DIST ) 
            {
                flag_circle = CIRCLE_RIGHT_OUT;
                cout<<"CIRCLE_RIGHT_OUT"<<endl;
            }
        }



        // 出环, 寻内圆
        else if(flag_circle == CIRCLE_RIGHT_OUT) 
        {
            //cout<<"CIRCLE_RIGHT_OUT"<<endl;
            flag_track = TRACK_RIGHT;

            if(IMG.Lpt0_found)
                IMG.ipts0_num = IMG.rpts0s_num = IMG.rptsc0_num = IMG.Lpt0_rpts0s_id;

            // 先丢左线后有线
            if(IMG.rpts0s_num < 0.2 / SAMPLE_DIST)
                none_left_line++;
            if(IMG.rpts0s_num > 1.0 / SAMPLE_DIST && none_left_line > 0 ) 
            {
                //7.1增加了第二次看到右角点结束out状态 && IMG.Lpt1_found
                if (have_left_line > 0) 
                {
                    none_left_line = 0;
                    have_left_line = 0;

                    flag_circle = CIRCLE_RIGHT_OUT_NONE;                //7.2添加---防止出右环二次看到角点进入begin状态
                    cout<<"CIRCLE_RIGHT_OUT_NONE"<<endl;
                    flag_track = TRACK_LEFT;

                    if(IMG.is_straight0 && IMG.is_straight1)
                    {
                        IMG.element_over = true;
                        flag_circle = CIRCLE_NONE;

                        cirle_1 = false;

                        right_cout = 1;
                        right_ring = false;
                    }
                } 
                else 
                {
                    have_left_line++;
                }
            }
        }

        if(flag_circle == CIRCLE_RIGHT_OUT_NONE && right_ring)//7.2添加-----在此状态内二次找角点
        {
            //cout<<"CIRCLE_RIGHT_OUT_NONE"<<endl;
            flag_track = TRACK_LEFT;
            if(IMG.is_straight0 && IMG.is_straight1)
            {
                IMG.element_over = true;
                flag_circle = CIRCLE_NONE;

                cirle_1 = false;
                 // first = true;

                right_cout = 1;
                right_ring = false;
                cout<<"CIRCLE_NONE"<<endl;
            }
        }
    }
}

bool Ring::quanhei_row_right(Mat frame)
{
	uint16_t col = 296 ;
    int16_t baidian = 0;
	for(int16_t row = 0; row < IMAGE_HEIGHT - 1; row++)
	{
		//if (frame.at<uchar>(i, lie) == 255)
        if(AT_IMAGE(frame, col, row) == 0)
		{
			baidian++;
		}
	}
	 
	if(baidian >= 190)
	{
		return true;
	}
	else 
		return false;
}


bool Ring::quanhei_row_left(Mat frame)
{
	uint8_t col = 3 ;
    int16_t baidian = 0;
	for(int16_t row = 0; row < IMAGE_HEIGHT - 1; row++)
	{
		//if (frame.at<uchar>(i, lie) == 255)
        if(AT_IMAGE(frame, col, row) == 0)
		{
			baidian++;
		}
	}
	 
	if(baidian >= 190)
	{
		return  true;
	}
	else 
		return false;
}
