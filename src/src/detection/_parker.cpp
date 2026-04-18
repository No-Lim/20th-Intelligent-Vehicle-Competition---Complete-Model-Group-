#include <algorithm>
#include "../recognition/tracking.cpp"
#include "../../include/detection.hpp"
#include "../../include/common.hpp"
#include "../recognition/crossroad.cpp"

class PARKER
{
public:     
    // 变量定义
    int count_bat = 0;
    int count_car = 0;
    int stop_cnt = 0;
    int tuichu_cnt = 0;
    int route = 0;
    int lucheng = 0;
    int y_threshold = 50;
    int route_buxian =0;
    bool near = false;
    bool far = false;
    bool flag_bubuxian =false;
    int cnt = 0;
    int cnt_buxian =0;
    PredictResult parker_resultObs;
    PredictResult car_resultObs;
    PredictResult car_resultObs_runing;
    
    // 函数
    void check_parker(Tracking &IMG);
    void run_parker(Tracking &IMG, bool parker_far, bool parker_near,Crossroad &cross,int parker_far_route_in,int parker_far_route_turn);

    float threshold = 5; // 判断充电车位置阈值
    float far_in_threshold = 150; // 远车位进入时机


};

void PARKER::check_parker(Tracking &IMG) 
{
    if (flag_parker == PARKER_DETECTION) 
    {
        cout << "PARKER_READY" << endl; 
        IMG.element_identify = 0;
        flag_parker = PARKER_READY;
        IMG.element_over = false;
    }
}

void PARKER::run_parker(Tracking &IMG, bool parker_far, bool parker_near,Crossroad &cross,int parker_far_route_in,int parker_far_route_turn)
{
    // 新增：检测到car时重置路程计数
    bool car_detected = false;

    for (size_t i = 0; i < com_predictor_results.size(); i++) {
        if(com_predictor_results[i].label == "car") {
            car_detected = true;
            car_resultObs = com_predictor_results[i]; // 记录最新car位置
            break;
        }
    }
    
    // 如果在RUNING状态且检测到car，重置路程计数
    if(flag_parker == PARKER_RUNING && car_detected&&parker_far) {
        lucheng = IMG.encoder.route;
        cout << "检测到car，重置路程计数为: " << lucheng << endl;
    }

    if(flag_parker == PARKER_READY)
    {
        for (size_t i = 0; i < com_predictor_results.size(); i++)
        {
            if ((com_predictor_results[i].label == "battery"))
            {
                parker_resultObs = com_predictor_results[i];
            }
        }

        // 路牌位置判断
        if((parker_resultObs.x + parker_resultObs.width < 150) && (parker_resultObs.x != 0) && parker_resultObs.y > 10)
        { 
            cout << "左       " << parker_resultObs.x << endl;
            flag_track = TRACK_RIGHT;
            if(!parker_left) {
                parker_left = true;
            }
            flag_parker = PARKER_IN;
        }

        if((parker_resultObs.x + parker_resultObs.width > 150) && (parker_resultObs.x != 0) && parker_resultObs.y > 10)
        {
            cout << "右       " << parker_resultObs.x << endl;
            flag_track = TRACK_LEFT;
            if(!parker_right) {
                parker_right = true; 
            }
            flag_parker = PARKER_IN;
        }
    }

    // 进入RUNING状态判断
    if(flag_parker == PARKER_IN && parker_far)
    {
        count_bat = 0;
        count_car = 0;
        for (size_t i = 0; i < com_predictor_results.size(); i++)
        {
            if(com_predictor_results[i].label == "battery") count_bat++;
            if(com_predictor_results[i].label == "car") count_car++;
        }
        
        if((count_bat == 0) && (count_car >= 1))
        {
            flag_parker = PARKER_RUNING;
            cout << "切入running" << endl;
        }
    }
    else if(flag_parker == PARKER_IN && parker_near)
    {
        flag_parker = PARKER_RUNING;
        cout << "切入running" << endl;
    }

    // 车库执行逻辑
    if(flag_parker == PARKER_RUNING)
    {
        // 检查是否应该退回IN状态
        for (size_t i = 0; i < com_predictor_results.size(); i++)
        {
            if(com_predictor_results[i].label == "battery" && parker_near)
            {
                flag_parker = PARKER_IN;
                cout << "         切回到in" << endl;
                near = false;
                far = false;
                break;
            }
        }

        // 真正处理RUNING状态
        if(flag_parker == PARKER_RUNING)
        {
            // 确定进入近库还是远库
            if((parker_far == 1 && !far))
            {
                far = true;
                lucheng = IMG.encoder.route;
                cout << "进远库，记录基准路程: " << lucheng << endl;
            }
            else if(parker_near == 1)
            {
                near = true;
                cout << "进近库" << endl;
            }

            // 近库处理
            if(near)
            {
                IMG.aim_distance_f = 0.3;
                if(parker_left)
                {
                    parker_near_moveFlag_L = true;
                    flag_track = TRACK_LEFT;
                    if((check_line_overlap(IMG.rpts0s, IMG.rpts1s, IMG.rpts0s_num, IMG.rpts1s_num)) /*&& 
                       (IMG.Lpt0_found && IMG.Lpt0_rpts0s_id < 60) || 
                       (IMG.Lpt1_found && IMG.Lpt1_rpts1s_id < 60)*/)
                    {
                        flag_parker = PARKER_STOP;
                        cout << "         stop" << endl;
                    }
                }
                else
                {
                    parker_near_moveFlag_R = true;
                    flag_track = TRACK_RIGHT;
                    if((check_line_overlap(IMG.rpts0s, IMG.rpts1s, IMG.rpts0s_num, IMG.rpts1s_num)))
                    {
                        flag_parker = PARKER_STOP;
                        cout << "         stop" << endl;
                    }
                }
            }
            
            // 远库处理
            if(far)
            {
                IMG.aim_distance_f = 0.3;
                if(parker_left)
                {
                    cout << "当前路程差: " << IMG.encoder.route - lucheng << endl;
                    if(IMG.encoder.route - lucheng < parker_far_route_in)
                        {
                            flag_track = TRACK_RIGHT;
                            car_left=true;
                        }
                        else
                        {
    
                            cout<<"             补线"<<endl;
                            move_flag=true;
                            cross.cross_farline_L(IMG);
                            if(cross.far_Lpt0_found)
                            {
                              point_Cal_Line(tracking.rpts1s[2][0],tracking.rpts1s[2][1],cross.far_rpts0s[cross.far_Lpt0_rpts0s_id][0],cross.far_rpts0s[cross.far_Lpt0_rpts0s_id][1],tracking.rightline,&tracking.rightline_num); 
                              Splicing_array(tracking.rightline,tracking.rightline_num, cross.far_rpts0s, cross.far_Lpt0_rpts0s_id,tracking.Splicing_rightline, &tracking.Splicing_rightline_num,1);
                              tracking.Splicing_rightline_s0s_num = sizeof(tracking.Splicing_rightline_s0s) / sizeof(tracking.Splicing_rightline_s0s[0]);
                              resample_points(tracking.Splicing_rightline, tracking.Splicing_rightline_num, tracking.Splicing_rightline_s0s, &tracking.Splicing_rightline_s0s_num, sample_dist * pixel_per_meter);
                              track_rightline(tracking.Splicing_rightline_s0s, tracking.Splicing_rightline_s0s_num, tracking.Splicing_rightline_center, (int) round(10.0), pixel_per_meter * ROAD_WIDTH / 2-5);//
                              tracking.Splicing_rightline_center_num = tracking.Splicing_rightline_s0s_num;
                              start_bu = true;
                              cnt_buxian++;
                            }
                            if(cnt_buxian==1)
                            {
                                route_buxian=IMG.encoder.route;
                                flag_bubuxian =true;
                            }
    
                            
                            flag_track = TRACK_RIGHT;
                        }
                        if(/*(check_line_overlap_far(IMG.rpts0s, IMG.rpts1s, IMG.rpts0s_num, IMG.rpts1s_num)) &&
                            (point_dis_line(IMG.rpts1s[IMG.Lpt1_rpts1s_id], IMG.rpts0s[IMG.Lpt0_rpts0s_id]) <= 25)*/ /*(IMG.Lpt0_found&&IMG.Lpt0_rpts0s_id<10)||*/
                            (IMG.encoder.route - lucheng > 145)||(IMG.encoder.route-route_buxian>75&&flag_bubuxian))
                         {
                             flag_parker = PARKER_STOP;
                             cout << "         stop" << endl;
                             move_flag=false;
                             start_bu=false;
                         }
                }
                else
                {
                    cout << "当前路程差: " << IMG.encoder.route - lucheng << endl;
                    cout << "补线路程差: " << IMG.encoder.route - route_buxian << endl;
                    if(IMG.encoder.route - lucheng < 30)
                    { 
                        flag_track = TRACK_LEFT;
                        car_right = true;
                    }
                    else
                    {


                        move_flag=true;
                        cross.cross_farline_R(IMG);
                        if(cross.far_Lpt1_found)
                        {
                          point_Cal_Line(tracking.rpts0s[5][0],tracking.rpts0s[5][1],cross.far_rpts1s[cross.far_Lpt1_rpts1s_id][0],cross.far_rpts1s[cross.far_Lpt1_rpts1s_id][1],tracking.leftline,&tracking.leftline_num); 
                          Splicing_array(tracking.leftline,tracking.leftline_num, cross.far_rpts1s, cross.far_Lpt1_rpts1s_id,tracking.Splicing_leftline, &tracking.Splicing_leftline_num,1);
                          tracking.Splicing_leftline_s1s_num = sizeof(tracking.Splicing_leftline_s1s) / sizeof(tracking.Splicing_leftline_s1s[0]);
                          resample_points(tracking.Splicing_leftline, tracking.Splicing_leftline_num, tracking.Splicing_leftline_s1s, &tracking.Splicing_leftline_s1s_num, sample_dist * pixel_per_meter);
                          track_leftline(tracking.Splicing_leftline_s1s, tracking.Splicing_leftline_s1s_num, tracking.Splicing_leftline_center, (int) round(10.0), pixel_per_meter * ROAD_WIDTH / 2 -5);//
                          tracking.Splicing_leftline_center_num = tracking.Splicing_leftline_s1s_num;
                          start_bu = true;
                          cnt_buxian++;
                        }
                        if(cnt_buxian==1)
                        {
                            route_buxian=IMG.encoder.route;
                            flag_bubuxian =true;
                        }

                        
                        flag_track = TRACK_LEFT;
                    }

                    if(/*(check_line_overlap_far(IMG.rpts0s, IMG.rpts1s, IMG.rpts0s_num, IMG.rpts1s_num)) &&
                       (point_dis_line(IMG.rpts1s[IMG.Lpt1_rpts1s_id], IMG.rpts0s[IMG.Lpt0_rpts0s_id]) <= 25)*/ /*(IMG.Lpt0_found&&IMG.Lpt0_rpts0s_id<10)||*/
                       (IMG.encoder.route - lucheng > 135)||(IMG.encoder.route-route_buxian>70&&flag_bubuxian))
                    {
                        flag_parker = PARKER_STOP;
                        cout << "         stop" << endl;
                        move_flag=false;
                        start_bu=false;
                    }
                }
            }
        }
    }

    // 停车状态处理
    if(flag_parker == PARKER_STOP)
    {
        stop_cnt++;
        if(stop_cnt >= 25)
        {
            if(near)
            flag_parker = PARKER_OUT;
            else
            flag_parker = PARKER_FAR_OUT0;

            
        }
        route = IMG.encoder.route;
    }

    // 出库状态处理
    if(flag_parker == PARKER_OUT)
    {
        //68是刚好直线
        //75偏一点,更稳一点
       if((IMG.encoder.route - route) <= (-85))
       {
            flag_parker = PARKER_END;
       }
    }

    if(flag_parker == PARKER_FAR_OUT0)
    {
        //之前是-5
       if((IMG.encoder.route - route) <= (-12))
       {
            flag_parker = PARKER_FAR_OUT1;
       }
    }
    
    if(flag_parker == PARKER_FAR_OUT1)
    {
       if((IMG.encoder.route - route) <= (-72))
       {
            flag_parker = PARKER_END;
       }
    }
    // 结束状态处理
    if(flag_parker == PARKER_END)
    {
        if(parker_left)
        {
            flag_track = TRACK_RIGHT;
        }
        else if(parker_right)
        {
            flag_track = TRACK_LEFT;
        }
        cout<<IMG.encoder.route - route<<endl;
        tuichu_cnt++;
        if(((IMG.encoder.route - route) >= 50)||tuichu_cnt > 100)
        {
            flag_parker = PARKER_NONE;
            cout << "停车场状态结束" << endl;
            IMG.element_over = true;
            route = 0;
            parker_left = false;
            parker_right = false;
            stop_cnt = 0;
            car_down = false;
            car_up = false;
            near = false;
            far = false;
            move_flag=false;
            flag_bubuxian =false;
            route_buxian = 0;
            cnt_buxian=0;
            parker_first=true;
        }
    }
}