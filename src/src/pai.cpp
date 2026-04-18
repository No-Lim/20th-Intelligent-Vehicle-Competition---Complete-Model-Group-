
 #include <opencv2/opencv.hpp>
 #include <iostream>
 using namespace cv;
 using namespace std;
 
 int main()
 {
     // 打开默认摄像头
     VideoCapture cap(0);
     if (!cap.isOpened()) {
         cout << "无法打开摄像头" << endl;
         return -1;
     }
     Mat frame;
     int photo_count = 0;
     while (true) {
         cap >> frame; // 拍照
         if (frame.empty()) {
             cout << "未捕获到图像" << endl;
             break;
         }
         imshow("Camera", frame); // 显示
         int key = waitKey(30);
         if (key == 27) // 按ESC退出
             break;
         if (key == 32) { // 按空格键拍照
             string filename = "photo_" + to_string(photo_count++) + ".jpg";
             imwrite(filename, frame);
             cout << "已保存: " << filename << endl;
         }
     }
     cap.release();
     destroyAllWindows();
     return 0;
 }
 
 
 
 
 
 
 