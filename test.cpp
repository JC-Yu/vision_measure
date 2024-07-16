#include <iostream>
#include "cct.h"
using namespace std;

int main(int argc, char *argv[])
{
    // 读取配置文件信息
    cv::FileStorage fs("../config.yaml", cv::FileStorage::READ);
    const string camera = fs["Camera"];
    const int N = fs["Bit_N"];
    const double R_th = fs["R_threshold"];
    fs.release();
    // 打开摄像头
    int camera_index = 0;
    if(camera == "Web_Cam")
        camera_index = 2;
    cv::VideoCapture cap(camera_index);
    // 检查摄像头是否成功打开
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera" << std::endl;
        return -1;
    }
    cv::Mat frame;
    while (true) {
        // 从摄像头捕获一帧图像
        cap >> frame;
        // 检查是否成功捕获到图像
        if (frame.empty()) {
            std::cerr << "Error: Could not grab a frame" << std::endl;
            break;
        }
        // 执行CCT解码程序
        cct::CCTExtractor ce(frame, N, R_th);
        ce.extract();
        vector<cct::Result> result_list = ce.getExtractionResult(); 
        // 如果按下ESC键，则退出循环
        if (cv::waitKey(30) == 27) {
            break;
        }
    }
    // 释放摄像头
    cap.release();
    // 销毁所有窗口
    cv::destroyAllWindows();
    return 0;
}
