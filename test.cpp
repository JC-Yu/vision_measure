#include <iostream>
#include "cct.h"
#include "img_trans.h"
using namespace std;

int main(int argc, char *argv[])
{
    // 读取配置文件信息
    cv::FileStorage fs("../config.yaml", cv::FileStorage::READ);
    const int N = fs["Bit_N"];
    const double R_th = fs["R_threshold"];
    const string inputFormat = fs["InputFormat"];
    // 视频输入格式
    if(inputFormat == "Video") {
        const string camera = fs["Camera"];
        fs.release();
        // 打开摄像头
        int camera_index = 0;
        if(camera == "Web_Cam")
            camera_index = 2;
        cout << "==> Camera initializing..." << endl;
        cv::VideoCapture cap(camera_index);
        // 检查摄像头是否成功打开
        if (!cap.isOpened()) {
            std::cerr << "Error: Could not open camera" << std::endl;
            return -1;
        }
        cout << "==> Camera is ready" << endl;
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
            vm::CCTExtractor ce(frame, N, R_th);
            cv::Mat image_extraction = ce.extract();
            cv::imshow("result", image_extraction);
            // 如果按下ESC键，则退出循环
            if (cv::waitKey(30) == 27) {
                break;
            }
        }
        // 释放摄像头
        cap.release();
        // 销毁所有窗口
        cv::destroyAllWindows();
    }
    // 单张图像输入格式
    if(inputFormat == "Image") {
        const string imagePath = fs["ImagePath"];
        cv::Mat image = cv::imread(imagePath);
        vm::CCTExtractor ce(image, N, R_th);
        cv::Mat image_extraction = ce.extract();
        cv::imwrite("cct_extraction_" + std::to_string(vm::CCTExtractor::counter) + ".jpg", image_extraction);
        map<int, cv::Point2f> res = ce.getExtractionResult(); 
        cv::waitKey();
    }
    // 多张图像输入格式
    if(inputFormat == "MultiView") {
        const int nums = fs["ImageNums"];
        const string format = fs["ImageFormat"];
        string dir = fs["ImagesDir"];
        if(dir.back() != '/')
            dir += '/';
        vector<cv::Mat> images;
        vm::Stich2Images sticher(N, R_th);  // 定义图像拼接器（用于拼接两张图像）
        for(int i=0; i<nums; ++i) {
            images.emplace_back(cv::imread(dir + to_string(i+1) + format));
        }
        cout << "==> Image loaded, nums = " << images.size() << endl;
        // 进行迭代式的图像拼接
        cout << "==> Start image stiching..." << endl;
        vector<cv::Mat> stiched_list = images;
        while(stiched_list.size() != 1) {
            int N = stiched_list.size();
            int count = N / 2;
            vector<cv::Mat> temp;
            for(int i=0; i<count; ++i) {
                cv::Mat result = sticher(stiched_list[2 * i], stiched_list[2 * i + 1]);
                temp.push_back(result);
            }
            if(N % 2 == 1)
                temp.push_back(stiched_list[N - 1]);
            stiched_list = temp;
        }
        cv::imwrite("imageStiched.jpg", stiched_list[0]);
        cout << "==> Stiching complete!" << endl;
    }
    return 0;
}
