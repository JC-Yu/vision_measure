#include <iostream>
#include "cct.h"
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
            map<int, cv::Point2f> results = ce.getExtractionResult(); 
            for(auto r : results) {
                cout << "==> id = " << r.first << " , center = " << r.second << endl;
            }
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
        cct::CCTExtractor ce(image, N, R_th);
        ce.extract();
        map<int, cv::Point2f> res = ce.getExtractionResult(); 
        for(auto r : res) {
            cout << "==> id = " << r.first << " , center = " << r.second << endl;
        }
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
        vector<map<int, cv::Point2f>> results_list;
        for(int i=0; i<nums; ++i) {
            cv::Mat image = cv::imread(dir + to_string(i+1) + format);
            cct::CCTExtractor ce(image, N, R_th);
            ce.extract();
            map<int, cv::Point2f> res = ce.getExtractionResult(); 
            for(auto r : res) {
                cout << "==> id = " << r.first << " , center = " << r.second << endl;
            }
            images.emplace_back(image);
            results_list.emplace_back(res);
            cv::waitKey();
        }
        cct::PointPairs pps = cct::cctMatching(images[0], images[1], results_list[0], results_list[1]);
        cout << "==> Found points pair nums = " << pps.size() << endl;
        cv::waitKey();
    }
    return 0;
}
