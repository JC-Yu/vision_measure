#include "cct.h"

namespace cct
{
    // CCT编码提取和识别函数
    void CCTExtractor::extract(void)
    {
        // 1. 图像预处理
        cv::Mat image_clone = image.clone();
        cv::Mat image_gray, image_binary;
        cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);    // 灰度化
        double otsuThreshold = cv::threshold(image_gray, image_binary, 0, 255, cv::THRESH_OTSU | cv::THRESH_BINARY);    // otsu二值化

        // 2. 提取轮廓
        std::vector<std::vector<cv::Point2i>> contours;
        cv::findContours(image_binary, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

        // 3. 进行CCT编码识别与解码
        for(auto& contour : contours) {
            // (1) 对提取的轮廓进行圆度阈值筛选
            /*
                阈值计算公式及筛选原则：
                        2 * sqrt(PI * A)
                    R = ————————————————
                                C
                其中，A是轮廓面积，C是轮廓周长。当 R >= R_th 时，认为轮廓圆度符合要求。
            */ 
            double area = cv::contourArea(contour, false);
            double length = cv::arcLength(contour, true);
            double R = 2 * sqrtf64(M_PI * area) / (length);
            if(R < R_th || length < 20)
                continue;
            // (2) 对筛选后的轮廓进行椭圆拟合，拟合结果存放在box中，box1，box2，box3分别是由内而外的三个同心椭圆
                // box数据结构：[椭圆中心点，长轴和短轴，倾斜角]
            cv::RotatedRect box1 = cv::fitEllipse(contour);     // 内环
            cv::RotatedRect box2(box1.center, cv::Size2f(box1.size.width*2, box1.size.height*2), box1.angle);   // 中环
            cv::RotatedRect box3(box1.center, cv::Size2f(box1.size.width*3, box1.size.height*3), box1.angle);   // 外环
            double a = std::max<double>(box3.size.width, box3.size.height);     // 外环大椭圆的长轴
            double cx = box3.center.x;  // 中心点横坐标
            double cy = box3.center.y;  // 中心点纵坐标
            double theta = box3.angle;  // 椭圆倾斜角
            // (3) 从原图中切割出CCT区域
            cv::Mat cct_roi;
            int row_min = std::round(cy - a/2);
            int row_max = std::round(cy + a/2);
            int col_min = std::round(cx - a/2);
            int col_max = std::round(cx + a/2);
            if(row_min>=0 && row_max<image.rows && col_min>=0 && col_max<image.cols) {
                cct_roi = image_clone.rowRange(row_min, row_max).colRange(col_min, col_max);    // 切割区域，得到roi
                // 计算原图与roi之间的坐标偏移量
                double dx = cx - a/2;
                double dy = cy - a/2;
                // (4) 进行仿射变换，将椭圆变换成正圆
                cv::Point2f rect_ori[4];
                box3.points(rect_ori);
                // （变换前）椭圆最小外接矩形的四个顶点+椭圆中心点
                std::vector<cv::Point2f> src = {
                    cv::Point2f(rect_ori[0].x-dx, rect_ori[0].y-dy),
                    cv::Point2f(rect_ori[1].x-dx, rect_ori[1].y-dy),
                    cv::Point2f(rect_ori[2].x-dx, rect_ori[2].y-dy),
                    cv::Point2f(rect_ori[3].x-dx, rect_ori[3].y-dy),
                    cv::Point2f(cx-dx, cy-dy)
                };
                // （变换后）正圆最小外接矩形的四个顶点+正圆圆心
                std::vector<cv::Point2f> dst = {
                    cv::Point2f(cx-dx-a/2, cy-dy+a/2),
                    cv::Point2f(cx-dx-a/2, cy-dy-a/2),
                    cv::Point2f(cx-dx+a/2, cy-dy-a/2),
                    cv::Point2f(cx-dx+a/2, cy-dy+a/2),
                    cv::Point2f(cx-dx, cy-dy)
                };
                cv::Mat M = LS_getAffineTransform(src, dst);    // 获取仿射变换矩阵
                cv::Mat cct_image;
                if(cct_roi.size().height>0 && cct_roi.size().width>0) {
                    // 进行仿射变换
                    cv::warpAffine(cct_roi, cct_image, M, cv::Size(std::round(a), std::round(a)), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
                }
                // (5) CCT图像预处理
                cv::Mat cct_large;
                cv::resize(cct_image, cct_large, cv::Size(0, 0), 200.0/a, 200.0/a, cv::INTER_LANCZOS4);
                cv::Mat cct_gray, cct_binary, cct_eroded;
                cv::cvtColor(cct_large, cct_gray, cv::COLOR_BGR2GRAY);
                cv::threshold(cct_gray, cct_binary, 0, 255, cv::THRESH_OTSU | cv::THRESH_BINARY);
                cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
                cv::erode(cct_binary, cct_eroded, kernel);
                cv::Mat cct_code = imageNormalization(cct_eroded);  // 0-255 to 0-1

                // (6) 解码
                if(isCCT(cct_code)) {   // 判断该区域是否真的包含CCT码
                    int id = cctDecoding(cct_code, N);  // 解码，并获取解码编号结果
                    // result_list.emplace_back(box1.center, id);
                    results.insert(std::make_pair(id, box1.center));
                    // 在图像中画出拟合的椭圆和解码后的数字和中心点
                    cv::ellipse(image_clone, box1, cv::Scalar(0, 255, 0));
                    cv::ellipse(image_clone, box2, cv::Scalar(0, 255, 0));
                    cv::ellipse(image_clone, box3, cv::Scalar(0, 255, 0));
                    cv::circle(image_clone, box1.center, 2, cv::Scalar(255, 0, 0), 3);
                    cv::putText(image_clone, std::to_string(id), box1.center, cv::HersheyFonts::FONT_ITALIC, 1, cv::Scalar(0, 0, 200), 2);
                }
            }
        }
        cv::namedWindow("result", cv::WINDOW_FREERATIO | cv::WINDOW_FULLSCREEN);
        cv::imshow("result", image_clone);
    }

    std::map<int, cv::Point2f> CCTExtractor::getExtractionResult(void) const
    {
        return this->results;
    }

    cv::Mat LS_getAffineTransform(const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst)
    {
        // 利用线性最小二乘法得到放射变换矩阵，参考博客：https://www.cnblogs.com/bingdaocaihong/p/7003581.html
        const int N = src.size();
        cv::Mat A = cv::Mat::zeros(2*N, 6, CV_32F);
        cv::Mat b = cv::Mat::zeros(2*N, 1, CV_32F);
        for(size_t i=0; i<N; ++i) {
            for(size_t j=0; j<2; ++j) {
                if(j == 0) {
                    A.at<float>(2*i+j, 0) = src[i].x;
                    A.at<float>(2*i+j, 1) = src[i].y;
                    A.at<float>(2*i+j, 4) = 1;
                    b.at<float>(2*i+j, 0) = dst[i].x;
                }
                if(j == 1) {
                    A.at<float>(2*i+j, 2) = src[i].x;
                    A.at<float>(2*i+j, 3) = src[i].y;
                    A.at<float>(2*i+j, 5) = 1;
                    b.at<float>(2*i+j, 0) = dst[i].y;
                }
            }
        }
        cv::Mat AT = cv::Mat::zeros(2*N, 6, CV_32F);
        cv::transpose(A, AT);
        cv::Mat ATA = AT * A;
        if(cv::determinant(ATA) < 0.1) {
            return cv::Mat::zeros(2, 3, CV_32F);
        }
        cv::Mat ATA_inv = cv::Mat::zeros(2*N, 6, CV_32F);
        cv::invert(ATA, ATA_inv);
        cv::Mat x = ATA_inv * AT * b;
        cv::Mat M = cv::Mat::zeros(2, 3, CV_32F);
        M.at<float>(0, 0) = x.at<float>(0, 0);
        M.at<float>(0, 1) = x.at<float>(1, 0);
        M.at<float>(0, 2) = x.at<float>(4, 0);
        M.at<float>(1, 0) = x.at<float>(2, 0);
        M.at<float>(1, 1) = x.at<float>(3, 0);
        M.at<float>(1, 2) = x.at<float>(5, 0);
        return M;
    }

    cv::Mat imageNormalization(const cv::Mat& image_0_255)
    {
        cv::Mat image_0_1 = cv::Mat::zeros(image_0_255.size(), CV_8U);
        for (int y = 0; y < image_0_255.rows; ++y) {
            for (int x = 0; x < image_0_255.cols; ++x) {
                uchar pixelValue = image_0_255.at<uchar>(y, x);
                uchar normalizedValue = pixelValue / 255; // 将0和255转换为0和1
                image_0_1.at<uchar>(y, x) = normalizedValue;
            }
        }
        return image_0_1;
    }

    // 根据CCT编码特性判断图像是否是CCT编码（内环全白，中环全黑，外环有黑有白）
    bool isCCT(const cv::Mat& image_bin)
    {
        const int sample_num = 36;
        const int x0 = image_bin.cols / 2;
        const int y0 = image_bin.rows / 2;
        const double r = x0 / 3.0;
        std::vector<int> pixels_0_5_r, pixels_1_5_r, pixels_2_5_r;
        for(int i=0; i<sample_num; ++i) {
            double xi = cos(10.0 * i / 180 * M_PI);
            double yi = sin(10.0 * i / 180 * M_PI);
            int x_0_5_r = x0 + 0.5*r*xi;
            int y_0_5_r = y0 + 0.5*r*yi;
            int x_1_5_r = x0 + 1.5*r*xi;
            int y_1_5_r = y0 + 1.5*r*yi;
            int x_2_5_r = x0 + 2.5*r*xi;
            int y_2_5_r = y0 + 2.5*r*yi;
            uchar pixelValue_0_5_r = image_bin.at<uchar>(round(y_0_5_r), round(x_0_5_r));
            uchar pixelValue_1_5_r = image_bin.at<uchar>(round(y_1_5_r), round(x_1_5_r));
            uchar pixelValue_2_5_r = image_bin.at<uchar>(round(y_2_5_r), round(x_2_5_r));
            pixels_0_5_r.push_back(static_cast<int>(pixelValue_0_5_r));
            pixels_1_5_r.push_back(static_cast<int>(pixelValue_1_5_r));
            pixels_2_5_r.push_back(static_cast<int>(pixelValue_2_5_r));
        }
        if(std::accumulate(pixels_0_5_r.begin(), pixels_0_5_r.end(), 0) == sample_num &&
            std::accumulate(pixels_1_5_r.begin(), pixels_1_5_r.end(), 0) == 0 &&
            std::accumulate(pixels_2_5_r.begin(), pixels_2_5_r.end(), 0) >= 2) {
                return true;
            }
        return false;
    }

    // 对二进制编码序列进行逻辑循环左移一位的函数
    // ==> 移位前：101010110011
    // ==> 移位后：010101100111
    void moveBits(std::deque<int>& seq)
    {
        int front = seq.front();
        seq.pop_front();
        seq.push_back(front);
    }

    // 二进制转十进制
    int B2D(const std::deque<int>& code)
    {
        const int N = code.size();
        int result = 0;
        for(int i=0; i<N; ++i) {
            if(code[i] == 1) {
                result += pow(2, i);
            }
        }
        return result;
    }

    // 二进制编码转对应最小的十进制数
    int B2D_min(const std::deque<int>& code)
    {
        std::deque<int> temp = code;
        const int N = temp.size();
        int minimum = 9999999;
        for(int i=0; i<N; ++i) {
            int result = B2D(temp);
            if(result < minimum)
                minimum = result;
            moveBits(temp);
        }
        return minimum;
    }

    // 十进制转二进制
    std::deque<int> D2B(const int D, const size_t N)
    {
        std::deque<int> result(N, 0);
        int value = D;
        for(int i=0; i<N; ++i) {
            if(value > 0) {
                result[i] = value % 2;
                value = value / 2;
            }
            else
                result[i] = 0;
        }
        return result;
    }

    // CCT解码函数
    int cctDecoding(const cv::Mat& cct_code, const int N)
    {
        const int x0 = cct_code.cols / 2;
        const int y0 = cct_code.rows / 2;
        const double r = x0 / 3.0;
        cv::Mat code_mat = cv::Mat::zeros(360/N, N, CV_8U);
        // 从多个起点开始进行遍历，对所有结果进行平均，提高解码算法的鲁棒性
        for(int i=0; i<360/N; ++i) {
            std::deque<int> code_i;
            // 如果N=12，那就是每隔30度取一个点
            for(int j=0; j<N; ++j) {
                double xi = cos((360.0 * j / N + i) / 180 * M_PI);
                double yi = sin((360.0 * j / N + i) / 180 * M_PI);
                int x = x0 + 2.5*r*xi;
                int y = y0 + 2.5*r*yi;
                uchar pixelValue = cct_code.at<uchar>(round(y), round(x));
                code_i.push_back(static_cast<int>(pixelValue));
            }

            int Dmin = B2D_min(code_i);
            std::deque<int> code_i_min = D2B(Dmin, N);
            for(int k=0; k<N; ++k) {
                code_mat.at<uchar>(i, k) = static_cast<uchar>(code_i_min[k]);
            }
        }
        cv::Mat code_avg;
        cv::reduce(code_mat, code_avg, 0, cv::ReduceTypes::REDUCE_AVG, CV_64F); // 对解码矩阵进行按列求平均
        // 对平均后的行向量进行二值化处理，从而得到最终的二进制解码序列
        std::deque<int> temp;
        for(size_t i=0; i<N; ++i) {
            if(code_avg.at<double>(0, i) <= 0.5)
                temp.push_back(0);
            else if(code_avg.at<double>(0, i) > 0.5)
                temp.push_back(1);
        }
        return B2D(temp);
    }

    cv::Scalar hsv2bgr(const float h, const float s, const float v)
    {
        cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(h/2, s*255, v*255));
        cv::Mat bgr;
        cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
        cv::Vec3b color = bgr.at<cv::Vec3b>(0, 0);
        return cv::Scalar(color[0], color[1], color[2]);
    }

    PointPairs cctMatching(const cv::Mat& img1, const cv::Mat& img2, const Results& res1, const Results& res2)
    {
        PointPairs pps;
        // 查找两张图像中的匹配点
        for(auto r : res1) {
            auto it = res2.find(r.first);
            if(it != res2.end()) {
                pps.emplace_back(std::make_pair(r.second, it->second));
            }
        }
        // 在图像中可视化匹配点
        cv::Mat image_matched = cv::Mat::zeros(cv::Size(img1.cols + img2.cols, std::max(img1.rows, img2.rows)), img1.type());
        cv::Mat left(image_matched, cv::Rect(0, 0, img1.cols, img1.rows));
        img1.copyTo(left);
        cv::Mat right(image_matched, cv::Rect(img1.cols, 0, img2.cols, img2.rows));
        img2.copyTo(right);
        const cv::Point2f offset(img1.cols, 0);
        for(int i=0; i<pps.size(); ++i) {
            const float hue = (i * 360.0f) / pps.size();
            cv::Scalar color = hsv2bgr(hue, 1.0f, 1.0f);
            cv::circle(image_matched, pps[i].first, 2, color, 2);
            cv::circle(image_matched, pps[i].second + offset, 2, color, 2);
            cv::line(image_matched, pps[i].first, pps[i].second + offset, color, 1);
        }
        cv::imshow("image_matched", image_matched);
        return pps;
    }
}
