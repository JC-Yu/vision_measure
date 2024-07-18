#include "img_trans.h"

namespace vm
{
    void ImageSticher::stich(void)
    {   
        // 1. 求解单应矩阵
        std::vector<cv::Point2f> keyPoints1, keyPoints2;
        for(auto pp : pps) {
            keyPoints1.emplace_back(pp.first);
            keyPoints2.emplace_back(pp.second);
        }
        cv::Mat H = cv::findHomography(keyPoints2, keyPoints1, cv::RANSAC); // 图像2到图像1的单应矩阵
        // std::cout << "==> H = \n" << H << std::endl;

        // 2. 获取变换后图片中的四个角点
        Corners corners = getTransImageCorners(image2, H);
        // std::cout << "top left: " << corners.tl << std::endl;
        // std::cout << "buttom left: " << corners.bl << std::endl;
        // std::cout << "top right: " << corners.tr << std::endl;
        // std::cout << "buttom right: " << corners.br << std::endl;
        
        // 3. 图像配准
        cv::warpPerspective(image2, imageTrans, H, cv::Size(std::max(corners.tr.x, corners.br.x), image2.rows));
        // cv::imwrite("imageTrans.jpg", imageTrans);

        // 4. 图像拼接
        imageStiched = cv::Mat::zeros(cv::Size(imageTrans.cols, image1.rows), CV_8UC3);
        imageTrans.copyTo(imageStiched(cv::Rect(0, 0, imageTrans.cols, imageTrans.rows)));
        image1.copyTo(imageStiched(cv::Rect(0, 0, image1.cols, image1.rows))); 

        // 5. 优化图像的连接边界
        optimizeSeam();
        // cv::imwrite("imageStiched.jpg", imageStiched);
    }

    // 获取拼接结果
    cv::Mat ImageSticher::getStichingResult(void) const
    {
        return imageStiched;
    }

    // 求解经单应矩阵映射后图像的四个角点
    Corners ImageSticher::getTransImageCorners(const cv::Mat& image, const cv::Mat& H)
    {
        Corners corners;
        // 左上角
        double tl_src[3] = {0, 0, 1};
        double tl_dst[3] = {0, 0, 0};
        cv::Mat TL_src = cv::Mat(3, 1, CV_64FC1, tl_src);
        cv::Mat TL_dst = cv::Mat(3, 1, CV_64FC1, tl_dst);
        TL_dst = H * TL_src;
        corners.tl.x = tl_dst[0] / tl_dst[2];
        corners.tl.y = tl_dst[1] / tl_dst[2];

        // 右上角
        double tr_src[3] = {static_cast<double>(image.cols), 0, 1};
        double tr_dst[3] = {0, 0, 0};
        cv::Mat TR_src = cv::Mat(3, 1, CV_64FC1, tr_src);
        cv::Mat TR_dst = cv::Mat(3, 1, CV_64FC1, tr_dst);
        TR_dst = H * TR_src;
        corners.tr.x = tr_dst[0] / tr_dst[2];
        corners.tr.y = tr_dst[1] / tr_dst[2];

        // 左下角
        double bl_src[3] = {0, static_cast<double>(image.rows), 1};
        double bl_dst[3] = {0, 0, 0};
        cv::Mat BL_src = cv::Mat(3, 1, CV_64FC1, bl_src);
        cv::Mat BL_dst = cv::Mat(3, 1, CV_64FC1, bl_dst);
        BL_dst = H * BL_src;
        corners.bl.x = bl_dst[0] / bl_dst[2];
        corners.bl.y = bl_dst[1] / bl_dst[2];

        // 右下角
        double br_src[3] = {static_cast<double>(image.cols), static_cast<double>(image.rows), 1};
        double br_dst[3] = {0, 0, 0};
        cv::Mat BR_src = cv::Mat(3, 1, CV_64FC1, br_src);
        cv::Mat BR_dst = cv::Mat(3, 1, CV_64FC1, br_dst);
        BR_dst = H * BR_src;
        corners.br.x = br_dst[0] / br_dst[2];
        corners.br.y = br_dst[1] / br_dst[2];

        return corners;
    }

    // 重叠部分图像优化,参考:https://www.cnblogs.com/skyfsm/p/7411961.html
    void ImageSticher::optimizeSeam(void)
    {
        // 计算重叠区域的起点和宽度
        const int start = std::min(corners.tl.x, corners.bl.x);
        const double width = image1.cols - start;

        // 计算优化权重
        double alpha = 1;
        for(int i=0; i<imageStiched.rows; ++i) {
            // 获取图像第 i 行的行指针
            uchar* p = image1.ptr<uchar>(i);
            uchar* t = imageTrans.ptr<uchar>(i);
            uchar* d = imageStiched.ptr<uchar>(i);
            for(int j=start; j<image1.cols; ++j) {
                // 根据像素灰度值以及距离起点的远近来计算权值
                if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0)
                    alpha = 1;
                else
                    alpha = (width - (j - start)) / width;
                // 优化像素值灰度
                d[j * 3] = p[j * 3] * alpha + t[j * 3] * (1 - alpha);
                d[j * 3 + 1] = p[j * 3 + 1] * alpha + t[j * 3 + 1] * (1 - alpha);
                d[j * 3 + 2] = p[j * 3 + 2] * alpha + t[j * 3 + 2] * (1 - alpha);
            }
        }
    }
}