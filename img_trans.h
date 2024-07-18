#ifndef __IMG_TRANS_
#define __IMG_TRANS_

#include <opencv2/opencv.hpp>
#include "cct.h"

namespace vm
{
    struct Corners
    {
        Corners() = default;
        Corners(const cv::Point2f& tl_, const cv::Point2f& tr_,
                const cv::Point2f& bl_, const cv::Point2f& br_) 
                : tl(tl_), tr(tr_), bl(bl_), br(br_) { }
        cv::Point2f tl;
        cv::Point2f tr;
        cv::Point2f bl;
        cv::Point2f br;
    };

    class ImageSticher
    {
        public:
            explicit ImageSticher(const cv::Mat& img1, const cv::Mat& img2, const PointPairs& pps_)
            : image1(img1), image2(img2), pps(pps_) { }
            cv::Mat getStichingResult(void) const;
            void stich(void);
            Corners getTransImageCorners(const cv::Mat& image, const cv::Mat& H);
            void optimizeSeam(void);

        private:
            cv::Mat image1;
            cv::Mat image2;
            cv::Mat imageTrans;
            cv::Mat imageStiched;
            PointPairs pps;
            Corners corners;
    };

    // 图像拼接器接口仿函数
    class Stich2Images
    {
        public:
            explicit Stich2Images(const int bit_N, const double R_threshold) : N(bit_N), R_th(R_threshold) { }
            cv::Mat operator() (const cv::Mat& img1, const cv::Mat& img2)
            {
                // 图片1的CCT解码
                CCTExtractor ce1(img1, N, R_th);
                cv::Mat image_extraction = ce1.extract();
                cv::imwrite("cct_extraction_" + std::to_string(vm::CCTExtractor::counter) + ".jpg", image_extraction);
                Results res1 = ce1.getExtractionResult();
                // 图片2的CCT解码
                CCTExtractor ce2(img2, N, R_th);
                image_extraction = ce2.extract();
                cv::imwrite("cct_extraction_" + std::to_string(vm::CCTExtractor::counter) + ".jpg", image_extraction);
                Results res2 = ce2.getExtractionResult();
                // 获取匹配点信息
                PointPairs pps = cctMatching(img1, img2, res1, res2);
                // 进行图像拼接
                ImageSticher is(img1, img2, pps);
                is.stich();
                return is.getStichingResult();
            }
        private:
            int N = 12;
            double R_th = 0.85;
    };
    
}

#endif