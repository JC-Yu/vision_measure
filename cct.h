#ifndef __CCT_
#define __CCT_

#include <opencv2/opencv.hpp>
#include <deque>
#include <map>
#include <numeric>

namespace vm
{
    using Results = std::map<int, cv::Point2f>;
    using PointPairs = std::vector<std::pair<cv::Point2f, cv::Point2f>>;
    class CCTExtractor
    {
        public:
            explicit CCTExtractor(const cv::Mat& img, const int bit_N, const double R_threshold) : image(img), N(bit_N), R_th(R_threshold) 
            { ++counter; }
            explicit CCTExtractor(const cv::Mat& img) : image(img) { ++counter; }
            Results getExtractionResult(void) const;
            cv::Mat extract(void);
            static int counter;

        private:
            cv::Mat image;
            int N = 12;
            double R_th = 0.85;
            Results results;
    };
    inline cv::Mat LS_getAffineTransform(const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst);
    inline cv::Mat imageNormalization(const cv::Mat& image_0_255);
    inline bool isCCT(const cv::Mat& image_bin);
    inline int B2D(const std::deque<int>& code);
    inline int B2D_min(const std::deque<int>& code);
    inline std::deque<int> D2B(const int D, const size_t N);
    inline void moveBits(std::deque<int>& seq);
    inline int cctDecoding(const cv::Mat& cct_code, const int N);
    inline cv::Scalar hsv2bgr(const float h, const float s, const float v);
    PointPairs cctMatching(const cv::Mat& img1, const cv::Mat& img2, const Results& res1, const Results& res2);
}

#endif