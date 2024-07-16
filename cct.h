#ifndef __CCT_
#define __CCT_

#include <opencv2/opencv.hpp>
#include <deque>
#include <numeric>

namespace cct
{
    struct Result 
    {
        Result() = default;
        Result(const cv::Point2f& center_, const int id_) : center(center_), id(id_) { }
        cv::Point2f center;
        int id = 0;
    };

    class CCTExtractor
    {
        public:
            explicit CCTExtractor(const cv::Mat& img, const int bit_N, const double R_threshold) : image(img), N(bit_N), R_th(R_threshold) { }
            explicit CCTExtractor(const cv::Mat& img) : image(img) { }
            std::vector<Result> getExtractionResult(void) const;
            void extract(void);

        private:
            cv::Mat image;
            int N = 12;
            double R_th = 0.85;
            std::vector<Result> result_list;
    };

    inline cv::Mat LS_getAffineTransform(const std::vector<cv::Point2f>& src, const std::vector<cv::Point2f>& dst);
    inline cv::Mat imageNormalization(const cv::Mat& image_0_255);
    inline bool isCCT(const cv::Mat& image_bin);
    inline int B2D(const std::deque<int>& code);
    inline int B2D_min(const std::deque<int>& code);
    inline std::deque<int> D2B(const int D, const size_t N);
    inline void moveBits(std::deque<int>& seq);
    inline int cctDecoding(const cv::Mat& cct_code, const int N);
}

#endif