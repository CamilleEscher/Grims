#ifndef TOOLS_HPP
#define TOOLS_HPP
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <vector>

cv::Mat				binarize(cv::Mat const& img, unsigned char threshold);

void				rotate(cv::Mat const& src, double angle, cv::Mat& dst);

double				radToDeg(double angle);

std::vector<int>	getHorizontalProfil(cv::Mat const& score);

int					getInterline(std::vector<int> profilVect);

cv::Mat				getVerticalProfil(cv::Mat const& score);

cv::Mat				sharp(cv::Mat& img);

cv::Mat				getLaplacianKernel(int radius);

#endif
