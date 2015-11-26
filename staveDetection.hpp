#ifndef STAVE_DETECTION_HPP
#define STAVE_DETECTION_HPP
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include "Bivector.hpp"

int						correlation(cv::Mat const& binaryImg);

void					print(std::vector<double> const& values, int height, int heightScale, int width, int widthScale);

cv::Mat					correctSlope(cv::Mat const& binaryImg);

double					getTheta(int hMax, int width);

std::vector<int>		getStavesProfilVect(std::vector<int> const& profilVect, int interline);

std::vector<int>		detectCenterLinePos(std::vector<int> const& profilVect, int interline);

std::vector<int>		getLocMaxima(std::vector<int> const& data, int range);

std::vector<int>		getLineThicknessHistogram(std::vector<int> const& middleLinePositions, int heightSize, cv::Mat const& binaryImg);

int						getLineThickness(std::vector<int> const& histogram, unsigned int maxHisto);

std::vector<cv::Mat>	extractSubImages(cv::Mat const& binaryImg, std::vector<int> const& centerLinePositions, int interline);

Bivector				getOrdsPosition(std::vector<cv::Mat> const& subImg, int thicknessMoy, int thickness0, int interline, std::vector<int> const& subImgCenter);

std::vector<unsigned char>	getMask(int interval, int thickness0);

#endif
