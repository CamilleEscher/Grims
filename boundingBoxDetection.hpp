#ifndef BOUNDING_BOX_DETECTION_HPP
#define BOUNDING_BOX_DETECTION_HPP
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include "Staves.hpp"

cv::Mat					getVerticalSegmentsMap(cv::Mat const& subImg);

void					gatherImages(std::vector<Stave> const& staves);

cv::Mat					getHorizontalSegInVerticalSeg(cv::Mat const& subImg);

cv::Mat 				applyClosingOperation(cv::Mat const& horLinesImg, cv::Mat const& subImg);

std::vector<cv::Mat>	highLightVerticals(std::vector<Stave> const& staves);

cv::Mat					getHorizontalSegmentsMap(cv::Mat const& subImg);

bool					areStavesPaired(cv::Mat const& subImgUp, cv::Mat const& subImgDown, int absCoordMiddleLineUp, int absCoordMiddleLineDown);

cv::Mat					gatherImages(cv::Mat const&	imgUp, cv::Mat const& imgDown, int absDownUpperStaveLine);

bool					compareRectangles(cv::Mat const& rectUp, cv::Mat const& rectDown);

#endif
