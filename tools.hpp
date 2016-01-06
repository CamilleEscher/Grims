#ifndef TOOLS_HPP
#define TOOLS_HPP
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <vector>

// the binarization of the score has to be automated, for now, I just assign a threshold equal to 220 because the Otsu method failed and I was not able to find a strong method fast enough to keep the further process working each time : to be continued
/*!
	\brief binarization of the page of score

	\param img page of score
	\param threshold the threshold I chose equals to 220
*/
cv::Mat				binarize(cv::Mat const& img, unsigned char threshold);

/*!
	\brief calculates the horizontal profile of the score of a part of the score
*/
std::vector<int>	getHorizontalProfile(cv::Mat const& score);

/*!
	\brief get the interline of the score (the method is not adjusted if there is many different widths of staves)

	\param profileVect the horizontalprofile of the staves
*/
int					findInterline(std::vector<int> profileVect);

/*!
	\brief get the vertical profile of the stave

	\param score image of a stave
*/
cv::Mat				getVerticalProfile(cv::Mat const& score);

/*!
	\brief get the maximum value of a vector

	\param data a vector of integers
*/
int					getMax(std::vector<int>	const&	data);

/*!
	\brief get the index of the maximum value of a vector

	\param data a vector of integers
*/
int					getMaxIndex(std::vector<int> const& data);

// some tests have been done but this is not used yet : kept for further improvement of the quality and the thickness of the lines of the staves by improving  pre treaments and avoiding some disturbance of the double bars eighth ('barres de doubles croches') that sometimes (where the lines are very thin because of a resize of the score) leads to a wrong detection of the middle line in a stave used for the detection and removal of the lines
//{
cv::Mat				filter(cv::Mat& img, cv::Mat kernel);
//}

#endif
