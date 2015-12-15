#ifndef TOOLS_HPP
#define TOOLS_HPP
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <vector>

// the binarization of the score has to be automated, for now, I just assign a threshol equal to 220 because the Otsu method failed and I was not able to find a strong method fast enough to keep the further process working each time : to be continued
cv::Mat				binarize(cv::Mat const& img, unsigned char threshold);

double				radToDeg(double angle);

std::vector<int>	getHorizontalProfil(cv::Mat const& score);

int					findInterline(std::vector<int> profilVect);

cv::Mat				getVerticalProfil(cv::Mat const& score);

int					getMax(std::vector<int>	const&	data);

int					getMaxIndex(std::vector<int> const& data);

// some tests have been maid but this is not used yet : I keep it because I think there is something to do with this to improve the quality and the thickness of the lines of the staves in further pre treaments and avoid some disturbance of the double bars eighth ('barres de doubles croches') that sometimes (where the lines are very thin because of a resize of the score) leads to a wrong detection of the middle line in a stave used for the detection and removal of the lines
//{
cv::Mat				filter(cv::Mat& img, cv::Mat kernel);

cv::Mat				getSobelKernel(int radius);

cv::Mat				getLaplacianKernel(int radius);
//}

#endif
