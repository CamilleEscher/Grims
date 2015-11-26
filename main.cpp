#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <cmath>
#include "tools.hpp"
#include "Staves.hpp"
#include "staveDetection.hpp"

int main()
{
	std::string	scoreFolder = "scores/";
	std::string	scoreName = "score_penche.png";//prelude-1.png";
	std::string	scorePath = scoreFolder + scoreName;
	cv::Mat		score = cv::imread(scorePath, CV_LOAD_IMAGE_GRAYSCALE);

	if(score.empty())
	{
		std::cout << scoreName << " can't be found" << std::endl;
		return -1;
	}
	else
	{
		//cv::resize(score, score, cv::Size(score.cols / 2, score.rows / 2));
		//cv::Mat					binaryScore(binarize(score, 220));
		Staves	staves;
		staves.setup(score);
		//cv::imshow("binaryScore", binaryScore);
		cv::waitKey(0);
	}
	return 0;
}
