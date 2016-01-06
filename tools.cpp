#include "tools.hpp"
#include <iostream>

cv::Mat	binarize(cv::Mat const& img, unsigned char thresh)
{
	cv::Mat	binarizedImg(img.rows, img.cols, CV_8UC1);
	cv::threshold(img, binarizedImg, thresh, 255, cv::THRESH_BINARY);

	return binarizedImg;
}

std::vector<int>	getHorizontalProfile(cv::Mat const& img)
{
	std::vector<int> profileVect;

	profileVect.assign(img.rows, 0);
	for(int i = 0; i < img.rows; ++i)
	{
		for(int j = 0; j < img.cols; ++j)
		{	
			auto pixColor = img.at<unsigned char>(i, j);
			profileVect.at(i) += ((pixColor / 255) + 1) % 2;
		}
	}
	//print profile
	cv::Mat profile(img.rows, img.cols, CV_8UC1, cv::Scalar(255));
	for(int i = 0; i < img.rows; ++i)
	{
		for(int j = 0; j < img.cols; ++j)
		{
			//img.cols - profileVect[i]
			if(profileVect.at(i) >= j)
			{
				profile.at<unsigned char>(i, j) = 0;
			}
		}
	}
	//cv::imshow("profile", profile);
	//cv::waitKey(0);
	return profileVect;
}

int		findInterline(std::vector<int> profileVect)
{

	std::vector<int>	autoCorrelationProfileVect;
	int					autoProfileMax = 0;
	int					interline = 0;

	autoCorrelationProfileVect.assign(100, 0);
	// autocorrelation of the profile
	for(int s = 0; s < 50; ++s)
	{
		for(size_t i = 0; i < profileVect.size(); ++i)
		{
			if((i + s) < profileVect.size())
			{
				autoCorrelationProfileVect.at(s) += profileVect.at(i) * profileVect.at(i + s);
			}
		}
		// max of the autocorrelation of the horizontal profile
		if(s > 3 && autoCorrelationProfileVect.at(s) >= autoProfileMax)
		{
			autoProfileMax = autoCorrelationProfileVect.at(s);
			interline = s;
		}
	}
	// print interline value
	//std::cout << "interline = " << interline << std::endl;
	return interline;
}

cv::Mat	getVerticalProfile(cv::Mat const& img)
{
	cv::Mat 			profile;
	std::vector<double> profileVect;

	profileVect.assign(img.rows, 0);
	profile = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
	for(int i = 0; i < img.cols; ++i)
	{
		for(int j = 0; j < img.rows; ++j)
		{	
			auto pixColor = img.at<cv::Vec3b>(cv::Point(i, j))[0];
			profileVect.at(i) += static_cast<double>(pixColor / 255.0);
		}
	}
	for(int i = 0; i < img.cols; ++i)
	{
		for(int j = 0; j < img.rows; ++j)
		{
			if(profileVect.at(i) > static_cast<double>(img.rows - 1 - j))
			{
				profile.at<unsigned char>(j, i) = 255;
			}
		}
	}
	return profile;
}

cv::Mat	filter(cv::Mat& img, cv::Mat kernel)
{
	if(img.empty() || kernel.empty())
	{
		std::cout << "param empty" << std::endl;
		return img;
	}
	cv::Mat	filteredImg;
	cv::dilate(img, filteredImg, kernel);
	for(int i = 0; i < 5; ++i)
	{
		cv::dilate(filteredImg, filteredImg, kernel);
	}
	cv::imshow("filteredImg", filteredImg);
	cv::waitKey(0);
	return filteredImg;
}

int		getMax(std::vector<int>	const&	data)
{
	int	dataSize = static_cast<int>(data.size());
	int	max = 0;

	for(int i = 0; i < dataSize; ++i)
	{
		if(data.at(i) > max)
		{
			max = data.at(i);
		}
	}
	return max;
}

int		getMaxIndex(std::vector<int> const& data)
{
	int	dataSize = static_cast<int>(data.size());
	int	max = 0;
	int	maxIndex = 0;

	for(int i = 0; i < dataSize; ++i)
	{
		if(data.at(i) > max)
		{
			max = data.at(i);
			maxIndex = i;
		}
	}
	return maxIndex;	
}
