#include "tools.hpp"
#include <iostream>

static double	PI = 3.14159265359;

cv::Mat	binarize(cv::Mat const& img, unsigned char thresh)
{
	cv::Mat	binarizedImg(img.rows, img.cols, CV_8UC1);
	cv::threshold(img, binarizedImg, thresh, 255, cv::THRESH_BINARY);

	return binarizedImg;
}

double	radToDeg(double angle)
{
	return (angle * 180.0 / PI);
}

std::vector<int>	getHorizontalProfil(cv::Mat const& img)
{
	std::vector<int> profilVect;

	profilVect.assign(img.rows, 0);
	for(int i = 0; i < img.rows; ++i)
	{
		for(int j = 0; j < img.cols; ++j)
		{	
			auto pixColor = img.at<unsigned char>(i, j);
			profilVect.at(i) += ((pixColor / 255) + 1) % 2;
		}
	}
	//print profil
	cv::Mat profil(img.rows, img.cols, CV_8UC1, cv::Scalar(255));
	for(int i = 0; i < img.rows; ++i)
	{
		for(int j = 0; j < img.cols; ++j)
		{
			//img.cols - profilVect[i]
			if(profilVect.at(i) >= j)
			{
				profil.at<unsigned char>(i, j) = 0;
			}
		}
	}
	//cv::imshow("profil", profil);
	//cv::waitKey(0);
	return profilVect;
}

int		findInterline(std::vector<int> profilVect)
{

	std::vector<int>	autoCorrelationProfilVect;
	int					autoProfilMax = 0;
	int					interline = 0;

	autoCorrelationProfilVect.assign(100, 0);
	// autocorrelation of the profil
	for(int s = 0; s < 50; ++s)
	{
		for(size_t i = 0; i < profilVect.size(); ++i)
		{
			if((i + s) < profilVect.size())
			{
				autoCorrelationProfilVect.at(s) += profilVect.at(i) * profilVect.at(i + s);
			}
		}
		// max of the autocorrelation of the horizontal profil
		if(s > 3 && autoCorrelationProfilVect.at(s) >= autoProfilMax)
		{
			autoProfilMax = autoCorrelationProfilVect.at(s);
			interline = s;
		}
	}
	// print interline value
	//std::cout << "interline = " << interline << std::endl;
	return interline;
}

cv::Mat	getVerticalProfil(cv::Mat const& img)
{
	cv::Mat 			profil;
	std::vector<double> profilVect;

	profilVect.assign(img.rows, 0);
	profil = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
	for(int i = 0; i < img.cols; ++i)
	{
		for(int j = 0; j < img.rows; ++j)
		{	
			auto pixColor = img.at<cv::Vec3b>(cv::Point(i, j))[0];
			profilVect.at(i) += static_cast<double>(pixColor / 255.0);
		}
	}
	for(int i = 0; i < img.cols; ++i)
	{
		for(int j = 0; j < img.rows; ++j)
		{
			if(profilVect.at(i) > static_cast<double>(img.rows - 1 - j))
			{
				profil.at<unsigned char>(j, i) = 255;
			}
		}
	}
	return profil;
}

cv::Mat	filter(cv::Mat& img, cv::Mat kernel)
{
	if(img.empty() || kernel.empty())
	{
		std::cout << "param empty" << std::endl;
		return img;
	}
	cv::Mat	filteredImg;
	//cv::filter2D(img, filteredImg, img.type(), kernel);
	cv::dilate(img, filteredImg, kernel);
	for(int i = 0; i < 5; ++i)
	{
		cv::dilate(filteredImg, filteredImg, kernel);
	}
	cv::imshow("filteredImg", filteredImg);
	cv::waitKey(0);
	return filteredImg;
}

cv::Mat	getSobelKernel(int radius)
{
	int		kernelSize = radius * 2 + 1;
	cv::Mat	sobelKernel;
	
	sobelKernel = cv::Mat::zeros(kernelSize, kernelSize, CV_8UC1);

	for(int i = 0; i < kernelSize; ++i)
	{
		sobelKernel.at<unsigned char>(((kernelSize - 1) / 2), i) = 1;
	}
	return sobelKernel;
}

cv::Mat	getLaplacianKernel(int radius)
{
	int		kernelSize = radius * 2 + 1;
	cv::Mat	laplacianKernel(cv::Mat_<float>(kernelSize, kernelSize));

	for(int i = 0; i < kernelSize; ++i)
	{
		for(int j = 0; j < kernelSize; ++j)
		{
			if((i == radius) && (j == radius))
			{
				laplacianKernel.at<float>(i, j) = static_cast<float>((kernelSize) * (kernelSize) - 1.f);
			}
			else
			{
				laplacianKernel.at<float>(i, j) = -1.f;
			}
		}
	}
	return laplacianKernel;
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
