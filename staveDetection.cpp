#include "staveDetection.hpp"
#include <cmath>
#include "tools.hpp"
#include <iostream>

int		correlation(cv::Mat const& binaryImg)
{
	std::vector<double>	vectCor;
	double				maxCor = 0.0;
	int					hMax = 0;
	int					hRangeMax = 60;

	vectCor.assign(hRangeMax, 0);
	// correlation processing
	for(int h = 0; h < hRangeMax; ++h)
	{
		for(int i = 0; i < binaryImg.rows; ++i)
		{
			for(int j = 0; j < binaryImg.cols / 2; ++j)
			{
				if((i - h + (hRangeMax / 2) < 0) || (i - h + (hRangeMax / 2) >= binaryImg.rows))
				{
					continue;
				}
				// Rescale the binary values of the image in the range [-1; 1] => if img(x,y) = 0, img'(x,y) = -1
				int leftPixelValue = 1;
				int rightPixelValue = 1;
				// on the 'left image' [0;  width / 2]
				if(binaryImg.at<unsigned char>(i, j) == 0)
				{
					leftPixelValue = -1;
				}
				// on the 'right image' [width / 2; width] which has been shifted vertically by the factor of height which contains values between [-hRangeMax / 2; hRangeMax / 2]
				if(binaryImg.at<unsigned char>(i - h + (hRangeMax / 2), j + (binaryImg.cols / 2)) == 0)
				{
					rightPixelValue = -1;
				}
				// add 1 to the correlation vector if pixColor(leftImg) = pixColor(rightShiftedImg) and -1 if not
				vectCor[h] += (leftPixelValue * rightPixelValue);
			}
		}
		// normalize the values of the correlation vector
		vectCor[h] *= 2.0;
	    vectCor[h] /= static_cast<double>(binaryImg.cols * binaryImg.rows);
		// get the index of the maximum value of the correlation vector (= hMax)
		if(vectCor[h] > maxCor)
		{
			maxCor = vectCor[h];
			hMax = h - hRangeMax / 2;
		}
	}
// print results
	//std::cout << "Results : hMax = " << hMax << "    maxCor = " << maxCor << "    theta = " << getTheta(hMax, binaryImg.cols) << std::endl;
// print the image of the correlation
	//print(vectCor, round(maxCor), 500, hRangeMax, 5);
	return hMax;
}

void	print(std::vector<double> const& values, int height, int heightScale, int width, int widthScale)
{
	cv::Mat	cor(height * heightScale + 10, width * widthScale, CV_8UC1, cv::Scalar(255));
	for(int w = 0; w < width; ++w)
	{
		// repeat 5 time the width of the values
		for(int wOffset = w * widthScale; wOffset < (w + 1) * widthScale; ++wOffset)
		{
		// color in black the image from the bottom to values[h] (heightScale fold)
			for(int rows = cor.rows - 1; rows >= cor.rows - round(values[w] * heightScale) - 1; --rows)
			{
				cor.at<unsigned char>(rows, wOffset) = 0;
			}
		}
	}
	cv::imshow("correlation", cor);
}

double	getTheta(int hMax, int width)
{
	return radToDeg(atan((2.0 * hMax) / static_cast<double>(width)));
}

cv::Mat	correctSlope(cv::Mat const& binaryImg)
{
	int hMax = correlation(binaryImg);
	cv::Mat	correctedImg;
	correctedImg = cv::Mat::zeros(binaryImg.rows, binaryImg.cols, CV_8UC1);
	for(int i = 0; i < binaryImg.rows; ++i)
	{
		for(int j = 0; j < binaryImg.cols; ++j)
		{
			if(i - (2 * hMax * j / binaryImg.cols) >= 0)
			correctedImg.at<unsigned char>(i, j) = binaryImg.at<unsigned char>(i - (2 * hMax * j / binaryImg.cols), j);
		}
	}
	return correctedImg;
}

std::vector<int>	getStavesProfilVect(std::vector<int> const& profilVect, int interline)
{
	int					profilVectSize = static_cast<int>(profilVect.size());
	std::vector<int>	stavesProfilVect;

	stavesProfilVect.assign(profilVectSize, 0);
	for(int x = 0; x < profilVectSize; ++x)
	{
		for(int i = -2; i <= 2; ++i)
		{
			for(int j = -1; j <= 1; ++j)
			{
				if((x + i * interline + j) < profilVectSize && (x + i * interline + j) >= 0)
				{
					stavesProfilVect.at(x) += profilVect.at(x + i * interline + j);
				}
			}
		}
		//std::cout << staveDetector.at(x) << std::endl;
	}
	return stavesProfilVect;
}

std::vector<int>	detectCenterLinePos(std::vector<int> const& profilVect, int interline)
{
	std::vector<int>	centerLinePositions;
	std::vector<int>	stavesProfilVect = getStavesProfilVect(profilVect, interline);

	centerLinePositions = getLocMaxima(stavesProfilVect, interline * 2);
	return centerLinePositions;
}

std::vector<int>	getLocMaxima(std::vector<int> const& data, int range)
{
	std::vector<int>	locMaxima;
	int					sizeData = static_cast<int>(data.size());
	int					initMax = getMax(data) / 3;
	int					max = initMax;
	int					indexMax = 0;

	locMaxima.reserve(10);
	for(int i = range; i < sizeData - range; ++i)
	{
		if(data.at(i) > max)
		{
			max = data.at(i);
			indexMax = i;
			for(int r = i - range; r < i + range; ++r)
			{
				if(data.at(r) > max)
				{
					indexMax = -1;
					break;
				}
			}
			if(indexMax == i)
			{
				locMaxima.push_back(i);
				i = i + range;
				max = initMax;
			}
		}
	}
	return locMaxima;
}

std::vector<int>	getLineThicknessHistogram(std::vector<int> const& middleLinePositions, int heightSize, cv::Mat const& binaryImg)
{
	std::vector<int>	histogram;
	int					halfHeightSize = round(heightSize / 2.0);

	histogram.assign(heightSize, 0);
	for(int j = 0; j < binaryImg.cols; ++j)
	{
		for(auto line = middleLinePositions.begin(); line != middleLinePositions.end(); ++line)
		{
			for(int i = *line - halfHeightSize; i < *line + halfHeightSize; ++i)
			{
				if(i >= 0 && i < binaryImg.rows)
				{
					auto colorPix = binaryImg.at<unsigned char>(i, j);
					if(colorPix == 0)
					{
						int thickness = 1;
						while(++i < *line + halfHeightSize && binaryImg.at<unsigned char>(i, j) == 0)
						{
							thickness++;
						}
						if(thickness < heightSize)
						{
							++histogram.at(thickness);
						}
					}
				}
			}
		}
	}
	return histogram;
}

int	getLineThickness(std::vector<int> const& histogram, unsigned int maxHisto)
{
	double				thicknessN = 0;
	double				thicknessD = 0;

	for(unsigned int e = maxHisto - 1; e <= maxHisto + 1; ++e)
	{
		thicknessN += e * histogram[e];
		thicknessD += histogram[e];
	}
	if(thicknessD != 0)
	{
		return (thicknessN / thicknessN);
	}
	return -1;	
}

std::vector<cv::Mat>	extractSubImages(cv::Mat const& binaryImg, std::vector<int> const& centerLinePositions, int interline)
{
	int	centerLinePositionsSize = static_cast<int>(centerLinePositions.size());
	std::vector<int>		profilVect;
	std::vector<int>		newCenterLinePositions;
	std::vector<int>		subImgCenter;
	std::vector<int>		subImgOrigin;
	std::vector<int>		subImgHeight;
	std::vector<cv::Mat>	subImages;
	int						hMax = 0;

	subImgCenter.reserve(centerLinePositionsSize);
	subImgOrigin.reserve(centerLinePositionsSize);
	subImgHeight.reserve(centerLinePositionsSize);
	subImages.reserve(centerLinePositionsSize);
	newCenterLinePositions.reserve(centerLinePositionsSize);
	subImgCenter.push_back(0);
	subImgOrigin.push_back(0);
	newCenterLinePositions.push_back(0);
	for(int i = 1; i < centerLinePositionsSize; ++i)
	{
		subImgCenter.push_back(round((centerLinePositions.at(i - 1) + centerLinePositions.at(i)) / 2.0));
		subImgOrigin.push_back(subImgCenter.at(i) - 2 * interline);
		newCenterLinePositions.push_back(subImgCenter.at(i) - subImgOrigin.at(i));
	}
	for(int i = 0; i < centerLinePositionsSize - 1; ++i)
	{
		subImgHeight.push_back(subImgCenter.at(i + 1) + 2 * interline - subImgOrigin.at(i));
	}
	subImgHeight.push_back(binaryImg.rows - subImgOrigin.at(centerLinePositionsSize - 1));
	for(int i = 0; i < centerLinePositionsSize; ++i)
	{
		profilVect.clear();
		profilVect.reserve(subImgHeight.at(i));
		subImages.push_back(cv::Mat::zeros(subImgHeight.at(i), binaryImg.cols, CV_8UC1));
		for(int x = 0; x < subImgHeight.at(i); ++x)
		{
			for(int y = 0; y < binaryImg.cols; ++y)
			{
				(subImages.at(i)).at<unsigned char>(x, y) = binaryImg.at<unsigned char>(x + subImgOrigin.at(i), y);
			}
		}
		hMax = correlation(subImages.at(i));
		subImages.at(i) = correctSlope(subImages.at(i));
		//cv::imshow(std::to_string(i), subImages.at(i));
	}
	return subImages;
}

Bivector  getOrdsPosition(std::vector<cv::Mat> const& subImg, int thicknessMoy, int thickness0, int interline, std::vector<int> const& subImgCenter)
{
	int 				deltaXRange = floor(static_cast<double>(thickness0) / 2.0);
	int					deltaXPRange = round(interline / 2.0);
	Bivector			ords;
	std::vector<int>	leftOrds;
	std::vector<int>	rightOrds;
	int					profilDeltaXP;
	std::vector<int>	profil;
	int					S = round(2.5 * thicknessMoy);

	leftOrds.assign(static_cast<int>(subImg.size()), -1);
	rightOrds.assign(static_cast<int>(subImg.size()), -1);
	for(int i = 0; i < static_cast<int>(subImg.size()); ++i)
	{
		profil.assign(subImg.at(i).cols, 0);
		for(int y = 0; y < subImg.at(i).cols; ++y)
		{
			profilDeltaXP = 0;
			for(int deltaXP = -deltaXPRange; deltaXP <= deltaXPRange; ++deltaXP)
			{
				for(int k = -2; k <= 2; ++k)
				{
					for(int deltaX = -deltaXRange; deltaX <= deltaXRange; ++deltaX)
					{
						profilDeltaXP += (subImg.at(i).at<unsigned char>(subImgCenter.at(i) + k * interline + deltaX + deltaXP, y) + 1) % 2;
					}
				}
			}
			if(profil[y] < profilDeltaXP)
			{
				profil[y] = profilDeltaXP;
			}
			if(leftOrds.at(i) == -1 && y > interline && profil[y] > S && profil[y - interline] > S)
			{
				for(int yBefore = y - interline; yBefore < y; ++yBefore)
				{
					if(profil[yBefore] < S)
					{
						break;
					}
					else if(yBefore == y - 1)
					{
						leftOrds.at(i) = y;
					}
				}
			}
			if(y < (subImg.at(i).cols - interline) && profil[y] > S && profil[y + 1] < S)
			{
				for(int yAfter = y + 1; yAfter < (y + interline); ++yAfter)
				{
					if(profil[yAfter] > S)
					{
						break;
					}
					else if(yAfter == y + interline - 1)
					{
						rightOrds.at(i) = y;
					}
				}
			}
		}
	}
	ords.setLeft(leftOrds);
	ords.setRight(rightOrds);
	return ords;
}

std::vector<unsigned char>	getMask(int interval, int thickness0, int staveHeight)
{	
	std::vector<unsigned char> mask;
	mask.assign(staveHeight, 0);
	int deltaB = floor(thickness0 / 2.0);
	int deltaH = thickness0 - 1 - deltaB;
	for(int x = 0; x < staveHeight; ++x)
	{
		for(int k = -2; k < 3; ++k)
		{
			for(int i = -deltaB; i <= deltaH; ++i)
			{
				if(x == ((Hp / 2.0) + k * interval + i))
				{
					mask.at(x) = 1;
				}
			}
		}
	}
	return mask;
}

std::vector<int>	getCenterLineAbsc(int centerLinePositions, int interline, int thickness0, cv::Mat subImgI, int leftOrd, int rightOrd)
{	
	int staveHeight = 2 * floor(2.5 * interval);
	int xShiftedRange = floot(interline / 2.0);
	std::vector<unsigned char>	mask = getMask(interval, thickness0, staveHeight);
	cv::Mat maskImgCorrelation;

	maskImgCorrelation = cv::Mat::zeros((xShiftedRange * 2 + 1), rightOrd - leftOrd, CV_8UC1);
	for(xShifted = -xShiftedRange; x <= xShiftedRange; ++xShifted)
	{
		for(y = leftOrd; y <= rightOrd; ++y)
		{
			for(h = 0; h < staveHeight; ++h)
			{
				maskImgCorrelation.at<unsigned char>(xShifted, y) += (1.0 / staveHeight) * mask(h) * subImgI.at<unsigned char>(h + centerLinePositions - (staveHeight / 2.0) + xShifted, y);
				//TODO
			}
		}
	}
}
