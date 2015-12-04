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
				vectCor.at(h) += (leftPixelValue * rightPixelValue);
			}
		}
		// normalize the values of the correlation vector
		vectCor.at(h) *= 2.0;
	    vectCor.at(h) /= static_cast<double>(binaryImg.cols * binaryImg.rows);
		// get the index of the maximum value of the correlation vector (= hMax)
		if(vectCor.at(h) > maxCor)
		{
			maxCor = vectCor.at(h);
			hMax = h - hRangeMax / 2;
		}
	}
// print results
	//std::cout << "Results : hMax = " << hMax << "    maxCor = " << maxCor << "    theta = " << getTheta(hMax, binaryImg.cols) << std::endl;
// print the image of the correlation
	//print(vectCor, round(maxCor), 500, hRangeMax, 5);
	return hMax;
}

void	print(std::vector<double> const& values, int height, int heightScale, int width, int widthScale, int first, int last)
{
	cv::Vec3b	black = {0, 0, 0};
	cv::Vec3b	red = {0, 0, 255};
	cv::Vec3b	white = {255, 255, 255};
	cv::Mat	cor(height * heightScale + 10, width * widthScale, CV_8UC3, white);

	for(int w = 0; w < width; ++w)
	{
		// repeat 5 time the width of the values
		for(int wOffset = w * widthScale; wOffset < (w + 1) * widthScale; ++wOffset)
		{
		// color in black the image from the bottom to values[h] (heightScale fold)
			for(int rows = cor.rows - 1; rows >= cor.rows - round(values.at(w) * heightScale) - 1; --rows)
			{
				cor.at<cv::Vec3b>(cv::Point(wOffset, rows)) = black;
			}
			if(w == first || w == last)
			{
				for(int rows = 0; rows <= height * heightScale; ++rows)
				{
					for(int cols = w; cols <= w * widthScale; ++cols)
					{
						cor.at<cv::Vec3b>(cv::Point(cols, rows)) = red;
					}
				}
			}
		}
	}
	cv::imshow("correlation", cor);
	cv::waitKey(0);
}

double	getTheta(int hMax, int width)
{
	return radToDeg(atan((2.0 * hMax) / static_cast<double>(width)));
}

cv::Mat	correctSlope(cv::Mat const& binaryImg)
{
	int		hMax = correlation(binaryImg);
	int		index = 0;
	cv::Mat	correctedImg;
	correctedImg = cv::Mat::zeros(binaryImg.rows, binaryImg.cols, CV_8UC1);
	for(int i = 0; i < binaryImg.rows; ++i)
	{
		for(int j = 0; j < binaryImg.cols; ++j)
		{
			index = i - (2 * hMax * j / binaryImg.cols);
			if(index >= 0 && index < binaryImg.rows)
			{
				correctedImg.at<unsigned char>(i, j) = binaryImg.at<unsigned char>(index, j);
			}
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

double	getLineThickness(std::vector<int> const& histogram, unsigned int maxHisto)
{
	double				thicknessN = 0;
	double				thicknessD = 0;

	for(unsigned int e = maxHisto - 1; e <= maxHisto + 1; ++e)
	{
		thicknessN += e * histogram.at(e);
		thicknessD += histogram.at(e);
	}
	if(thicknessD != 0)
	{
		return (thicknessN / thicknessD);
	}
	return -1;	
}

std::vector<cv::Mat>	extractSubImages(cv::Mat const& binaryImg, std::vector<int> const& centerLinePositions, int interline)
{
	int						centerLinePositionsSize = static_cast<int>(centerLinePositions.size());
	std::vector<int>		profilVect;
	std::vector<int>		subImgCenter;
	std::vector<int>		subImgOrigin;
	std::vector<int>		subImgHeight;
	std::vector<cv::Mat>	subImages;
	int						hMax = 0;

	subImgCenter.reserve(centerLinePositionsSize);
	subImgOrigin.reserve(centerLinePositionsSize);
	subImgHeight.reserve(centerLinePositionsSize);
	subImages.reserve(centerLinePositionsSize);
	subImgCenter.push_back(0);
	subImgOrigin.push_back(0);
	for(int i = 1; i < centerLinePositionsSize; ++i)
	{
		subImgCenter.push_back(round((centerLinePositions.at(i - 1) + centerLinePositions.at(i)) / 2.0));
		subImgOrigin.push_back(subImgCenter.at(i) - 2 * interline);
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
		//cv::waitKey(0);
	}
	return subImages;
}

Bivector  getOrdsPosition(std::vector<cv::Mat> const& subImg, double thicknessMoy, int thickness0, int interline, std::vector<int> const& subImgCenter)
{
	int 				deltaXRange = floor(static_cast<double>(thickness0) / 2.0);
	int					deltaXPRange = round(interline / 2.0);
	Bivector			ords;
	std::vector<int>	leftOrds;
	std::vector<int>	rightOrds;
	int					profilDeltaXP = 0;
	std::vector<double>	profil;
	int					S = round(2.5 * thicknessMoy);
	int					index = 0;
	int					yMax = 0;

	leftOrds.assign(static_cast<int>(subImg.size()), -1);
	rightOrds.assign(static_cast<int>(subImg.size()), -1);
	for(int i = 0; i < static_cast<int>(subImg.size()); ++i)
	{
		profil.assign(subImg.at(i).cols, 0);
		for(int y = 0; y < subImg.at(i).cols; ++y)
		{
			for(int deltaXP = -deltaXPRange; deltaXP <= deltaXPRange; ++deltaXP)
			{
				profilDeltaXP = 0;
				for(int k = -2; k <= 2; ++k)
				{
					for(int deltaX = -deltaXRange; deltaX <= deltaXRange; ++deltaX)
					{
						index = subImgCenter.at(i) + k * interline + deltaX + deltaXP;
						if(index >= 0 && index < subImg.at(i).rows)
						{
							profilDeltaXP += ((subImg.at(i).at<unsigned char>(index, y) / 255) + 1) % 2;
						}
					}
				}
				if(profil.at(y) < profilDeltaXP)
				{
					profil.at(y) = profilDeltaXP;
				}
			}
		}
		for(int y = 0; y < subImg.at(i).cols; ++y)
		{
			if(leftOrds.at(i) == -1 && y > 0 && profil.at(y) > S && profil.at(y - 1) < S)
			{
				for(int yAfter = y + 1; yAfter < y + interline; ++yAfter)
				{
					if(profil.at(yAfter) < S)
					{
						break;
					}
					else if(yAfter == y + interline - 1)
					{
						leftOrds.at(i) = y;
					}
				}
			}
			if(y < (subImg.at(i).cols - interline) && profil.at(y) > S && profil.at(y + 1) < S)
			{
				if(y + interline - 1 < static_cast<int>(profil.size()))
				{
					yMax = y + interline;
				}
				else
				{
					yMax = static_cast<int>(profil.size());
				}
				for(int yAfter = y + 1; yAfter < yMax; ++yAfter)
				{
					if(profil.at(yAfter) > S)
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
		// show the vertical profil and the detected ordinates extrema of the staves
		//print(profil, subImg.at(i).rows, 1, subImg.at(i).cols, 1, leftOrds.at(i), rightOrds.at(i));
	}
	ords.setLeft(leftOrds);
	ords.setRight(rightOrds);
	return ords;
}

std::vector<int>	getMask(int interval, int thickness0, int staveHeight)
{	
	std::vector<int> mask;
	int				height = round(staveHeight);
	mask.assign(height, -1);
	int deltaB = floor(thickness0 / 2.0);
	int deltaH = thickness0 - 1 - deltaB;
	for(int x = 0; x < height; ++x)
	{
		for(int k = -2; k < 3; ++k)
		{
			for(int i = -deltaB; i <= deltaH; ++i)
			{
				if(x == (round(staveHeight / 2.0) + k * interval + i))
				{
					mask.at(x) = 1;
				}
			}
		}
	}
	return mask;
}

void	printCenterLineAbs(std::vector<int> const& improvedCenterLineAbsc, cv::Mat const& subImgI, int leftOrd, int rightOrd, int centerLinePosition)
{
	cv::Mat		img;
	cv::Vec3b	red = {0, 0, 255};
	cv::Vec3b	blue = {255, 0, 0};

	cvtColor(subImgI, img, CV_GRAY2RGB);
	for(int y = leftOrd; y <= rightOrd; ++y)
	{
		img.at<cv::Vec3b>(cv::Point(y, centerLinePosition)) = blue;
		img.at<cv::Vec3b>(cv::Point(y, improvedCenterLineAbsc.at(y - leftOrd))) = red;
	}
	cv::imshow("img", img);
	cv::waitKey(0);
}

void	printMask(std::vector<int> const& mask, double staveHeight)
{
	int	height = round(staveHeight);
	cv::Mat	maskImg(height, 150, CV_8UC1, cv::Scalar(255));

	for(int h = 0; h < height; ++h)
	{
		for(int w = 0; w < 150; ++w)
		{
			if(mask.at(h) == 1)
			{
				maskImg.at<unsigned char>(h, w) = 0;
			}
		}
	}
	cv::imshow("mask", maskImg);
	cv::waitKey(0);
}

std::vector<int>	getCenterLineAbsc(int centerLinePosition, int interline, int thickness0, cv::Mat subImgI, int leftOrd, int rightOrd)
{	
	double						staveHeight = 2.0 * floor(2.5 * interline);
	int							xShiftedRange = floor(interline / 2.0);
	std::vector<int>			mask = getMask(interline, thickness0, staveHeight);
	std::vector<int>			improvedCenterLineAbsc;
	cv::Mat						maskImgCorrelation;
	double						maxCor = -1.0;
	double						alpha = 0.98;
	int							shift = 0;
	unsigned char				pixCol = 0;
	double						imgValue = 0;
	double						maskValue = 0;

	if(leftOrd == -1 || rightOrd == -1)
	{
		std::cout << "left or right ordinate = -1" << std::endl;
	}
	//printMask(mask, staveHeight);
	maskImgCorrelation = cv::Mat::zeros((xShiftedRange * 2 + 1), rightOrd + 1, CV_64F);
	improvedCenterLineAbsc.assign(rightOrd - leftOrd + 1, centerLinePosition);
	// process the correlation beetwen the mask and the subImgI
	for(int y = leftOrd; y <= rightOrd; ++y)
	{
		for(int xShifted = -xShiftedRange; xShifted <= xShiftedRange; ++xShifted)
		{
			for(int h = 0; h < static_cast<int>(staveHeight); ++h)
			{
				pixCol = subImgI.at<unsigned char>(h + centerLinePosition - round(staveHeight / 2.0) + xShifted, y);
				imgValue = -1.0;
				maskValue = static_cast<double>(mask.at(h));
				if(pixCol == 0)
				{
					imgValue = 1.0;
				}
				// get the correlation of every ordinates and with x shifted aroud the abscissa of the center line of the stave
				maskImgCorrelation.at<double>(xShifted + xShiftedRange, y) += maskValue * imgValue;
			}
			maskImgCorrelation.at<double>(xShifted + xShiftedRange, y) /= staveHeight;
			if(y == leftOrd && maskImgCorrelation.at<double>(xShifted + xShiftedRange, y) > maxCor)
			{
				// store xShifted maximizing the correlation at the left ordinate of the stave
				maxCor = maskImgCorrelation.at<double>(xShifted + xShiftedRange, leftOrd);
				shift = xShifted;
			}
		}
	}
	improvedCenterLineAbsc.at(0) += shift;
	// we adjust the correlation values to smooth the detected line
	for(int y = leftOrd + 1; y <= rightOrd; ++y)
	{
		maxCor = 0.0;
		for(int xShifted = -xShiftedRange; xShifted <= xShiftedRange; ++xShifted)
		{
			// update the value of the correlation at y by weighting its value with its previous value (at y - 1)
			maskImgCorrelation.at<double>(xShifted + xShiftedRange, y) *= (1.0 - alpha);
			maskImgCorrelation.at<double>(xShifted + xShiftedRange, y) +=  (maskImgCorrelation.at<double>(xShifted + xShiftedRange, y - 1) * alpha);
			// store xShifted that maximize the filtered correlation
			if(maskImgCorrelation.at<double>(xShifted + xShiftedRange, y) > maxCor)
			{
				maxCor = maskImgCorrelation.at<double>(xShifted + xShiftedRange, y);
				shift = xShifted;
			}
		}
		// update the abscissa of the center line position
		improvedCenterLineAbsc.at(y - leftOrd) += shift;
	}
	//maskImgCorrelation.release();
	//printCenterLineAbs(improvedCenterLineAbsc, subImgI, leftOrd, rightOrd, centerLinePosition);
	return improvedCenterLineAbsc;
}
