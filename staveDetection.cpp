#include "staveDetection.hpp"
#include <cmath>
#include "tools.hpp"
#include <iostream>

// To find the angle of the slope of the staves, we use the correlation beetwen the half left part and right part of the score (the best vertical shift (h) of one of them enables us to find hMax and then the angle)
int		correlation(cv::Mat const& binaryImg)
{
	std::vector<double>	vectCor;
	double				maxCor = 0.0;
	int					hMax = 0;
	int					hRangeMax = 60;
	int					indexRowShifted = 0;

	vectCor.assign(hRangeMax, 0);
	// correlation processing
	for(int h = 0; h < hRangeMax; ++h)
	{
		for(int i = 0; i < binaryImg.rows; ++i)
		{
			for(int j = 0; j < binaryImg.cols / 2; ++j)
			{
				// the values of indexRowShifted are beetwen i + [-hRangeMax / 2; hRangeMax / 2]
				indexRowShifted = i - h + round(hRangeMax / 2.0);
				if((indexRowShifted < 0) || (indexRowShifted >= binaryImg.rows))
				{
					continue;
				}
				// Rescale the binary values of the image in the range [-1; 1] => if img(x,y) = 0, img'(x,y) = -1
				int leftPixelValue = 1;
				int rightPixelValue = 1;
				// the 'left image' [0;  width / 2]
				if(binaryImg.at<unsigned char>(i, j) == 0)
				{
					leftPixelValue = -1;
				}
				// the 'right image' [width / 2; width] shifted vertically by [-hRangeMax / 2; hRangeMax / 2]
				if(binaryImg.at<unsigned char>(indexRowShifted, j + round(binaryImg.cols / 2.0)) == 0)
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
		// get the index of the maximum value of the correlation vector (= hMax) which represents the best vertical shift ot the right image so that the lines of both images are superimposed
		if(vectCor.at(h) > maxCor)
		{
			maxCor = vectCor.at(h);
			// shift by hRangeMax / 2 so that the range of h [0 ; hRangeMax] correponds to the range of shift [-hRangeMax / 2; hRangeMax / 2]
			hMax = h - hRangeMax / 2;
		}
	}
// print results
	//std::cout << "Results : hMax = " << hMax << "    maxCor = " << maxCor << "    theta = " << getTheta(hMax, binaryImg.cols) << std::endl;
// print the image of the correlation
	//print(vectCor, round(maxCor), 500, hRangeMax, 5, -1, -1, -1, "hMax = " + std::to_string(hMax));
	return hMax;
}

// to test the validity of the found angle of the slope of the lines in the score with the best vertical shift (hMax) superimposing the lines in the right part of the image with those in the left one (it is not used for the process but it could be integrated to a unit test later
double	getTheta(int hMax, int width)
{
	return radToDeg(atan((2.0 * hMax) / static_cast<double>(width)));
}

// this function was used with different vector of values to visualize the results (correlations, vertical and horizontal profils)
void	print(std::vector<double> const& values, int height, int heightScale, int width, int widthScale, int first, int last, double S, std::string title)
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
			// print vertical threshold (used to visualize the detected first and last ordinates of the stave with the vertical profil)
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
			// visualize the minimum threshold used with the vertical profil
			if(S >= 0 && S < cor.cols)
			{
				cor.at<cv::Vec3b>(cv::Point(wOffset, (height - static_cast<int>(S)) * heightScale)) = {0, 255, 0};
			}
		}
	}
	// print image
	cv::imshow(title, cor);
	cv::waitKey(0);
}

// correction of the slope according to hMax
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

// find every index of rows of the middle line of all the staves in the whole score
std::vector<int>	detectCenterLinePos(std::vector<int> const& profilVect, int interline)
{
	std::vector<int>	centerLinePositions;
	if(interline > 0 && profilVect.size() > 0)
	{
		std::vector<int>	stavesProfilVect = getStavesProfilVect(profilVect, interline);

		centerLinePositions = getLocMaxima(stavesProfilVect, interline * 2);
	}
	return centerLinePositions;
}

// according to the processed vertical profil of the image (where the maximums correspond to the lines of the staves) we process a new profil which maxima represents the middle line of every stave
std::vector<int>	getStavesProfilVect(std::vector<int> const& profilVect, int interline)
{
	std::vector<int>	stavesProfilVect;
	if(interline > 0 && profilVect.size() > 0)
	{
		int	profilVectSize = static_cast<int>(profilVect.size());
		int	indexLineRow = 0;

		stavesProfilVect.assign(profilVectSize, 0);
		// run through all the vertical profil range of values
		for(int x = 0; x < profilVectSize; ++x)
		{
			// look for the line which is in the middle of the 5 lines of the stave
			for(int i = -2; i <= 2; ++i)
			{
				// use an epsilon range on the considered interline because it varies at beetwen the lines of a stave
				for(int j = -1; j <= 1; ++j)
				{
					// process the index of the row of the i considered line in the stave
					indexLineRow = x + i * interline + j;
					if((indexLineRow) < profilVectSize && (indexLineRow) >= 0)
					{
						stavesProfilVect.at(x) += profilVect.at(indexLineRow);
					}
				}
			}
		}
	}
	return stavesProfilVect;
}

// get the local maxima in the profil where maxima represent the middle line of every staves (the range of every considered maxima has to be near of the heigth of a stave : [-2 * interline ; 2 * interline])
std::vector<int>	getLocMaxima(std::vector<int> const& data, int range)
{
	std::vector<int>	locMaxima;
	if(range > 0 && data.size() > 0)
	{
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
	}
	return locMaxima;
}

// adapt the method of detectCenterLinePos to adjust the index of row of the middle line of stave according the vertical profil of just one stave (profilVect here is a part of the previous considered profilVect)
int	detectCenterLinePosInSub(std::vector<int> const& profilVect, int interline)
{
	//std::vector<int>	centerLinePositions;
	int	indexMax = 0;

	if(interline > 0 && profilVect.size() > 0)
	{
		int					max = 0;
		std::vector<int>	stavesProfilVect = getStavesProfilVect(profilVect, interline);
		int sizeStavesProfilVect = static_cast<int>(stavesProfilVect.size());
	
		// we only have to find one maximum because the stavesProfilVect just contains one stave
		for(int i = 0; i < sizeStavesProfilVect; ++i)
		{
			if(stavesProfilVect.at(i) > max)
			{
				max = stavesProfilVect.at(i);
				indexMax = i;
			}
		}
		/*
		cv::Mat	cor(stavesProfilVect.size(), getMax(stavesProfilVect) / 10, CV_8UC3, {255, 255, 255});

		for(int r = 0; r < cor.rows; ++r)
		{
			for(int col = cor.cols - 1; col >= cor.cols - 1 - (stavesProfilVect.at(r) / 10) - 1; --col)
			{
				cor.at<cv::Vec3b>(cv::Point(col, r)) = {0, 0, 0};
			}
		}
		for(int col = cor.cols - 1; col >= 0; --col)
		{
			cor.at<cv::Vec3b>(cv::Point(col, indexMax)) = {255, 0, 0};
		}
		cv::imshow("correlation", cor);
		cv::waitKey(0);
		*/
	}
	return indexMax;
}

// To determine the average thickness of the lines in all the staves, we process the histogram of the thickness of every lines
std::vector<int>	getLineThicknessHistogram(std::vector<int> const& middleLinePositions, int heightSize, cv::Mat const& binaryImg)
{
	std::vector<int>	histogram;
	if(middleLinePositions.size() > 0 && heightSize > 0)
	{
		int					halfHeightSize = round(heightSize / 2.0);
		int					black = 0;
		unsigned char		colorPix = 0;

		histogram.assign(heightSize, 0);
		for(int j = 0; j < binaryImg.cols; ++j)
		{
			for(auto line = middleLinePositions.begin(); line != middleLinePositions.end(); ++line)
			{
				// we run through every rows beetwen middleLineRow - staveHalfHeight to middleLineRow + staveHalfHeight
				for(int i = *line - halfHeightSize; i < *line + halfHeightSize; ++i)
				{
					if(i >= 0 && i < binaryImg.rows)
					{
						colorPix = binaryImg.at<unsigned char>(i, j);
						if(colorPix == black)
						{
							// if the color of the pixel is black => the thickness of the line is initialized at 1
							int thickness = 1;
							// increase the value of thickness if the color of the pixel in the next row (++i) is still black (== 0)
							while(++i < *line + halfHeightSize && binaryImg.at<unsigned char>(i, j) == 0)
							{
								thickness++;
							}
							// to avoid the overtaking of the definition of the vector
							if(thickness < heightSize)
							{
								// store the thickness
								++histogram.at(thickness);
							}
						}
					}
				}
			}
		}
	}
	return histogram;
}

// maxHisto represents 'thickness0', the most represented value in the histogram of the thicknesses; this functions evaluate 'thicknessAvg' by getting an average of the 3 columns in the histogram that are around thickness0
double	getLineThickness(std::vector<int> const& histogram, unsigned int maxHisto)
{
	if(histogram.size() > 0)
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
	}
	return -1;	
}

// we extract the sub images of every staves in the whole score
std::vector<cv::Mat>	extractSubImages(cv::Mat const& binaryImg, std::vector<int> const& centerLinePositions, int interline)
{
	int						centerLinePositionsSize = static_cast<int>(centerLinePositions.size());
	std::vector<int>		profilVect;
	std::vector<int>		subImgCenter;
	std::vector<int>		subImgOrigin;
	std::vector<int>		subImgHeight;
	std::vector<cv::Mat>	subImages;
	int						hMax = 0;

	// subImgCenter represents the middle of 2 successive index of rows of middle lines in staves
	subImgCenter.reserve(centerLinePositionsSize);
	// subImgOrigin represents the position of the upper line in every staves
	subImgOrigin.reserve(centerLinePositionsSize);
	// subImgHeight is the height of the sub image 
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
		// this formula has been implemented according to the expermiments of the thesis of Mrs ROSSANT, to get all the notes that are out of the lines of the staves, this is a valid height because it is greater than the height of a stave, it keeps an extra heights (according to the middle of 2 successive staves and the value of the inerline), which will be centered around the row of the middle line of the stave
		subImgHeight.push_back(subImgCenter.at(i + 1) + 2 * interline - subImgOrigin.at(i));
	}
	// the last height is processed accoring the last row of the score
	subImgHeight.push_back(binaryImg.rows - subImgOrigin.at(centerLinePositionsSize - 1));
	// filling of the sub images
	for(int i = 0; i < centerLinePositionsSize; ++i)
	{
		subImages.push_back(cv::Mat::zeros(subImgHeight.at(i), binaryImg.cols, CV_8UC1));
		for(int x = 0; x < subImgHeight.at(i); ++x)
		{
			for(int y = 0; y < binaryImg.cols; ++y)
			{
				(subImages.at(i)).at<unsigned char>(x, y) = binaryImg.at<unsigned char>(x + subImgOrigin.at(i), y);
			}
		}
		// correction of the slope of every sub image
		hMax = correlation(subImages.at(i));
		subImages.at(i) = correctSlope(subImages.at(i));
		// print sub images
		//cv::imshow(std::to_string(i), subImages.at(i));
		//cv::waitKey(0);
	}
	return subImages;
}

// Process of the left and right ordinates of every staves in every sub images with an horizontal profil of every sub images thresholded in the lest and right part
// ords is an instance of a class Bivector which contains 2 vectors : the vector of the left ordinates of the sub images and the vector of the right ones
Bivector  getOrdsPosition(std::vector<cv::Mat> const& subImg, double thicknessAvg, int thickness0, int interline, std::vector<int> const& subImgCenter)
{
	int 				deltaXRange = floor(static_cast<double>(thickness0) / 2.0) + 1;
	static_cast<void>(thickness0);
	int					deltaXPRange = round(interline / 2.0);
	Bivector			ords;
	std::vector<int>	leftOrds;
	std::vector<int>	rightOrds;
	int					profilDeltaXP = 0;
	std::vector<double>	profil;
	int					originThresh = round(2.5 * thicknessAvg);
	int					thresh = originThresh;
	int					index = 0;
	int					yMax = 0;
	int					black = 0;

	leftOrds.assign(static_cast<int>(subImg.size()), -1);
	rightOrds.assign(static_cast<int>(subImg.size()), -1);
	for(int i = 0; i < static_cast<int>(subImg.size()); ++i)
	{
		profil.assign(subImg.at(i).cols, 0);
		for(int y = 0; y < subImg.at(i).cols; ++y)
		{
			// deltaXPRange range is [-interline / 2; interline / 2] to evaluate the best vertical shift in this range of values that maximizes the horizontal profil and then represents the best shift to find the nearest values of the middle of every lines in a stave
			for(int deltaXP = -deltaXPRange; deltaXP <= deltaXPRange; ++deltaXP)
			{
				profilDeltaXP = 0;
				for(int k = -2; k <= 2; ++k)
				{
					// deltaX range is approximately [-thickness0 / 2; thickness0 / 2] to run through all the rows that define a single line (all the thickness)
					for(int deltaX = -deltaXRange; deltaX <= deltaXRange; ++deltaX)
					{
						index = subImgCenter.at(i) + k * interline + deltaX + deltaXP;
						if(index >= 0 && index < subImg.at(i).rows)
						{
							if(subImg.at(i).at<unsigned char>(index, y) == black)
							{
								profilDeltaXP += 1;
							}
						}
					}
				}
				// we keep the maximum values of the profil according to the deltaXPRange (best 
				if(profil.at(y) < profilDeltaXP)
				{
					profil.at(y) = profilDeltaXP;
				}
			}
		}
		thresh = originThresh + 1;
		//  finding the left and right ordinates of a stave
		while((leftOrds.at(i) == -1 || rightOrds.at(i) == -1) && --thresh >= 0)
		{
			for(int y = 0; y < subImg.at(i).cols; ++y)
			{
				// if leftOrds of the considered sub image has not been assign yet, and if the value of the horizontal profil at the considered column is above the threshold value, and if every values of the profil on the next columns (on a width of 1 interline) is greater than the threshold, then we define the left ordinate of the stave as this column
				if(leftOrds.at(i) == -1 && y > 0 && profil.at(y) > thresh)
				{
					yMax = y + interline;
					if(interline < 4)
					{
						yMax = y + 5;
					}
					for(int yAfter = y + 1; yAfter < yMax; ++yAfter)
					{
						if(profil.at(yAfter) < thresh)
						{
							break;
						}
						else if(yAfter == yMax - 1)
						{
							leftOrds.at(i) = y;
						}
					}
				}
				// same principle than above, is every values after the considered colum in the profil are below the threshol, this is the right ordinate of the stave
				if(y < (subImg.at(i).cols - interline) && profil.at(y) > thresh && profil.at(y + 1) < thresh)
				{
					yMax = static_cast<int>(profil.size());
					if(y + interline - 1 < static_cast<int>(profil.size()))
					{
						yMax = y + interline;
					}
					for(int yAfter = y + 1; yAfter < yMax; ++yAfter)
					{
						if(profil.at(yAfter) > thresh)
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
		// print the vertical profil and the detected ordinates extrema of the staves
		//print(profil, subImg.at(i).rows, 1, subImg.at(i).cols, 1, leftOrds.at(i), rightOrds.at(i), thresh);
	}
	// store the values in the Bivector 'ords'
	ords.setLeft(leftOrds);
	ords.setRight(rightOrds);
	return ords;
}

// this mask is a vector which length is the same as the considered stave and which takes the value 1 when a line is supposed to be printed at this row, and -1 otherwise
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

// this helps to visualize the adjusted rows (different from one column to the other) of the middle lines of the stave compared to the previous constant row
void	printCenterLineAbs(std::vector<int> const& improvedCenterLineAbsc, cv::Mat const& subImgI, int leftOrd, int rightOrd, int centerLinePosition)
{
	cv::Mat		img;
	cv::Vec3b	red = {0, 0, 255};
	cv::Vec3b	blue = {255, 0, 0};

	cvtColor(subImgI, img, cv::COLOR_GRAY2RGB);
	for(int y = leftOrd; y <= rightOrd; ++y)
	{
		img.at<cv::Vec3b>(cv::Point(y, centerLinePosition)) = blue;
		img.at<cv::Vec3b>(cv::Point(y, improvedCenterLineAbsc.at(y - leftOrd))) = red;
	}
	cv::imshow("img", img);
	cv::waitKey(0);
}

// this prints the mask with a width of 150
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

// this process the 'algorithme de poursuite des portées' => tracking of staves algorithm in the thesis : it calculates the better abscissa (row) of the middle line of a stave at every column
std::vector<int>	getCenterLineAbsc(int centerLinePosition, int interline, int thickness0, cv::Mat subImgI, int leftOrd, int rightOrd)
{	
	std::vector<int>			improvedCenterLineAbsc;
	double						staveHeight = 2.0 * floor(2.5 * interline);
	int							xShiftedRange = floor(interline / 2.0);
	std::vector<int>			mask = getMask(interline, thickness0, staveHeight);
	cv::Mat						maskImgCorrelation;
	double						maxCor = -1.0;
	double						alpha = 0.98;
	int							shift = 0;
	unsigned char				pixCol = 0;
	double						imgValue = 0;
	double						maskValue = 0;
	int							startY = leftOrd - 1;
	bool						trueYFound = false;
	int							whitePix = 0;
	int							blackPix = 0;

	// if the left and right ordinates has not been detected in even one sub image, they will be equal to -1 which leads to a segmentation fault in this process, so we better have to check
	if(leftOrd >= 0 && rightOrd >= 0 && !subImgI.empty() && centerLinePosition > 0 && interline > 0 && thickness0 > 0)
	{
		//printMask(mask, staveHeight);
		maskImgCorrelation = cv::Mat::zeros((xShiftedRange * 2 + 1), rightOrd + 1, CV_64F);
		improvedCenterLineAbsc.assign(rightOrd - leftOrd + 1, centerLinePosition);

		// start to process the correlation when all the pixels at startY ordinate are no more all equal to 0 (avoid the first black vertical line before the keys)
		while(trueYFound == false)
		{
			whitePix = 0;
			blackPix = 0;
			++startY;
			for(int h = 0; h < static_cast<int>(staveHeight); ++h)
			{
				pixCol = subImgI.at<unsigned char>(h + centerLinePosition - round(staveHeight / 2.0), startY);
				if(pixCol == 255)
				{
					++whitePix;
				}
				else
				{
					++blackPix;
				}
			}
			if(whitePix > blackPix)
			{
				trueYFound = true;
				break;
			}
		}
		// process the correlation beetwen the mask and the subImgI
		for(int y = startY; y <= rightOrd; ++y)
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
				if(y == startY && maskImgCorrelation.at<double>(xShifted + xShiftedRange, y) > maxCor)
				{
					// store xShifted maximizing the correlation at the left ordinate of the stave
					maxCor = maskImgCorrelation.at<double>(xShifted + xShiftedRange, y);
					shift = xShifted;
					// keep the same value of correlation at the ordinates of the black vertical line in the begining of the stave
					for(int yShift = leftOrd; yShift < startY; ++yShift)
					{
						maskImgCorrelation.at<double>(xShifted + xShiftedRange, yShift) = maxCor;
					}
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
		maskImgCorrelation.release();
		//printCenterLineAbs(improvedCenterLineAbsc, subImgI, leftOrd, rightOrd, centerLinePosition);
	}
	return improvedCenterLineAbsc;
}
