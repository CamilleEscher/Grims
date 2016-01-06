#include "staveDetection.hpp"
#include <cmath>
#include "tools.hpp"
#include <iostream>

/*!
  	\brief
	Get the local maxima in the profile where maxima represent the middle line of every stave (the range of every considered maximum has to be near of the height of a stave : [-2 * interline ; 2 * interline])

	\param data vector of data where local maxima have to be found
	\param range minimum range between 2 local maxima
*/
static std::vector<int>	getLocMaxima(std::vector<int> const& data, int range);

/*!
  \brief
  Get the best value of the vertical profile of one considered column by getting the maximum value of the vertical profile around the theoretical abscissa of the lines according to middleLineAbscs shifted in the range [-thickness0 / 2; thickness0 / 2].
  Called by getOrdsPosition

  \param interline see getStavesProfileVect
  \param middleLineAbsc abscissa of the third line of the considered stave
  \param subImg sub image of one stave of the score
  \param thickness0 most represented value in the histogram of the thicknesses of line
  \param col considered column
*/
static int	getMaxDeltaOrdProfile(int interline, int subImgCenter, cv::Mat const& subImg, int thickness0, int col);

/*!
  \brief
  Find the left ordinate of the stave by running through every column of the sub image and keeping the one with a corresponding horizontal profile above the threshold and every values of the profile of the next columns (on a width of 2 interlines) greater than the same threshold

  \param profile vertical profile of the stave
  \param thresh minimum number of black pixel corresponding to the thickness of 5 lines of staves
  \param subImgWidth width of the image of the stave
  \param interline see getStavesProfileVect
*/
static int	getLeftOrd(std::vector<double> profile, int thresh, int subImgWidth, int interline);

/*!
   \brief
   Transpose getLeftOrd

*/
static int	getRightOrd(std::vector<double> profile, int thresh, int subImgWidth, int interline);

/*!
  	\brief
	mask is a vector which length is the same as the considered stave and which takes the value 1 when a line is supposed to be printed at this row, and -1 otherwise

	\param interline see getStavesProfileVect
	\param thickness0 see getMaxDeltaOrdProfile
	\param staveHeight heigth of the sub image of the stave
*/
static std::vector<int>	getMask(int interline, int thickness0, int staveHeight);

/*!
  	\brief
	Find the startY ordinate which pixels are no more all equal to 0 (avoid the first black vertical line before the keys to initialize the correlation process)

	\param staveHeight height of the sub image of the stave
	\param subImgI see subImg in getMaxDeltaOrdProfile
	\param startY first ordinate of the sub image of stave after the vertical bar before the key. This param is reprocessed in this function
	\param middleLineAbsc see getMaxDeltaOrdProfile
*/
static void	findStartY(double staveHeight, cv::Mat const& subImgI, int& startY, int middleLineAbsc);

/*!
  	\brief
	Process the correlation beetwen the mask and the subImgI

	\param startY see findStartY
	\param leftOrd see getMask
	\param rightOrd see getMask
	\param xShiftedRange equals to interline / 2
	\param staveHeight see getMiddleLineAbsc
	\param middleLineAbsc see getMaxDeltaOrdProfile
	\param interline see getStavesProfileVect
	\param shift equals 0. Modified in this function. Represents the vertical shift to add to get the maximum of correlation between the mask and the image of stave
	\param thickness0 see getMaxDeltaOrdProfile
	\param subImgI see subImg in getMaxDeltaOrdProfile
*/
static cv::Mat	processMaskImgCorrelation(int startY, int leftOrd, int rightOrd, int xShiftedRange, double staveHeight, int middleLineAbsc, int interline, int& shift, int thickness0, cv::Mat const& subImgI);

int		correlation(cv::Mat const& binaryImg)
{
	std::vector<double>	vectCor;
	double				maxCor = 0.0;
	int					hMax = 0;
	int					hRangeMax = 60;
	int					indexRowShifted = 0;
	int 				leftPixelValue = 1;
	int 				rightPixelValue = 1;
	int					halfImgWidth = std::round(binaryImg.cols / 2.0);

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
				leftPixelValue = 1;
				rightPixelValue = 1;
				// the 'left image' [0;  width / 2]
				if(binaryImg.at<unsigned char>(i, j) == 0)
				{
					leftPixelValue = -1;
				}
				// the 'right image' [width / 2; width] shifted vertically by [-hRangeMax / 2; hRangeMax / 2]
				if(binaryImg.at<unsigned char>(indexRowShifted, j + halfImgWidth) == 0)
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
	return hMax;
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

std::vector<int>	detectMiddleLineAbsc(std::vector<int> const& profileVect, int interline)
{
	std::vector<int>	middleLineAbscs;
	std::vector<int>	stavesProfileVect;

	if(interline > 0 && profileVect.size() > 0)
	{
		stavesProfileVect = getStavesProfileVect(profileVect, interline);
		middleLineAbscs = getLocMaxima(stavesProfileVect, interline * 2);
	}
	return middleLineAbscs;
}

std::vector<int>	getStavesProfileVect(std::vector<int> const& profileVect, int interline)
{
	std::vector<int>	stavesProfileVect;
	int	profileVectSize = static_cast<int>(profileVect.size());
	int	indexLineRow = 0;

	if(interline > 0 && profileVect.size() > 0)
	{
		stavesProfileVect.assign(profileVectSize, 0);
		// run through all the range of value of the vertical profile
		for(int x = 0; x < profileVectSize; ++x)
		{
			// look for the line which is in the middle of the 5 lines of the stave
			for(int i = -2; i <= 2; ++i)
			{
				// use an epsilon range on the considered interline because it varies beetwen every line of stave
				for(int j = -1; j <= 1; ++j)
				{
					// process the index of the row of the i-th considered line in the stave
					indexLineRow = x + i * interline + j;
					if((indexLineRow) < profileVectSize && (indexLineRow) >= 0)
					{
						stavesProfileVect.at(x) += profileVect.at(indexLineRow);
					}
				}
			}
		}
	}
	return stavesProfileVect;
}

static std::vector<int>	getLocMaxima(std::vector<int> const& data, int range)
{
	std::vector<int>	locMaxima;
	int					sizeData = static_cast<int>(data.size());
	int					initMax = 0;
	int					max = 0;
	int					indexMax = 0;

	if(range > 0 && data.size() > 0)
	{
		initMax = std::round(getMax(data) / 3.0);
		max = initMax;
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

int	detectMiddleLineAbscInSub(std::vector<int> const& profileVect, int interline)
{
	int					indexMax = 0;
	int					max = 0;
	std::size_t			sizeStavesProfileVect = 0;
	std::vector<int>	stavesProfileVect;

	if(interline > 0 && profileVect.size() > 0)
	{
		stavesProfileVect = getStavesProfileVect(profileVect, interline);
		sizeStavesProfileVect = stavesProfileVect.size();
	
		// we only have to find one maximum because the stavesProfileVect just contains one stave
		for(auto i = 0u; i < sizeStavesProfileVect; ++i)
		{
			if(stavesProfileVect.at(i) > max)
			{
				max = stavesProfileVect.at(i);
				indexMax = i;
			}
		}
	}
	return indexMax;
}

std::vector<int>	getLineThicknessHistogram(std::vector<int> const& middleLineAbscs, int heightSize, cv::Mat const& binaryImg)
{
	std::vector<int>	histogram;
	int					halfHeightSize = 0;
	int					black = 0;
	unsigned char		colorPix = 0;

	if(middleLineAbscs.size() > 0 && heightSize > 0)
	{
		halfHeightSize = std::round(heightSize / 2.0);
		histogram.assign(heightSize, 0);
		for(int j = 0; j < binaryImg.cols; ++j)
		{
			for(auto line = middleLineAbscs.begin(); line != middleLineAbscs.end(); ++line)
			{
				// we run through every row beetwen middleLineRow - staveHalfHeight to middleLineRow + staveHalfHeight
				for(int i = *line - halfHeightSize; i < *line + halfHeightSize; ++i)
				{
					if(i >= 0 && i < binaryImg.rows)
					{
						colorPix = binaryImg.at<unsigned char>(i, j);
						if(colorPix == black)
						{
							// if the color of pixel is black => the thickness of line is initialized at 1
							int thickness = 1;
							// increase the value of thickness if the color of pixel in the next row (++i) is still black (== 0)
							while(++i < *line + halfHeightSize && binaryImg.at<unsigned char>(i, j) == 0)
							{
								++thickness;
							}
							// to avoid the overtaking of the definition of our vector
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

double	getLineThickness(std::vector<int> const& histogram, unsigned int maxHisto)
{
	double				thicknessN = 0;
	double				thicknessD = 0;

	if(histogram.size() > 0)
	{
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

std::vector<cv::Mat>	extractSubImages(cv::Mat const& binaryImg, std::vector<int> const& middleLineAbscs, int interline)
{
	int						middleLineAbscsSize = static_cast<int>(middleLineAbscs.size());
	std::vector<int>		profileVect;
	std::vector<int>		subImgCenter;
	std::vector<int>		subImgOrigin;
	std::vector<int>		subImgHeight;
	std::vector<cv::Mat>	subImages;

	// subImgCenter represents the middle of 2 successive middle lines of the staves in the page of score
	subImgCenter.reserve(middleLineAbscsSize);
	// subImgOrigin represents the position of the upper line in every stave
	subImgOrigin.reserve(middleLineAbscsSize);
	// subImgHeight is the height of the sub image 
	subImgHeight.reserve(middleLineAbscsSize);
	subImages.reserve(middleLineAbscsSize);
	subImgCenter.push_back(0);
	subImgOrigin.push_back(0);
	for(int i = 1; i < middleLineAbscsSize; ++i)
	{
		subImgCenter.push_back(round((middleLineAbscs.at(i - 1) + middleLineAbscs.at(i)) / 2.0));
		subImgOrigin.push_back(subImgCenter.at(i) - 2 * interline);
	}
	for(int i = 0; i < middleLineAbscsSize - 1; ++i)
	{
		// this formula has been implemented according to the expermiments of the thesis of Mrs ROSSANT, to get all the notes that are out of the lines of the staves, this is a valid height because it is greater than the height of a stave, it keeps extra heights (according to the middle of 2 successive staves and the value of inerline), which will be centered around the row of the middle line of the stave
		subImgHeight.push_back(subImgCenter.at(i + 1) + 2 * interline - subImgOrigin.at(i));
	}
	// the last height is processed accoring the last row of the score
	subImgHeight.push_back(binaryImg.rows - subImgOrigin.at(middleLineAbscsSize - 1));
	// fill the sub images
	for(int i = 0; i < middleLineAbscsSize; ++i)
	{
		subImages.push_back(cv::Mat::zeros(subImgHeight.at(i), binaryImg.cols, CV_8UC1));
		for(int x = 0; x < subImgHeight.at(i); ++x)
		{
			for(int y = 0; y < binaryImg.cols; ++y)
			{
				(subImages.at(i)).at<unsigned char>(x, y) = binaryImg.at<unsigned char>(x + subImgOrigin.at(i), y);
			}
		}
		//correction of the slope of every sub image
		subImages.at(i) = correctSlope(subImages.at(i));
		// print sub images
		//cv::imshow(std::to_string(i), subImages.at(i));
		//cv::waitKey(0);
	}
	return subImages;
}

static int	getMaxDeltaOrdProfile(int interline, int subImgCenter, cv::Mat const& subImg, int thickness0, int col)
{
	int		profileDeltaXP = 0;
	int 	black = 0;
	int		maxProfile = 0;
	int 	deltaXRange = std::floor(static_cast<double>(thickness0) / 2.0) + 1;
	int		deltaXPRange = std::round(interline / 2.0);
	int		index = 0;
	
	// deltaXPRange range is [-interline / 2; interline / 2] to evaluate the best vertical shift in this range of value that maximizes the horizontal profile and then represents the best shift to find the nearest value of the middle of every line in a stave
	for(int deltaXP = -deltaXPRange; deltaXP <= deltaXPRange; ++deltaXP)
	{
		profileDeltaXP = 0;
		for(int k = -2; k <= 2; ++k)
		{
			// deltaX range is approximately [-thickness0 / 2; thickness0 / 2] to run through all the rows that define a single line (all the thickness)
			for(int deltaX = -deltaXRange; deltaX <= deltaXRange; ++deltaX)
			{
				index = subImgCenter + k * interline + deltaX + deltaXP;
				if(index >= 0 && index < subImg.rows)
				{
					if(subImg.at<unsigned char>(index, col) == black)
					{
						profileDeltaXP += 1;
					}
				}
			}
		}
		// we keep the maximum values of the profile according to the deltaXPRange
		if(maxProfile < profileDeltaXP)
		{
			maxProfile = profileDeltaXP;
		}
	}
	return maxProfile;
}

static int	getLeftOrd(std::vector<double> profile, int thresh, int subImgWidth, int interline)
{
	int yMax = 0;

	for(int y = 0; y < subImgWidth; ++y)
	{
		if(y > 0 && profile.at(y) > thresh)
		{
			yMax = y + interline * 2;
			for(int yAfter = y + 1; yAfter < yMax; ++yAfter)
			{
				if(profile.at(yAfter) < thresh)
				{
					break;
				}
				else if(yAfter == yMax - 1)
				{
					return y;
				}
			}
		}
	}
	return -1;
}

static int	getRightOrd(std::vector<double> profile, int thresh, int subImgWidth, int interline)
{
	int	rightOrd = -1;

	for(int y = interline + 1; y < subImgWidth; ++y)
	{
		if(profile.at(y) > thresh)
		{
			for(int yBefore = y - interline; yBefore < y; ++yBefore)
			{
				if(profile.at(yBefore) < thresh)
				{
					break;
				}
				else if(yBefore == y - 1)
				{
					rightOrd = y;
				}
			}
		}
	}
	return rightOrd;
}

Bivector  getOrdsPosition(std::vector<cv::Mat> const& subImg, double thicknessAvg, int thickness0, int interline, std::vector<int> const& subImgCenter)
{
	Bivector			ords;
	std::vector<int>	leftOrds;
	std::vector<int>	rightOrds;
	std::vector<double>	profile;
	int					originThresh = round(2.5 * thicknessAvg);
	int					thresh = originThresh + 1;

	leftOrds.assign(static_cast<int>(subImg.size()), -1);
	rightOrds.assign(static_cast<int>(subImg.size()), -1);
	for(int i = 0; i < static_cast<int>(subImg.size()); ++i)
	{
		profile.clear();
		profile.reserve(subImg.at(i).cols);
		for(int y = 0; y < subImg.at(i).cols; ++y)
		{
			profile.push_back(getMaxDeltaOrdProfile(interline, subImgCenter.at(i), subImg.at(i), thickness0, y));
		}
		//  finding the left and right ordinates of a stave
		while(leftOrds.at(i) == -1 && --thresh >= 0)
		{
			leftOrds.at(i) = getLeftOrd(profile, thresh, subImg.at(i).cols, interline);
		}
		thresh = originThresh + 1;
		while(rightOrds.at(i) == -1 && --thresh >= 0)
		{
			rightOrds.at(i) = getRightOrd(profile, thresh, subImg.at(i).cols, interline);
		}
	}
	// store the values in the Bivector 'ords'
	ords.setLeft(leftOrds);
	ords.setRight(rightOrds);
	return ords;
}

static std::vector<int>	getMask(int interline, int thickness0, int staveHeight)
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
				if(x == (round(staveHeight / 2.0) + k * interline + i))
				{
					mask.at(x) = 1;
				}
			}
		}
	}
	return mask;
}

std::vector<int>	getMiddleLineAbsc(int middleLineAbsc, int interline, int thickness0, cv::Mat subImgI, int leftOrd, int rightOrd)
{	
	std::vector<int>			improvedCenterLineAbsc;
	double						staveHeight = 2.0 * floor(2.5 * interline);
	int							xShiftedRange = floor(interline / 2.0);
	cv::Mat						maskImgCorrelation;
	double						maxCor = -1.0;
	double						alpha = 0.98;
	int							shift = 0;
	int							startY = leftOrd - 1;

	// if the left and right ordinates has not been detected in even one sub image, they will be equal to -1 which leads to a segmentation fault in this process, so we better have to check
	if(leftOrd >= 0 && rightOrd >= 0 && !subImgI.empty() && middleLineAbsc > 0 && interline > 0 && thickness0 > 0)
	{
		improvedCenterLineAbsc.assign(rightOrd - leftOrd + 1, middleLineAbsc);

		findStartY(staveHeight, subImgI, startY, middleLineAbsc);
		maskImgCorrelation = processMaskImgCorrelation(startY, leftOrd, rightOrd, xShiftedRange, staveHeight, middleLineAbsc, interline, shift, thickness0, subImgI);
		improvedCenterLineAbsc.at(0) += shift;
		// adjust the correlation values to smooth the detected line
		for(int y = leftOrd + 1; y <= rightOrd; ++y)
		{
			maxCor = 0.0;
			for(int xShifted = -xShiftedRange; xShifted <= xShiftedRange; ++xShifted)
			{
				// update the value of the correlation at y by weighting its value with its previous value (at y - 1)
				maskImgCorrelation.at<double>(xShifted + xShiftedRange, y) *= (1.0 - alpha);
				maskImgCorrelation.at<double>(xShifted + xShiftedRange, y) +=  (maskImgCorrelation.at<double>(xShifted + xShiftedRange, y - 1) * alpha);
				// store xShifted maximizing the filtered correlation
				if(maskImgCorrelation.at<double>(xShifted + xShiftedRange, y) > maxCor)
				{
					maxCor = maskImgCorrelation.at<double>(xShifted + xShiftedRange, y);
					shift = xShifted;
				}
			}
			// update the abscissa of the middle line
			improvedCenterLineAbsc.at(y - leftOrd) += shift;
		}
		maskImgCorrelation.release();
	}
	return improvedCenterLineAbsc;
}

static void	findStartY(double staveHeight, cv::Mat const& subImgI, int& startY, int middleLineAbsc)
{
	int 			whitePix = 0;
	int 			blackPix = 0;
	int				halfStaveHeight = round(staveHeight / 2.0);
	int				stopCondition = round(subImgI.cols / 3.0);
	unsigned char	pixCol = 0;

	while(true)
	{
		whitePix = 0;
		blackPix = 0;
		++startY;
		for(int h = -halfStaveHeight; h < halfStaveHeight; ++h)
		{
			pixCol = subImgI.at<unsigned char>(h + middleLineAbsc, startY);
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
			break;
		}
		if(startY > stopCondition)
		{
			std::cout << "Error in findStartY() : could not find startY" << std::endl;
			break;
		}
	}
}

static cv::Mat	processMaskImgCorrelation(int startY, int leftOrd, int rightOrd, int xShiftedRange, double staveHeight, int middleLineAbsc, int interline, int& shift, int thickness0, cv::Mat const& subImgI)
{
	cv::Mat						maskImgCorrelation;
	double						maxCor = -1.0;
	int							halfStaveHeight = std::round(staveHeight / 2.0);
	unsigned char				pixCol = 0;
	std::vector<int>			mask = getMask(interline, thickness0, staveHeight);
	double						imgValue = 0;
	double						maskValue = 0;

	maskImgCorrelation = cv::Mat::zeros((xShiftedRange * 2 + 1), rightOrd + 1, CV_64F);
	for(int y = startY; y <= rightOrd; ++y)
	{
		for(int xShifted = -xShiftedRange; xShifted <= xShiftedRange; ++xShifted)
		{
			for(int h = -halfStaveHeight; h < halfStaveHeight; ++h)
			{
				pixCol = subImgI.at<unsigned char>(h + middleLineAbsc + xShifted, y);
				imgValue = -1.0;
				maskValue = static_cast<double>(mask.at(h + halfStaveHeight));
				if(pixCol == 0)
				{
					imgValue = 1.0;
				}
				// get the correlation of every ordinate and with x shifted aroud the abscissa of the middle line of the stave
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
	return maskImgCorrelation;
}
