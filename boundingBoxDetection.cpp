#include "boundingBoxDetection.hpp"

void	gatherImages(std::vector<Stave> const& staves)
{
	std::vector<cv::Mat>	subImgV;
	cv::Mat					subImg;
	int						subImagesNb = static_cast<unsigned int>(staves.size());
	Stave					previousStave = staves.at(0);

	subImgV.reserve(subImagesNb);
    for(int i = 0; i < subImagesNb; ++i)
	{
		Stave const& stave = staves.at(i);
		subImg = stave.getStaveImg();
		// seek vertical segments in subImg
		subImgV.push_back(getVerticalSegmentsMap(subImg));
		if(i > 0)
		{
			int absUp = previousStave.getStaveLines().at(2).getAbsCoords().at(0);	
			int absDown = stave.getStaveLines().at(2).getAbsCoords().at(0);
			bool arePaired = areStavesPaired(subImgV.at(i - 1), subImgV.at(i), absUp, absDown);
			if(arePaired)
			{
				subImg = gatherImages(previousStave.getStaveImg(), stave.getStaveImg(), stave.getStaveLines().at(0).getAbsCoords().at(0));
				cv::imshow("gathered " + std::to_string(i), subImg);
				cv::waitKey(0);
			}
		}
		previousStave = stave;
	}
}

// this process represents 4 of 6 steps to get the bounding boxes, the rest is not implemented yet
std::vector<cv::Mat>	highLightVerticals(std::vector<Stave> const& staves)
{
	// erase() has to be applied before entering this function
	std::vector<cv::Mat>	subImgHInV;
	std::vector<cv::Mat>	subImgV;
	std::vector<cv::Mat>	subImgH;
	std::vector<cv::Mat>	subImagesClosed;
	cv::Mat					subImg;
	int						subImagesNb = static_cast<unsigned int>(staves.size());

	subImgHInV.reserve(subImagesNb);
	subImgV.reserve(subImagesNb);
	subImgH.reserve(subImagesNb);
	subImagesClosed.reserve(subImagesNb);
    for(int i = 0; i < subImagesNb; ++i)
	{
		//Stave const& stave = staves.at(i);
		subImg = staves.at(i).getStaveImg();
		// extract horizontal segments which belongs to vertical segments
		subImgHInV.push_back(getHorizontalSegInVerticalSeg(subImg));
		// closing according to the maxima of horLinesImg
		subImagesClosed.push_back(applyClosingOperation(subImgHInV.at(i), subImg));
		// extract horizontal segments
		subImgH.push_back(getHorizontalSegmentsMap(subImagesClosed.at(i)));
		// extract vertical segments on the closed sub image
		subImgV.push_back(getVerticalSegmentsMap(subImagesClosed.at(i)));
		cv::imshow("verticals of image " + std::to_string(i), subImgV.at(i));
		cv::waitKey(0);
	}
	return subImgV;
}

cv::Mat	gatherImages(cv::Mat const&	imgUp, cv::Mat const& imgDown, int absDownUpperStaveLine)
{
	cv::Mat	bottomRectInUp = imgUp(cv::Rect(0, imgUp.rows - 21, imgUp.cols, 20));
	cv::Mat	gatheredImg;

	for(int i = absDownUpperStaveLine; i >= 20; --i)
	{
		cv::Mat	upperRectInDown = imgDown(cv::Rect(0, i - 20, imgDown.cols, 20));
		if(compareRectangles(bottomRectInUp, upperRectInDown))
		{
			gatheredImg = cv::Mat::zeros(imgUp.rows + imgDown.rows - i, imgDown.cols, CV_8UC1);
			for(int x = 0; x < gatheredImg.cols; ++x)
			{
				for(int y = 0; y < gatheredImg.rows; ++y)
				{
					if(y < imgUp.rows)
					{
						gatheredImg.at<unsigned char>(y, x) = imgUp.at<unsigned char>(y, x);
					}
					else if(y - imgUp.rows + 1 < imgDown.rows && y - imgUp.rows + 1 >= 0)
					{
						gatheredImg.at<unsigned char>(y, x) = imgDown.at<unsigned char>(y - imgUp.rows + i + 1, x);	
					}
				}
			}
			break;
		}
	}
	return gatheredImg;
}

bool	compareRectangles(cv::Mat const& rectUp, cv::Mat const& rectDown)
{
	if(rectUp.size() == rectDown.size())
	{
		for(int x = 0; x < rectUp.cols; ++x)
		{
			for(int y = 0; y < rectUp.rows; ++y)
			{
				if(rectUp.at<unsigned char>(y, x) != rectDown.at<unsigned char>(y, x))
				{
					return false;
				}
			}
		}
	}
	return true;
}

bool	areStavesPaired(cv::Mat const& subImgVUp, cv::Mat const& subImgVDown, int absCoordMiddleLineUp, int absCoordMiddleLineDown)
{
	unsigned char				pixCol = 0;
	int							minLengthUp = 0;
	int							minLengthDown = 0;
	int							paired = 0;
	int							lastIndexDetected = -1;

	minLengthUp = subImgVUp.rows - absCoordMiddleLineUp;
	minLengthDown = absCoordMiddleLineDown;
	if(minLengthUp > 255)
	{
		minLengthUp = 255;
	}
	if(minLengthDown > 255)
	{
		minLengthDown = 255;
	}
	for(int i = 0; i < subImgVUp.cols; ++i)
	{
		pixCol = subImgVUp.at<unsigned char>(subImgVUp.rows - 1, i);
		if(pixCol > static_cast<unsigned char>(minLengthUp))
		{
			if(i > 0 && lastIndexDetected == i - 1)
			{
				lastIndexDetected = i;
			}
			else
			{
				for(int k = i - 2; k < i + 3; ++k)
				{
					if(k >= 0 && k < subImgVDown.cols)
					{
						if(subImgVDown.at<unsigned char>(0, k) > static_cast<unsigned char>(minLengthDown))
						{
							++paired;
							lastIndexDetected = i;
							break;
						}
					}
				}
			}
		}
	}
	return (paired > 2);
}

cv::Mat	getVerticalSegmentsMap(cv::Mat const& subImg)
{
	unsigned char			lineLength = 0;
	int						xShift = 0;
	cv::Mat					vertLinesImg;

	vertLinesImg = cv::Mat::zeros(subImg.rows, subImg.cols, CV_8UC1);
	for(int y = 0; y < vertLinesImg.cols; ++y)
	{
		for(int x = 0; x < vertLinesImg.rows; ++x)
		{
			lineLength = 0;
			xShift = x;
			while(xShift < vertLinesImg.rows && subImg.at<unsigned char>(xShift, y) == 0)
			{
				if(lineLength <= 255)
				{
					++lineLength;
				}
				++xShift;
			}
			for(int xLine = x; xLine < xShift; ++xLine)
			{
				vertLinesImg.at<unsigned char>(xLine, y) = lineLength;
			}
			if(xShift < vertLinesImg.rows)
			{
				x = xShift;
			}
			else
			{
				x = vertLinesImg.rows - 1;
			}
		}
	}
	return vertLinesImg;
}

cv::Mat	getHorizontalSegmentsMap(cv::Mat const& subImg)
{
	unsigned char			lineLength = 0;
	int						yShift = 0;
	cv::Mat					horSegmentsMap;

	horSegmentsMap = cv::Mat::zeros(subImg.rows, subImg.cols, CV_8UC1);
	for(int x = 0; x < horSegmentsMap.rows; ++x)
	{
		for(int y = 0; y < horSegmentsMap.cols; ++y)
		{
			lineLength = 0;
			yShift = y;
			while(yShift < horSegmentsMap.cols && subImg.at<unsigned char>(x, yShift) == 0) 
			{
				if(lineLength <= 255)
				{
					++lineLength;
				}
				++yShift;
			}
			for(int yLine = y; yLine < yShift; ++yLine)
			{
				horSegmentsMap.at<unsigned char>(x, yLine) = lineLength;
			}
			if(yShift < horSegmentsMap.cols)
			{
				y = yShift;
			}
			else
			{
				y = horSegmentsMap.cols - 1;
			}
		}
	}
	return horSegmentsMap;
}

cv::Mat	getHorizontalSegInVerticalSeg(cv::Mat const& subImg)
{
	cv::Mat				horLinesImg;
	std::vector<float>	kernel;
	cv::Mat				negSubImg = 255 - subImg;
	float				pixCol = 0;
	
	// assign horLinesImg 
	horLinesImg = cv::Mat::zeros(subImg.rows, subImg.cols, CV_8UC1);
	// assign kernel
	kernel.assign(9, 0);
	kernel.at(0) = 0.25f;
	kernel.at(1) = 0.25f;
	kernel.at(7) = 0.25f;
	kernel.at(8) = 0.25f;	
	// process convolution beetwen the kernel and the negSubImg then convolve with subImg
	for(int y = 0; y < horLinesImg.rows; ++y)
	{
		for(int x = 4; x < horLinesImg.cols - 4; ++x)
		{
			pixCol = 0;
			for(int xEps = -4; xEps < 5; ++xEps)
			{
				pixCol += (static_cast<float>(subImg.at<unsigned char>(y, x + xEps) / 255) * kernel.at(xEps + 4));
			}
			pixCol *= static_cast<float>(negSubImg.at<unsigned char>(y, x) / 255);
			switch(static_cast<int>(pixCol * 100))
			{
				case 25 : 
					horLinesImg.at<unsigned char>(y, x) = 70;
					break;
				case 50 : 
					horLinesImg.at<unsigned char>(y, x) = 130;
					break;
				case 75 : 
					horLinesImg.at<unsigned char>(y, x) = 190;
					break;
				case 100 : 
					horLinesImg.at<unsigned char>(y, x) = 255;
					break;
				default :
					break;
			}
		}
	}
	return horLinesImg;
}

cv::Mat applyClosingOperation(cv::Mat const& horLinesImg, cv::Mat const& subImg)
{
	cv::Mat	newSubImg = subImg.clone();
	for(int x = 0; x < horLinesImg.cols; ++x)
	{
		for(int y = 1; y < horLinesImg.rows - 2; ++y)
		{
			if(horLinesImg.at<unsigned char>(y - 1, x) == 255 && subImg.at<unsigned char>(y, x) == 255)
			{
				if(horLinesImg.at<unsigned char>(y + 1, x) == 255)
				{
					newSubImg.at<unsigned char>(y, x) = 0;
				}
				else if(horLinesImg.at<unsigned char>(y + 2, x) == 255)
				{
					newSubImg.at<unsigned char>(y, x) = 0;
					newSubImg.at<unsigned char>(y + 1, x) = 0;
				}
			}
		}
	}
	return newSubImg;
}
