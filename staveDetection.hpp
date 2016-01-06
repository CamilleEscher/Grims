#ifndef STAVE_DETECTION_HPP
#define STAVE_DETECTION_HPP
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include "Bivector.hpp"

/*!
  	\brief
	To find the angle of the slope of the staves, we use the correlation beetwen the half left part and right part of the score (the best vertical shift (h) of one of them enables us to find hMax and then the angle)

	\param binaryImg binarized image of the page of score
*/
int						correlation(cv::Mat const& binaryImg);

/*!
  	\brief
	Correction of the slope according to hMax

	\param binaryImg binarized image of the page of score
*/
cv::Mat					correctSlope(cv::Mat const& binaryImg);

/*!
  	\brief
	According to the processed vertical profile of the image (where the maximums correspond to the lines of the staves) we process a new profile which maxima represents the middle line of every stave

	\param profileVect vector containing a number of elements equals to the number of row of the page of score. Each element corresponds to the number of black pixel (which belong to the lines of stave)
	\param interline average distance between 2 lines of stave
 */
std::vector<int>		getStavesProfileVect(std::vector<int> const& profileVect, int interline);

/*!
  	\brief
	Find every index of rows of the middle line of all the staves in the whole score

	\param profileVect see getStavesProfileVect param
	\param interline see getStavesProfileVect param
*/
std::vector<int>		detectMiddleLineAbsc(std::vector<int> const& profileVect, int interline);

/*!
  	\brief
	Adapt the method of detectMiddleLineAbsc to adjust the index of row of the middle line of stave according the vertical profile of just one stave (profileVect here is a part of the previous considered profileVect)

	\param profileVect vector containing a number of elements equals to the number of row of the i-th stave of the score. Each element corresponds to the number of black pixel (which belong to the lines of stave)
	\param interline see getStavesProfileVect param
*/
int						detectMiddleLineAbscInSub(std::vector<int> const& profileVect, int interline);

/*!
  	\brief
	To determine the average thickness of the lines in all the staves, we process the histogram of the thickness of every lines

	\param middleLineAbscs vector of abscissa of the third lines of stave in every stave of the image
	\param heightSize heigth of the page of score
	\param binaryImg binarized page of score
*/
std::vector<int>		getLineThicknessHistogram(std::vector<int> const& middleLineAbscs, int heightSize, cv::Mat const& binaryImg);

/*!
  	\brief
	maxHisto represents 'thickness0', the most represented value in the histogram of the thicknesses; this functions evaluate 'thicknessAvg' by getting an average of the 3 columns in the histogram that are around thickness0

	\param histogram histogram of the vertical thicknesses in the page of score
	\param maxHisto maximum value of the histogram
*/
double					getLineThickness(std::vector<int> const& histogram, unsigned int maxHisto);

/*!
  	\brief
	Extraction the sub images of every staves in the whole score

	\param binaryImg see getLineThicknessHistogram
	\param middleLineAbscs see getLineThicknessHistogram
	\param interline see getStavesProfileVect
*/
std::vector<cv::Mat>	extractSubImages(cv::Mat const& binaryImg, std::vector<int> const& middleLineAbscs, int interline);

/*!
  	\brief
	Process of the left and right ordinates of every staves in every sub images with an horizontal profile of every sub images thresholded in the lest and right part ords is an instance of a class Bivector which contains 2 vectors : the vector of the left ordinates of the sub images and the vector of the right ones

	\param subImg see getLineThicknessHistogram
	\param thicknessAvg average vertical thickness of the lines of stave
	\param thickness0 most represented value in the histogram of vertical thicknesses
	\param interline see getStavesProfileVect
	\param middleLineAbscs see getLineThicknessHistogram
*/
Bivector				getOrdsPosition(std::vector<cv::Mat> const& subImg, double thicknessAvg, int thickness0, int interline, std::vector<int> const& middleLineAbscs);

/*!
  	\brief
	Processes the 'algorithme de poursuite des portées' => tracking of staves algorithm in the thesis : it calculates the better abscissa (row) of the middle line of a stave at every column

	\param middleLineAbsc see getMaxDeltaOrdProfile
	\param interline see getStavesProfileVect
	\param thickness0 see getMaxDeltaOrdProfile
	\param subImgI see subImg in getMaxDeltaOrdProfile
	\param leftOrd first detected ordinate of the stave
	\param rightOrd last detected oridnate of the stave
*/
std::vector<int>		getMiddleLineAbsc(int middleLineAbsc, int interline, int thickness0, cv::Mat subImgI, int leftOrd, int rightOrd);

#endif
