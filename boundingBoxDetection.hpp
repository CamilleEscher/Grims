#ifndef BOUNDING_BOX_DETECTION_HPP
#define BOUNDING_BOX_DETECTION_HPP
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include "Staves.hpp"

/*!
  \brief
  Gather the sub images that contains paired staves (linked by curly brackets)

  \param staves vector of Stave
*/
void					gatherImages(std::vector<Stave> const& staves);

/*! 
  \brief
  Display the significant vertical segments of every stave (the brightest represent the longest)

  \param staves vector of Stave
*/
std::vector<cv::Mat>	highLightVerticals(std::vector<Stave> const& staves);

/*!
  \brief
  use the hough transform to detect the circles in sub images
*/
void					detectCircles(std::vector<Stave> const& staves, int interline);

/*!
 \brief
 erode the sub images according to a circular kernel which size is proportional to the interline
*/
void					erodeWithEllipseElement(std::vector<Stave> const& staves, int interline);

#endif
