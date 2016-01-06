#ifndef STAVES_HPP
#define STAVES_HPP
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include "tools.hpp"
#include "staveDetection.hpp"

/*!
  \class StaveLine
  \brief StaveLine stores the informations of a specific line of stave

*/
class StaveLine
{
	unsigned int		m_id;
	std::vector<int>	m_absCoords;

public :
	explicit				StaveLine(unsigned int id);
	std::vector<int> const&	getAbsCoords() const;
	unsigned int			getId() const;
	/*!
		set the fields of the StaveLine instances

		Called by the setup function of Stave for every element of m_staveLines (vector of StaveLines)
		\param middleLineAbscs the abscissa of the third line of the stave between the first and last ordinates of the stave
		\param interline average distance between 2 lines of stave
	 */
	void					setup(std::vector<int> const& middleLineAbscs, int interline);
};

/*!
	\class Stave
	\brief Stave stores the shared informations of a stave defined by an id

 */
class Stave
{
	unsigned int			m_id;
	std::vector<StaveLine>	m_staveLines;
	cv::Mat					m_staveImg;
	int						m_leftOrd = -1;
	int						m_rightOrd = -1;

public :
									Stave(unsigned int id);
	std::vector<StaveLine> const&	getStaveLines() const;
	cv::Mat	const&					getStaveImg() const;
	int								getId() const;
	int								getLeftOrd() const;
	int								getRightOrd() const;
	/*!
		set all the fields of the instance of Stave

		Called by the setup function of Staves
		\param subImg image of the i-th stave of the page of score
		\param leftOrd the ordinate of the beginning of the stave
		\param rightOrd the ordintate of the end of the stave
		\param middleLineAbsc the abscissa of the third line of the stave between the first and last ordinates of the stave
		\param interline the average distance between 2 lines of stave
	 */
	void							setup(cv::Mat subImg, int leftOrd, int rightOrd, std::vector<int> middleLineAbscs, int interline);
	void							setStaveImg(cv::Mat const& img);
};

/*!
  \class Staves
  \brief Staves stores all the needed informations relatives to the position of the staves in one page of score and their specificities

*/
class Staves
{
	unsigned int		m_stavesNb;
	std::vector<Stave>	m_staves;
	int					m_interline;
	double				m_thicknessAvg;
	int					m_thickness0;
	cv::Mat				m_score;

public :
	std::vector<Stave> const&	getStaves() const;
	unsigned int				getStavesNb() const;
	int							getInterline() const;
	double						getThicknessMoy() const;
	int							getThickness0() const;
	cv::Mat const&				getScore() const;
	/*!
		set all the fields of the instance of Staves

		Called by the main
		\param score image of one page of score in gray scale
	 */
	void						setup(cv::Mat const& score);
	/*!
		display every sub image of stave of the page with highlighted lines of stave

		Called by the argument "printLines" when executing the program 
	 */
	void						print() const;
	/*!
		display every sub image of stave of the page with the lines erased

		Called by the argument "eraseLines" when executing the program 
	 */
	void						erase();
};

#endif
