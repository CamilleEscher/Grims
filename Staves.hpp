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

class StaveLine
{
	unsigned int		m_id;
	std::vector<int>	m_absCoords;

public :
	explicit				StaveLine(unsigned int id);
	std::vector<int> const&	getAbsCoords() const;
	unsigned int			getId() const;
	void					setup(std::vector<int> const& centerLineAbsc, int interline);
};

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
	void							setup(cv::Mat subImg, int leftOrd, int rightOrd, std::vector<int> centerLineAbs, int interline);
	void							setStaveImg(cv::Mat const& img);
};

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
	void						setup(cv::Mat const& score);
	void						print() const;
	void						erase();
};

#endif
