#include "Staves.hpp"
#include "Bivector.hpp"

// StaveLine implementation
StaveLine::StaveLine(unsigned int id) :
	m_id(id)
{

}

std::vector<int> const&	StaveLine::getAbsCoords() const
{
	return m_absCoords;
}

unsigned int	StaveLine::getId() const
{
	return m_id;
}

void	StaveLine::setup(std::vector<int> const&	centerLineAbsc, int interline)
{
	int	shift = (interline) * (m_id - 2);
	int	vectorSize = centerLineAbsc.size();
	m_absCoords.reserve(centerLineAbsc.size());
	for(int i = 0; i < vectorSize; ++i)
	{
		m_absCoords.push_back(centerLineAbsc.at(i) + shift);
	}
}

// Stave implementation
Stave::Stave(unsigned int id) :
	m_id(id)
{
	
}

std::vector<StaveLine> const&	Stave::getStaveLines() const
{
	return m_staveLines;
}

cv::Mat	Stave::getStaveImg() const
{
	return m_staveImg;
}

int		Stave::getId() const
{
	return m_id;
}

int	Stave::getLeftOrd() const
{
	return m_leftOrd;
}

int	Stave::getRightOrd() const
{
	return m_rightOrd;
}

// Staves implementation

std::vector<Stave> const&	Staves::getStaves() const
{
	return m_staves;
}

unsigned int	Staves::getStavesNb() const
{
	return m_stavesNb;
}

int		Staves::getInterline() const
{
	return m_interline;
}

double	Staves::getThicknessMoy() const
{
	return m_thicknessMoy;
}

int		Staves::getThickness0() const
{
	return m_thickness0;
}

cv::Mat const&	Staves::getScore() const
{
	return m_score;
}
void	Stave::setup(cv::Mat subImg, int leftOrd, int rightOrd, std::vector<int> centerLineAbs, int interline)
{
	std::vector<StaveLine>	staveLines;
	unsigned int			staveLinesSize = 5;

	m_staveImg = subImg.clone();
	m_leftOrd = leftOrd;
	m_rightOrd = rightOrd;
	staveLines.reserve(staveLinesSize);

	for(unsigned int i = 0; i < staveLinesSize; ++i)
	{
		StaveLine staveLine(i);	
		staveLine.setup(centerLineAbs, interline);
		staveLines.push_back(staveLine);
	}
	m_staveLines = staveLines;
}

void	Staves::setup(cv::Mat const& score)
{
	std::vector<int>			profilVect;
	std::vector<int>			centerLinePositions;
	std::vector<int>			newCenterLinePositions;
	std::vector<int>			lineThicknessHistogram;
	std::vector<int>			centerLineAbsc;
	std::vector<int> 			leftOrds;
	std::vector<int> 			rightOrds; 
	std::vector<cv::Mat>		subImg;
	Bivector					ords;

	m_score = correctSlope(binarize(score.clone(), 220));
	profilVect = getHorizontalProfil(m_score);
	m_interline = findInterline(profilVect);
	centerLinePositions = detectCenterLinePos(profilVect, m_interline);
	m_stavesNb = static_cast<unsigned int>(centerLinePositions.size());
	lineThicknessHistogram = getLineThicknessHistogram(centerLinePositions, static_cast<int>(6 * m_interline), m_score);
	m_thickness0 = getMaxIndex(lineThicknessHistogram);
	m_thicknessMoy = getLineThickness(lineThicknessHistogram, m_thickness0);
	subImg = extractSubImages(m_score, centerLinePositions, m_interline);
	for(int i = 0; i < static_cast<int>(subImg.size()); ++i)
	{
		profilVect = getHorizontalProfil(subImg.at(i));
		centerLinePositions.at(i) = (detectCenterLinePos(profilVect, m_interline).at(0));
	}
	ords = getOrdsPosition(subImg, m_thicknessMoy, m_thickness0, m_interline, centerLinePositions);
	m_staves.reserve(m_stavesNb); 
	leftOrds = ords.getLeft();
	rightOrds = ords.getRight();
	for(unsigned int i = 0; i < m_stavesNb; ++i)
	{
		Stave stave(i);
		centerLineAbsc = getCenterLineAbsc(centerLinePositions.at(i), m_interline, m_thickness0, subImg.at(i), leftOrds.at(i), rightOrds.at(i));
		stave.setup(subImg.at(i), leftOrds.at(i), rightOrds.at(i), centerLineAbsc, m_interline);
		m_staves.push_back(stave);
		//cv::imshow(std::to_string(i), subImg.at(i));
		//cv::waitKey(0);
	}
}

void	Staves::print() const
{
	cv::Vec3b	blue = {255, 0, 0};

	for(unsigned int stave_id = 0; stave_id < m_stavesNb; ++stave_id)
	{
		Stave stave = m_staves.at(stave_id);
		cv::Mat	subImg;
		cvtColor(stave.getStaveImg(), subImg, CV_GRAY2RGB);
		int left = stave.getLeftOrd();
		int right = stave.getRightOrd();
		for(unsigned int staveLine_id = 0; staveLine_id < 5; ++staveLine_id)
		{
			std::vector<int>	abs = stave.getStaveLines().at(staveLine_id).getAbsCoords();
			for(int y = left; y <= right; ++y)
			{
				subImg.at<cv::Vec3b>(cv::Point(y, abs.at(y - left))) = blue;
			}
		}
		cv::imshow(std::to_string(stave_id), subImg);
		cv::waitKey(0);
	}
}
