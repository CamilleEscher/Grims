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

void	StaveLine::setup(?)
{
	std::vector<int> absCoords;
	//?
	m_absCoords = absCoords;
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

int		Staves::getThicknessMoy() const
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

void	Stave::setup(cv::Mat subImg, int leftOrd, int rightOrd, std::vector<int> centerLineAbs)
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
		//staveLine.setup();
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
	std::vector<cv::Mat>		subImg;
	Bivector					ords;

	m_score = correctSlope(binarize(score.clone(), 220));
	profilVect = getHorizontalProfil(m_score);
	m_interline = findInterline(profilVect);
	centerLinePositions = detectCenterLinePos(profilVect, m_interline);
	m_stavesNb = static_cast<unsigned int>(centerLinePositions.size());
	lineThicknessHistogram = getLineThicknessHistogram(centerLinePositions, static_cast<int>(6 * m_interline), m_score);
	m_thicknessMoy = getMaxIndex(lineThicknessHistogram);
	m_thickness0 = getLineThickness(lineThicknessHistogram, m_thicknessMoy);
	subImg = extractSubImages(m_score, centerLinePositions, m_interline);
	newCenterLinePositions.reserve(static_cast<int>(subImg.size()));
	for(int i = 0; i < static_cast<int>(subImg.size()); ++i)
	{
		profilVect = getHorizontalProfil(subImg.at(i));
		centerLinePositions.push_back(detectCenterLinePos(profilVect, m_interline).at(0));
	}
	ords = getOrdsPosition(subImg, m_thicknessMoy, m_thickness0, m_interline, centerLinePositions);
	m_staves.reserve(m_stavesNb); 
	for(unsigned int i = 0; i < m_stavesNb; ++i)
	{
		Stave stave(i);
		centerLineAbs = getCenterLineAbsc(centerLinePositions.at(i), interline, thickness0, subImg.at(i), ords.getLeft().at(i), ords.getRight.at(i));
		stave.setup(subImg.at(i), ords.getLeft().at(i), ords.getRight().at(i), centerLineAbs);
		//cv::imshow(std::to_string(i), subImg.at(i));
		//cv::waitKey(0);
	}
}
