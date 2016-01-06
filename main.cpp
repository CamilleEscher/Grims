#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <set>
#include <cmath>
#include <string>
#include "tools.hpp"
#include "Staves.hpp"
#include "staveDetection.hpp"
#include "boundingBoxDetection.hpp"
#include <stdexcept>

static std::string const	OPTION_PRINT = "printLines";
static std::string const	OPTION_RESIZE = "resize";
static std::string const	OPTION_ERASE = "eraseLines";
static std::string const	OPTION_GATHER = "gatherStaves";
static std::string const	OPTION_VERTICALLINES = "printVerticalLines";
static std::string const	OPTION_CIRCLES = "printCircles";

std::set<std::string>	makeArgumentSet(int argc, char* argv[])
{
	std::set<std::string>	arguments;

	for(int i = 2; i < argc; ++i)
	{
		arguments.insert(argv[i]);
	}
	return arguments;
}

bool	isInSet(std::set<std::string> const& arguments, std::string const& argument)
{
	return (arguments.find(argument) != arguments.end());
}

int main(int argc, char* argv[])
{
	std::string				fileName;
	Staves					staves;
	cv::Mat					score;
	std::set<std::string>	arguments = makeArgumentSet(argc, argv);

	if(argc > 1)
	{
		fileName = argv[1];
		score = cv::imread(fileName, cv::IMREAD_GRAYSCALE);

		if(score.empty())
		{
			std::cout << "'" << fileName << "' can't be found" << std::endl;
			return -1;
		}
		else
		{
			try
			{
				if(isInSet(arguments, OPTION_RESIZE))
				{
					cv::resize(score, score, cv::Size(score.cols / 2, score.rows / 2));
				}
				staves.setup(score);
				if(isInSet(arguments, OPTION_PRINT))
				{
					staves.print();
				}
				if(isInSet(arguments, OPTION_ERASE))
				{
					staves.erase();
				}
				if(isInSet(arguments, OPTION_GATHER))
				{
					gatherImages(staves.getStaves());
				}
				if(isInSet(arguments, OPTION_VERTICALLINES))
				{
					std::vector<cv::Mat>	verticalLines = highLightVerticals(staves.getStaves());
				}
				if(isInSet(arguments, OPTION_CIRCLES))
				{
					erodeWithEllipseElement(staves.getStaves(), staves.getInterline());
					detectCircles(staves.getStaves(), staves.getInterline());
				}
			}
			catch(std::exception &e)
			{
				std::cout << e.what() << std::endl;
			}
		}
	}
	return 0;
}
