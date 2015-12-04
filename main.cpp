#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <set>
#include <cmath>
#include "tools.hpp"
#include "Staves.hpp"
#include "staveDetection.hpp"

static std::string const OPTION_PRINT = "print";
static std::string const OPTION_RESIZE = "resize";

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
		score = cv::imread(fileName, CV_LOAD_IMAGE_GRAYSCALE);

		if(score.empty())
		{
			std::cout << "'" << fileName << "' can't be found" << std::endl;
			return -1;
		}
		else
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
		}
	}
	return 0;
}
