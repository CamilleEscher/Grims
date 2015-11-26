#ifndef BIVECTOR_HPP
#define BIVECTOR_HPP
#include <vector>

class Bivector
{
	std::vector<int>	m_left;
	std::vector<int>	m_right;

public : 
	std::vector<int> const&	getLeft() const;
	std::vector<int> const&	getRight() const;
	void					setLeft(std::vector<int> const& left);
	void					setRight(std::vector<int> const& right);
};

#endif
