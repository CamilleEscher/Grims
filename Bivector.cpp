#include "Bivector.hpp"

std::vector<int> const&	Bivector::getLeft() const
{
	return m_left;
}

std::vector<int> const&	Bivector::getRight() const
{
	return m_right;
}

void	Bivector::setLeft(std::vector<int> const& left)
{
	m_left = left;
}

void	Bivector::setRight(std::vector<int> const& right)
{
	m_right = right;
}
