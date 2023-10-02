#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>
#include <math.h>

#define PI acos(-1)

int main()
{
	Eigen::Vector3f v(2.0f, 1.0f, 1.0f);
	Eigen::Matrix3f i,j;
	i << cos(PI / 4), -sin(PI / 4), 0, sin(PI / 4), cos(PI / 4), 0, 0, 0, 1;
	j << 1, 0, 1, 0, 1, 2, 0, 0, 1;
	std::cout << j * i * v << std::endl;
	return 0;
}