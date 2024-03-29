#include "params_in_polygon.h"
#include <iostream>

void test_0()
{
	std::vector<float> poly_x = { 0, 0, 1, 1 };
	std::vector<float> poly_y = { 0, 1, 1, 0 };
	point_in_polygon pip(poly_x, poly_y);
	std::cout << pip.is_in_polygon(0.1, 0.5) << std::endl;
}


void test_1()
{
	std::vector<float> poly_x = { 0, 5, 10, 5 };
	std::vector<float> poly_y = { 0, 5, 5, 0 };
	point_in_polygon pip(poly_x, poly_y);
	std::cout << pip.is_in_polygon(0.1, 0.09) << std::endl;
}

void test_params_in_ptree()
{
	params_in_polygon params_ip;
	params_ip.load("F:/workspace/accumulation/zswlib/point_in_polygon/params_in_polygon.json");
	std::cout << params_ip.get_params_of<float>(307.097587, 267.144858, "valid_deg_ratio_small_space", 1.0) << std::endl;
	std::cout << params_ip.get_params_of<float>(307.097587, 267.144858, "valid_deg_ratio_big_space", 1.0) << std::endl;
	std::cout << params_ip.get_params_of<float>(307.097587, 267.144858, "min_reg_sum_ratio", 1.0) << std::endl;
}

int main(int argc, char* argv[])
{
	// test_0();
	// test_1();

	test_params_in_ptree();
	return 0;
}
