#include "params_in_polygon.h"
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <boost/foreach.hpp>

bool params_in_polygon::load(const std::string& jsson_file)
{
	// check file exist
	std::cout << "loading config file params_in_polygon." << std::endl;
	if (!boost::filesystem::exists(jsson_file)) {
		std::cout << "config file params_in_polygon, not used." << std::endl;
		return false;
	}
	boost::property_tree::read_json(jsson_file, pt_);
	std::cout << "config file params_in_polygon, loaded." << std::endl;
	BOOST_FOREACH(boost::property_tree::ptree::value_type &child_tree, pt_.get_child("params_in_polygon"))
	{
		boost::property_tree::ptree &cur_params_pt = child_tree.second;
		params_pt_.emplace_back(cur_params_pt);
		
		std::vector<float> poly_x, poly_y;
		for (auto& item : cur_params_pt.get_child("coord_x"))
			poly_x.push_back(item.second.get_value <float> ());
		for (auto& item : cur_params_pt.get_child("coord_y"))
			poly_y.push_back(item.second.get_value <float>());
		pips_.emplace_back(poly_x, poly_y);
	}
}
