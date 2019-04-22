#ifndef PARAMS_IN_POLYGON_H
#define PARAMS_IN_POLYGON_H

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "point_in_polygon.h"

class params_in_polygon final
{
public:
	params_in_polygon() {}
	params_in_polygon(const std::string &jsson_file) { load(jsson_file); }
	bool load(const std::string &jsson_file);
	boost::property_tree::ptree get_params_of(const float x, const float y);
private:
	boost::property_tree::ptree pt_;
	std::vector<point_in_polygon> pips_;
	std::vector<boost::property_tree::ptree> params_pt_;
};

#endif //PARAMS_IN_POLYGON_H