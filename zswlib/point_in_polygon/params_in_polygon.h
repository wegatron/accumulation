#ifndef PARAMS_IN_POLYGON_H
#define PARAMS_IN_POLYGON_H

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "point_in_polygon.h"

class params_in_polygon
{
public:
	params_in_polygon() {}
	params_in_polygon(const std::string &jsson_file) { load(jsson_file); }
	bool load(const std::string &jsson_file);
	
	template<typename TYPE>
	TYPE get_params_of(const float x, const float y, const std::string &key, const TYPE &default_value)
	{
		TYPE ret_val = default_value;
		for (int i = 0; i< pips_.size(); ++i)
		{
			if (pips_[i].is_in_polygon(x, y))
			{
				boost::optional<TYPE> val = params_pt_[i].get_optional<TYPE>(key);
				if (val)
				{
					ret_val = val.get();
				}
			}
		}
		return ret_val;
	}
private:
	boost::property_tree::ptree pt_;
	std::vector<point_in_polygon> pips_;
	std::vector<boost::property_tree::ptree> params_pt_;
};

#endif //PARAMS_IN_POLYGON_H