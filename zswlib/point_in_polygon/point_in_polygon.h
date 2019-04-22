#ifndef POINT_IN_POLYGON_H
#define POINT_IN_POLYGON_H

#include <vector>

/**
 * \brief judge if point is in an arbitrary polygon.
 * ref: http://alienryderflex.com/polygon/
 */
class point_in_polygon final
{
public:
	point_in_polygon() {}

	point_in_polygon(std::vector<float> &x, std::vector<float> &y) { set_polygon(x, y); }

	void set_polygon(std::vector<float> &x, std::vector<float> &y);

	bool is_in_polygon(const float x, const float y);

private:
	float min_x_;
	float min_y_;
	float max_x_;
	float max_y_;

	std::vector<float> poly_x_;
	std::vector<float> poly_y_;
	std::vector<float> constant_;
	std::vector<float> multiple_;

};

#endif // POINT_IN_POLYGON