#include "point_in_polygon.h"
#include <cassert>

void point_in_polygon::set_polygon(std::vector<float>& x, std::vector<float>& y)
{
	assert(x.size() == y.size() && x.size() > 0 && y.size() > 0);
	poly_x_ = x; poly_y_ = y;
	int   i, j = x.size() - 1;
	min_x_ = max_x_ = x[0];
	min_y_ = max_y_ = y[0];
	constant_.resize(x.size());
	multiple_.resize(x.size());
	for (i = 0; i < x.size(); i++) {
		if (x[i] > max_x_) max_x_ = x[i];
		else if (x[i] < min_x_) min_x_ = x[i];

		if (y[i] > max_y_) max_y_ = y[i];
		else if (y[i] < min_y_) min_x_ = y[i];

		if (y[j] == y[i]) {
			constant_[i] = x[i];
			multiple_[i] = 0;
		}
		else {
			constant_[i] = x[i] - (y[i] * x[j]) / (y[j] - y[i]) + (y[i] * x[i]) / (y[j] - y[i]);
			multiple_[i] = (x[j] - x[i]) / (y[j] - y[i]);
		}
		j = i;
	}
}

bool point_in_polygon::is_in_polygon(const float x, const float y)
{
	if (x < min_x_ || x > max_x_ || y < min_y_ || y> max_y_) return false;
	bool oddNodes = false, current = poly_y_.back() > y, previous;
	for (int i = 0; i < poly_y_.size(); i++) {
		previous = current; current = poly_y_[i] > y;
		if (current != previous) oddNodes ^= y * multiple_[i] + constant_[i] < x;
	}
	return oddNodes;
}
