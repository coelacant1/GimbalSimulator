#pragma once

#include "Mathematics.h"

class Average {
private:
	std::vector<double> values;

public:
	Average();
	void Add(double value);
	double Calculate();

};
