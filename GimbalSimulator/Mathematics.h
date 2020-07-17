#pragma once

#include <iomanip>
#include <iostream>
#include <limits>
#include <cmath>
#include <sstream>
#include <stdarg.h>
#include <string>
#include <vector>

class Mathematics {
public:
	static double PI;

	static double Constrain(double value, double minimum, double maximum);
	static double DegreesToRadians(double degrees);
	static double RadiansToDegrees(double radians);
	static double CosineInterpolate(double start, double end, double ratio);
	static std::string DoubleToCleanString(double value);
	static void CleanPrint(int values, ...);
	static bool IsNaN(double);
	static bool IsInfinite(double);
	static bool IsFinite(double);
	static int Sign(double);

};
