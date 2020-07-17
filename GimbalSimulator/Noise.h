#pragma once
#include <random>
#include "Vector.h"

class Noise {
private:
	std::default_random_engine generator;

public:
	Noise(int seed);

	double GetRandNormal(double range);

	double GetRandUniform(double range);
	Vector3D RandomizeNormal(double scale);
	Vector3D RandomizeUniform(double scale);
};
