#pragma once

#include "Average.h"
#include "Vector.h"

class VectorAverage {
private:
	Average X;
	Average Y;
	Average Z;

public:
	VectorAverage();

	void Add(Vector3D input);
	Vector3D CalculateAverage();
	Vector3D CalculateStdDev();

};