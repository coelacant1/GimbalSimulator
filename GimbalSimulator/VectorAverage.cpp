#include "VectorAverage.h"

VectorAverage::VectorAverage() {
	X = Average();
	Y = Average();
	Z = Average();
}

void VectorAverage::Add(Vector3D input) {
	X.Add(input.X);
	Y.Add(input.Y);
	Z.Add(input.Z);
}

Vector3D VectorAverage::CalculateAverage() {
	return Vector3D{
		X.CalculateAverage(),
		Y.CalculateAverage(),
		Z.CalculateAverage()
	};
}

Vector3D VectorAverage::CalculateStdDev() {
	return Vector3D{
		X.CalculateStdDev(),
		Y.CalculateStdDev(),
		Z.CalculateStdDev()
	};
}
