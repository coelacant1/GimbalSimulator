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

Vector3D VectorAverage::Calculate() {
	return Vector3D{
		X.Calculate(),
		Y.Calculate(),
		Z.Calculate()
	};
}
