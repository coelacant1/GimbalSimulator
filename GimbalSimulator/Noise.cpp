#include "Noise.h"

Noise::Noise(int seed) {
	generator.seed(seed);
	srand(seed);
}

double Noise::GetRandNormal(double range) {
	std::normal_distribution<double> distribution(-range / 2.0, range / 2.0);

	return distribution(generator);
}

double Noise::GetRandUniform(double range) {
	return ((double)rand() / 32767.0) * range - (range / 2.0);
}

Vector3D Noise::RandomizeNormal(double scale) {
	double rand1 = scale * GetRandNormal(1.0);
	double rand2 = scale * GetRandNormal(1.0);
	double rand3 = scale * GetRandNormal(1.0);

	//rand1 = GetRandNormal(1) > 0.45 ? rand1 + GetRandNormal(scale / 4.0) : rand1;
	//rand2 = GetRandNormal(1) > 0.45 ? rand2 + GetRandNormal(scale / 4.0) : rand2;
	//rand3 = GetRandNormal(1) > 0.45 ? rand3 + GetRandNormal(scale / 4.0) : rand3;

	return Vector3D(rand1, rand2, rand3);
}

Vector3D Noise::RandomizeUniform(double scale) {
	double rand1 = scale * GetRandUniform(1.0);
	double rand2 = scale * GetRandUniform(1.0);
	double rand3 = scale * GetRandUniform(1.0);

	//rand1 = GetRandUniform(1) > 0.45 ? rand1 + GetRandUniform(scale / 4.0) : rand1;
	//rand2 = GetRandUniform(1) > 0.45 ? rand2 + GetRandUniform(scale / 4.0) : rand2;
	//rand3 = GetRandUniform(1) > 0.45 ? rand3 + GetRandUniform(scale / 4.0) : rand3;

	return Vector3D(rand1, rand2, rand3);
}