#include "MPUNoisifier.h"

MPUNoisifier::MPUNoisifier(Noise noise) {
	mpuOS = MPUOS(
		noise.RandomizeUniform(0.25),//1 simulated
		noise.RandomizeUniform(0.25),//1 simulated
		Vector3D(1, 1, 1).Add(noise.RandomizeUniform(0.25)),//1 simulated
		Vector3D(1, 1, 1).Add(noise.RandomizeUniform(0.25))//1 simulated
	);
}
