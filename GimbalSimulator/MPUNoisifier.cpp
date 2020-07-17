#include "MPUNoisifier.h"

MPUNoisifier::MPUNoisifier(Noise noise) {
	mpuOS = MPUOS(
		noise.RandomizeUniform(1),
		noise.RandomizeUniform(1),
		Vector3D(1, 1, 1).Add(noise.RandomizeUniform(1)),
		Vector3D(1, 1, 1).Add(noise.RandomizeUniform(1))
	);
}
