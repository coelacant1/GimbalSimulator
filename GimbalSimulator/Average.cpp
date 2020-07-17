#include "Average.h"

Average::Average() {}

void Average::Add(double value) {
	values.push_back(value);
}

double Average::Calculate() {
	int count = 0;
	double sum = 0;
	double avg = 0.0;

	for (std::vector <double>::iterator i = values.begin(); i != values.end(); ++i) {
		sum += *i;
		count++;
	}

	if (count > 0) {
		avg = sum / count;
	}

	return avg;
}
