#include "Utility.h"

float linear_mapping(float input, float min_input, float max_input, float min_output, float max_output)
{
	return (input - min_input) * (max_output - min_output) / (max_input - min_input) + min_output;
};

void *operator new(size_t size) {
	return malloc(size);
}

void *operator new[](size_t size) {
	return malloc(size);
}

void operator delete(void * ptr) {
	free(ptr);
}

void operator delete[](void * ptr) {
	free(ptr);
}
