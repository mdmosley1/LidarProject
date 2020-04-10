#include "helper.h"
#include <cstdlib> // RAND_MAX

int GetRandInt(int max)
{
    return std::rand() / (RAND_MAX / max);
}
