#ifndef TO_STRING_H
#define TO_STRING_H

#include <string>

std::string str(int num, bool skip_trailing_zeros = false);
std::string str(float num, int max_digits);

#endif //TO_STRING_H
