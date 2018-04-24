#include <cmath>
#include <string>
#include <ToString.h>

std::string str(int num, bool skip_trailing_zeros) {
	bool negative = num < 0, stop = false;
	int trailing_zeros = 0;
	std::string out;

	if(negative) num = std::abs(num);
	if(num == 0) out += "0";
	while (num > 0) {
		char ascii = char(num % 10) + '0';
		out.insert(0, 1, ascii);
		num /= 10;

		if(!stop && ascii == '0') trailing_zeros++;
		else stop = true;
	}
	if(negative) out.insert(0, 1, '-');

	if(skip_trailing_zeros) return out.substr(0, out.size()-trailing_zeros);
	else return out;
}

std::string str(float num, int max_digits) {
	float int_part;
	float fract_part = std::abs(std::modf(num, &int_part));
	std::string int_str = str(int(int_part));

	if(max_digits > 6) max_digits = 6;

	std::string fract_str = ".";
	int dig;
	do {
		float fp = fract_part * 10;
		dig = int(std::round(fp));
		if(dig == 0) {
			fract_str += '0';
			fract_part = fp;
			max_digits--;
		}
	} while(max_digits > 0 && dig == 0);

	if(max_digits > 0) {
		fract_part = fract_part * float(std::pow(10,max_digits));
		return int_str + fract_str + str(int(fract_part), true);
	} else {
		return int_str;
	}
}
