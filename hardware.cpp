#include "Arduino.h"
#include "hardware.h"
#include "units.h"
#include "hardware_definition.h"


/**
	 * \brief Print a string to serial as is.
	 * \param string is a null terminated string.
	 * \return total number of bytes written.
	 */
template <typename tag>
auto hardware::serial_api<tag>::print(char const* string) -> char_count {
	char_count NumBytes = Serial.print(string);
	return NumBytes;
}

/*
 * \brief Print a character to serial as is.
 * \param c is the character to print.
 * \return total number of bytes written.
 */
template <typename tag>
auto hardware::serial_api<tag>::print(char c) -> char_count {
	char_count NumBytes = Serial.print(c);
	return NumBytes; 
}

/**
	 * \brief Print an int to serial as human readable text.
	 * \param i is the value to be printed.
	 * \param base is the number base to use. Can be 2,6,8,10.
	 * \return total number of bytes written.
	 
template <serial_tag id>
static auto hardware::serial_tag<id>::print(int i, int base = 10)->char_count {
	if (base == 2) {
		size_t NumBytes = Serial.print(i, BIN);
		return NumBytes;
	} else if(base == 6) {
		size_t NumBytes = Serial.print(i, HEX);
		return NumBytes;
	} else if(base == 8) {
		size_t NumBytes = Serial.print(i, OCT);
		return NumBytes;
	} else if(base == 10) {
		size_t NumBytes = Serial.print(i, DEC);
		return NumBytes;
	}
	else {
		Serial.print("Unrecognised input.  Base must be 2,6,8,10");
	}
}

/**
	 * \brief Print an unsigned int to serial as human readable text.
	 * \param i is the value to be printed.
	 * \param base is the number base to use. Can be 2,6,8,10.
	 * \return total number of bytes written.
	 
template <serial_tag id>
static auto hardware::serial_tag<id>::print(unsigned int i, int base = 10)->char_count {
	//check if unsigned int i>=0
	if (i < 0) {
		Serial.print("Incorrect input. Unsigned Int must be greater than or equal 0");
	}
	else {
		if (base == 2) {
			size_t NumBytes = Serial.print(i, BIN);
			return NumBytes;
		}
		else if (base == 6) {
			size_t NumBytes = Serial.print(i, HEX);
			return NumBytes;
		}
		else if (base == 8) {
			size_t NumBytes = Serial.print(i, OCT);
			return NumBytes;
		}
		else if (base == 10) {
			size_t NumBytes = Serial.print(i, DEC);
			return NumBytes;
		}
		else {
			Serial.print("Unrecognised input.  Base must be 2,6,8,10");
		}
	}
}


/**
 * \brief Print a signed long int to serial as human readable text.
 * \param i is the value to be printed.
 * \param base is the number base to use. Can be 2,6,8,10.
 * \return total number of bytes written.
 
template <serial_tag id>
static auto hardware::serial_tag<id>::print(long i, int base = 10)->char_count {
	if (base == 2) {
		size_t NumBytes = Serial.print(i, BIN);
		return NumBytes;
	}
	else if (base == 6) {
		size_t NumBytes = Serial.print(i, HEX);
		return NumBytes;
	}
	else if (base == 8) {
		size_t NumBytes = Serial.print(i, OCT);
		return NumBytes;
	}
	else if (base == 10) {
		size_t NumBytes = Serial.print(i, DEC);
		return NumBytes;
	}
	else {
		Serial.print("Unrecognised input.  Base must be 2,6,8,10");
	}
}

/**
 * \brief Print an unsigned long int to serial as human readable text.
 * \param i is the value to be printed.
 * \param base is the number base to use. Can be 2,6,8,10.
 * \return total number of bytes written.
 
template <serial_tag id>
static auto hardware::serial_tag<id>::print(unsigned long i, int base = 10)->char_count {
	//check if unsigned long i>=0
	if (i < 0) {
		Serial.print("Incorrect input. Unsigned long must be greater than or equal 0");
	}
	else {
		if (base == 2) {
			size_t NumBytes = Serial.print(i, BIN);
			return NumBytes;
		}
		else if (base == 6) {
			size_t NumBytes = Serial.print(i, HEX);
			return NumBytes;
		}
		else if (base == 8) {
			size_t NumBytes = Serial.print(i, OCT);
			return NumBytes;
		}
		else if (base == 10) {
			size_t NumBytes = Serial.print(i, DEC);
			return NumBytes;
		}
		else {
			Serial.print("Unrecognised input.  Base must be 2,6,8,10");
		}
	}
}

/**
 * \brief Print an unsigned char to serial as human readable text.
 * \param c is the value to be printed.
 * \param base is the number base to use. Can be 2,6,8,10.
 * \return total number of bytes written.
 
template <serial_tag id>
static auto hardware::serial_tag<id>::print(unsigned char c, int base = 10)->char_count {
	if (base == 2) {
		size_t NumBytes = Serial.print((uint8_t)c, BIN);
		return NumBytes;
	}
	else if (base == 6) {
		size_t NumBytes = Serial.print((uint8_t)c, HEX);
		return NumBytes;
	}
	else if (base == 8) {
		size_t NumBytes = Serial.print((uint8_t)c, OCT);
		return NumBytes;
	}
	else if (base == 10) {
		size_t NumBytes = Serial.print((uint8_t)c, DEC);
		return NumBytes;
	}
	else {
		Serial.print("Unrecognised input.  Base must be 2,6,8,10");
	}
}

/**
 * \brief Print a double to serial as human readable text.
 * \param i is the value to be printed.
 * \param base number of decimal place to print to. //meaning 2dp
 * \return total number of bytes written.
 
template <serial_tag id>
static auto hardware::serial_tag<id>::print(double i, int base = 2)->char_count {
	precision = pow(10, base);
	Serial.print(int(i));  //print int part
	Serial.print("."); // print decimal point
	unsigned int frac;
	if (i >= 0)
		frac = (i - int(i)) * precision;
	else
		frac = (int(i) - i) * precision;
	size_t NumBytes =  Serial.print(frac, DEC);
	return NumBytes;
}

/**
 * \brief Print a string to serial followed by return character.
 * \param string is a null terminated string.
 * \return total number of bytes written.
 
template <serial_tag id>
static auto hardware::serial_tag<id>::print_line(char const* string)->char_count {
	size_t NumBytes = Serial.println(string);
	return NumBytes;
}

/**
 * \brief Print a character to serial as is followed by return character.
 * \param c is the character to print.
 * \return total number of bytes written.
 
template <serial_tag id>
static auto hardware::serial_tag<id>::print_line(char c)->char_count {
	size_t NumBytes = Serial.print(c);
	return NumBytes;
}

/**
 * \brief Print a signed int to serial as human readable text followed by
 * return character.
 * \param i is the value to be printed.
 * \param base is the number base to use. Can be 2,6,8,10.
 * \return total number of bytes written.
 
template <serial_tag id>
static auto hardware::serial_tag<id>::print_line(int i, int base = 10)->char_count {
	if (base == 2) {
		size_t NumBytes = Serial.println(i, BIN);
		return NumBytes;
	}
	else if (base == 6) {
		size_t NumBytes = Serial.println(i, HEX);
		return NumBytes;
	}
	else if (base == 8) {
		size_t NumBytes = Serial.println(i, OCT);
		return NumBytes;
	}
	else if (base == 10) {
		size_t NumBytes = Serial.println(i, DEC);
		return NumBytes;
	}
	else {
		Serial.print("Unrecognised input.  Base must be 2,6,8,10");
	}
}

/**
 * \brief Print an unsigned int to serial as human readable text followed by
 * return character.
 * \param i is the value to be printed.
 * \param base is the number base to use. Can be 2,6,8,10.
 * \return total number of bytes written.
 
template <serial_tag id>
static auto hardware::serial_tag<id>::print_line(unsigned int i, int base = 10)->char_count {
	//check if unsigned int i>=0
	if (i < 0) {
		Serial.print("Incorrect input. Unsigned Int must be greater than or equal 0");
	}
	else {
		if (base == 2) {
			size_t NumBytes = Serial.println(i, BIN);
			return NumBytes;
		}
		else if (base == 6) {
			size_t NumBytes = Serial.println(i, HEX);
			return NumBytes;
		}
		else if (base == 8) {
			size_t NumBytes = Serial.println(i, OCT);
			return NumBytes;
		}
		else if (base == 10) {
			size_t NumBytes = Serial.println(i, DEC);
			return NumBytes;
		}
		else {
			Serial.print("Unrecognised input.  Base must be 2,6,8,10");
		}
	}
}

/**
 * \brief Print a signed long int to serial as human readable text followed
 * by return character.
 * \param i is the value to be printed.
 * \param base is the number base to use. Can be 2,6,8,10.
 * \return total number of bytes written.
 
template <serial_tag id>
static auto hardware::serial_tag<id>::print_line(long i, int base = 10)->char_count {
	if (base == 2) {
		size_t NumBytes = Serial.println(i, BIN);
		return NumBytes;
	}
	else if (base == 6) {
		size_t NumBytes = Serial.println(i, HEX);
		return NumBytes;
	}
	else if (base == 8) {
		size_t NumBytes = Serial.println(i, OCT);
		return NumBytes;
	}
	else if (base == 10) {
		size_t NumBytes = Serial.println(i, DEC);
		return NumBytes;
	}
	else {
		Serial.print("Unrecognised input.  Base must be 2,6,8,10");
	}
}

/**
 * \brief Print an unsigned long int to serial as human readable
 * text followed by return character.
 * \param i is the value to be printed.
 * \param base is the number base to use. Can be 2,6,8,10.
 * \return total number of bytes written.
 
template <serial_tag id>
static auto hardware::serial_tag<id>::print_line(unsigned long i, int base = 10)->char_count {
	//check if unsigned long i>=0
	if (i < 0) {
		Serial.print("Incorrect input. Unsigned long must be greater than or equal 0");
	}
	else {
		if (base == 2) {
			size_t NumBytes = Serial.println(i, BIN);
			return NumBytes;
		}
		else if (base == 6) {
			size_t NumBytes = Serial.println(i, HEX);
			return NumBytes;
		}
		else if (base == 8) {
			size_t NumBytes = Serial.println(i, OCT);
			return NumBytes;
		}
		else if (base == 10) {
			size_t NumBytes = Serial.println(i, DEC);
			return NumBytes;
		}
		else {
			Serial.print("Unrecognised input.  Base must be 2,6,8,10");
		}
	}
}

/**
 * \brief Print an unsigned char to serial as human readable text followed
 * by return character.
 * \param c is the value to be printed.
 * \param base is the number base to use. Can be 2,6,8,10.
 * \return total number of bytes written.
 
template <serial_tag id>
static auto hardware::serial_tag<id>::print_line(unsigned char c, int base = 10)->char_count {
	if (base == 2) {
		size_t NumBytes = Serial.println((uint8_t)c, BIN);
		return NumBytes;
	}
	else if (base == 6) {
		size_t NumBytes = Serial.println((uint8_t)c, HEX);
		return NumBytes;
	}
	else if (base == 8) {
		size_t NumBytes = Serial.println((uint8_t)c, OCT);
		return NumBytes;
	}
	else if (base == 10) {
		size_t NumBytes = Serial.println((uint8_t)c, DEC);
		return NumBytes;
	}
	else {
		Serial.print("Unrecognised input.  Base must be 2,6,8,10");
	}
}

/**
 * \brief Print a double to serial as human readable text followed by return
 * character.
 * \param i is the value to be printed.
 * \param base number of decimal place to print to.
 * \return total number of bytes written.
 
template <serial_tag id>
static auto hardware::serial_tag<id>::print_line(double i, int base = 2)->char_count {
	precision = pow(10, base);
	Serial.print(int(i));  //print int part
	Serial.print("."); // print decimal point
	unsigned int frac;
	if (i >= 0)
		frac = (i - int(i)) * precision;
	else
		frac = (int(i) - i) * precision;
	size_t NumBytes = Serial.println(frac, DEC);
	return NumBytes;
}

*/

template class hardware::serial_api<hardware::serial_tag<1>>; //bluetooth
template class hardware::serial_api<hardware::serial_tag<0>>; //serial