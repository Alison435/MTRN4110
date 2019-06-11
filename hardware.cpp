#include "Arduino.h"
#include "hardware.h"
#include "units.h"
#include "hardware_definition.h"
#include "Wire.h"
#include "external_DFRobot_LCD.h"

/* QUESTIONS FOR TUTOR!!!!!!!!!!!!

 //QUESTION TO TUTOR FOR ALL PORTS OR JUST 1 ? SERIAL SERIAL1 SERIAL2 SERIAL3

 //How to like LCD



*/
////////////////////
//SERIAL & BLUETOOTH
////////////////////

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
*/
template <typename tag>
auto hardware::serial_api<tag>::print(int i, int base = 10) -> char_count {
	if (base == 2) {
		char_count NumBytes = Serial.print(i, BIN);
		return NumBytes;
	} else if(base == 6) {
		char_count NumBytes = Serial.print(i, HEX);
		return NumBytes;
	} else if(base == 8) {
		char_count NumBytes = Serial.print(i, OCT);
		return NumBytes;
	} else if(base == 10) {
		char_count NumBytes = Serial.print(i, DEC);
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
*/
template <typename tag>
auto hardware::serial_api<tag>::print(unsigned int i, int base = 10) -> char_count {
	if (base == 2) {
		char_count NumBytes = Serial.print(i, BIN);
		return NumBytes;
	}
	else if (base == 6) {
		char_count NumBytes = Serial.print(i, HEX);
		return NumBytes;
	}
	else if (base == 8) {
		char_count NumBytes = Serial.print(i, OCT);
		return NumBytes;
	}
	else if (base == 10) {
		char_count NumBytes = Serial.print(i, DEC);
		return NumBytes;
	}
	else {
		Serial.print("Unrecognised input.  Base must be 2,6,8,10");
	}
}


/**
 * \brief Print a signed long int to serial as human readable text.
 * \param i is the value to be printed.
 * \param base is the number base to use. Can be 2,6,8,10.
 * \return total number of bytes written.
 */
template <typename tag>
auto hardware::serial_api<tag>::print(long i, int base = 10) -> char_count {
	if (base == 2) {
		char_count NumBytes = Serial.print(i, BIN);
		return NumBytes;
	}
	else if (base == 6) {
		char_count NumBytes = Serial.print(i, HEX);
		return NumBytes;
	}
	else if (base == 8) {
		char_count NumBytes = Serial.print(i, OCT);
		return NumBytes;
	}
	else if (base == 10) {
		char_count NumBytes = Serial.print(i, DEC);
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
 */
template <typename tag>
auto hardware::serial_api<tag>::print(unsigned long i, int base = 10) -> char_count {
	if (base == 2) {
		char_count NumBytes = Serial.print(i, BIN);
		return NumBytes;
	}
	else if (base == 6) {
		char_count NumBytes = Serial.print(i, HEX);
		return NumBytes;
	}
	else if (base == 8) {
		char_count NumBytes = Serial.print(i, OCT);
		return NumBytes;
	}
	else if (base == 10) {
		char_count NumBytes = Serial.print(i, DEC);
		return NumBytes;
	}
	else {
		Serial.print("Unrecognised input.  Base must be 2,6,8,10");
	}
}

/**
 * \brief Print an unsigned char to serial as human readable text.
 * \param c is the value to be printed.
 * \param base is the number base to use. Can be 2,6,8,10.
 * \return total number of bytes written.
 */
template <typename tag>
auto hardware::serial_api<tag>::print(unsigned char c, int base = 10) -> char_count {
	if (base == 2) {
		char_count NumBytes = Serial.print((uint8_t)c, BIN);
		return NumBytes;
	}
	else if (base == 6) {
		char_count NumBytes = Serial.print((uint8_t)c, HEX);
		return NumBytes;
	}
	else if (base == 8) {
		char_count NumBytes = Serial.print((uint8_t)c, OCT);
		return NumBytes;
	}
	else if (base == 10) {
		char_count NumBytes = Serial.print((uint8_t)c, DEC);
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
 */
template <typename tag>
auto hardware::serial_api<tag>::print(double i, int base = 2) -> char_count {
	double precision = pow(10, base);
	Serial.print(int(i));  //print int part
	Serial.print("."); // print decimal point
	unsigned int frac;
	if (i >= 0)
		frac = (i - int(i)) * precision;
	else
		frac = (int(i) - i) * precision;
	char_count NumBytes =  Serial.print(frac, DEC);
	return NumBytes;
}

/**
 * \brief Print a string to serial followed by return character.
 * \param string is a null terminated string.
 * \return total number of bytes written.
 */
template <typename tag>
auto hardware::serial_api<tag>::print_line(char const* string) -> char_count {
	char_count NumBytes = Serial.println(string);
	return NumBytes;
}

/**
 * \brief Print a character to serial as is followed by return character.
 * \param c is the character to print.
 * \return total number of bytes written.
 */
template <typename tag>
auto hardware::serial_api<tag>::print_line(char c) -> char_count {
	char_count NumBytes = Serial.print(c);
	return NumBytes;
}

/**
 * \brief Print a signed int to serial as human readable text followed by
 * return character.
 * \param i is the value to be printed.
 * \param base is the number base to use. Can be 2,6,8,10.
 * \return total number of bytes written.
 */
template <typename tag>
auto hardware::serial_api<tag>::print_line(int i, int base = 10) -> char_count {
	if (base == 2) {
		char_count NumBytes = Serial.println(i, BIN);
		return NumBytes;
	}
	else if (base == 6) {
		char_count NumBytes = Serial.println(i, HEX);
		return NumBytes;
	}
	else if (base == 8) {
		char_count NumBytes = Serial.println(i, OCT);
		return NumBytes;
	}
	else if (base == 10) {
		char_count NumBytes = Serial.println(i, DEC);
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
 */
template <typename tag>
auto hardware::serial_api<tag>::print_line(unsigned int i, int base = 10) -> char_count {
	if (base == 2) {
		char_count NumBytes = Serial.println(i, BIN);
		return NumBytes;
	}
	else if (base == 6) {
		char_count NumBytes = Serial.println(i, HEX);
		return NumBytes;
	}
	else if (base == 8) {
		char_count NumBytes = Serial.println(i, OCT);
		return NumBytes;
	}
	else if (base == 10) {
		char_count NumBytes = Serial.println(i, DEC);
		return NumBytes;
	}
	else {
		Serial.print("Unrecognised input.  Base must be 2,6,8,10");
	}
}

/**
 * \brief Print a signed long int to serial as human readable text followed
 * by return character.
 * \param i is the value to be printed.
 * \param base is the number base to use. Can be 2,6,8,10.
 * \return total number of bytes written.
 */
template <typename tag>
auto hardware::serial_api<tag>::print_line(long i, int base = 10) -> char_count {
	if (base == 2) {
		char_count NumBytes = Serial.println(i, BIN);
		return NumBytes;
	}
	else if (base == 6) {
		char_count NumBytes = Serial.println(i, HEX);
		return NumBytes;
	}
	else if (base == 8) {
		char_count NumBytes = Serial.println(i, OCT);
		return NumBytes;
	}
	else if (base == 10) {
		char_count NumBytes = Serial.println(i, DEC);
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
 */
template <typename tag>
auto hardware::serial_api<tag>::print_line(unsigned long i, int base = 10) -> char_count {
	if (base == 2) {
		char_count NumBytes = Serial.println(i, BIN);
		return NumBytes;
	}
	else if (base == 6) {
		char_count NumBytes = Serial.println(i, HEX);
		return NumBytes;
	}
	else if (base == 8) {
		char_count NumBytes = Serial.println(i, OCT);
		return NumBytes;
	}
	else if (base == 10) {
		char_count NumBytes = Serial.println(i, DEC);
		return NumBytes;
	}
	else {
		Serial.print("Unrecognised input.  Base must be 2,6,8,10");
	}
}

/**
 * \brief Print an unsigned char to serial as human readable text followed
 * by return character.
 * \param c is the value to be printed.
 * \param base is the number base to use. Can be 2,6,8,10.
 * \return total number of bytes written.
 */
template <typename tag>
auto hardware::serial_api<tag>::print_line(unsigned char c, int base = 10) -> char_count {
	if (base == 2) {
		char_count NumBytes = Serial.println((uint8_t)c, BIN);
		return NumBytes;
	}
	else if (base == 6) {
		char_count NumBytes = Serial.println((uint8_t)c, HEX);
		return NumBytes;
	}
	else if (base == 8) {
		char_count NumBytes = Serial.println((uint8_t)c, OCT);
		return NumBytes;
	}
	else if (base == 10) {
		char_count NumBytes = Serial.println((uint8_t)c, DEC);
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
 */
template <typename tag>
auto hardware::serial_api<tag>::print_line(double i, int base = 2) -> char_count {
	double precision = pow(10, base);
	Serial.print(int(i));  //print int part
	Serial.print("."); // print decimal point
	unsigned int frac;
	if (i >= 0)
		frac = (i - int(i)) * precision;
	else
		frac = (int(i) - i) * precision;
	char_count NumBytes = Serial.println(frac, DEC);
	return NumBytes;
}

/**
	 * \brief write a unsigned long int to serial as a series of bytes.
	 * \param n is the value.
	 * \return total number of bytes written.
	 */
template <typename tag>
auto hardware::serial_api<tag>::write(unsigned long n) -> char_count {
	char_count NumBytes = Serial.write(n);
	return NumBytes;
}

/**
	 * \brief write a signed long int to serial as a series of bytes.
	 * \param n is the value.
	 * \return total number of bytes written.
	 */
template <typename tag>
auto hardware::serial_api<tag>::write(long n) -> char_count {
	char_count NumBytes = Serial.write(n);
	return NumBytes;
}

/**
 * \brief write a int to serial as a series of bytes.
 * \param n is the value.
 * \return total number of bytes written.
 */
template <typename tag>
auto hardware::serial_api<tag>::write(int n) -> char_count {
	char_count NumBytes = Serial.write(n);
	return NumBytes;
}

/**
 * \brief write a unsigned byte to serial.
 * \param n is the value.
 * \return total number of bytes written.
 */
template <typename tag>
auto hardware::serial_api<tag>::write(uint8_t n) -> char_count {
	char_count NumBytes = Serial.write(n);
	return NumBytes;
}

/**
 * \brief write a string to serial as a series of byte.
 * \param str is a null terminated string.
 * \return total number of bytes written.
 */
template <typename tag>
auto hardware::serial_api<tag>::write(const char* str) -> char_count {
	char_count NumBytes = Serial.write(str);
	return NumBytes;
}

/**
 * \brief write n bytes to serial.
 * \param string is an array of char.
 * \param size is number of bytes to send.
 * \return total number of bytes written.
 */
template <typename tag>
auto hardware::serial_api<tag>::write(char const* string, size_t size) -> char_count {
	char_count NumBytes = Serial.write(string, size);
	return NumBytes;
}

/**
 * \brief The begin method set the data rate of hardware serial connection
 * and start communication.
 * \param baud in bits per seconds. Common values include 9600,38400,115200,
 */
template <typename tag>
auto hardware::serial_api<tag>::enable(unsigned long baud = 115200) -> void {
	Serial.begin(baud);
}

/**
 * \brief The end method stop serial communication.
 */
template <typename tag>
 auto hardware::serial_api<tag>::end() -> void {
	 Serial.end();
}

/**
 * \brief The available method returns number of bytes available for read.
 * \return number of bytes available to read.
 */
template <typename tag>
auto hardware::serial_api<tag>::input_byte_in_buffer() -> int {
	//Get the number of bytes (characters) available for reading from the serial port.
	int NumBytesRead = Serial.available();
	return NumBytesRead;
}

/**
 * \brief The available_for_write method returns the number of bytes
 * available for write without blocking operation. Essentially the amount of
 * space left in the buffer.
 * \return number of bytes.
 */
template <typename tag>
auto hardware::serial_api<tag>::output_buffer_space() -> int {
	//Get the number of bytes (characters) available for writing in the serial buffer without blocking the write operation.
	int NumBytesAvaliable = Serial.availableForWrite();
	return NumBytesAvaliable;
}

/**
 * \brief The peek method returns the next byte in the stream without
 * removing it from the internal buffer.
 * \return the next byte
 */
template <typename tag>
auto hardware::serial_api<tag>::peek() -> int {
	int NextByte = Serial.peek();
	return NextByte;
}

/**
 * \brief The read method return the next character from stream.
 * \return the next byte
 */
template <typename tag>
auto hardware::serial_api<tag>::read() -> int {
	int NextByte = Serial.read();
	return NextByte;
}

/**
 * \brief The read_bytes method read from stream until either length char
 * have been read, or times out;
 * \param buffer is a pointer to the return buffer.
 * \param length is the maximum number of bytes to read.
 * \return number of character read.
 */
template <typename tag>
auto hardware::serial_api<tag>::read_bytes(char* buffer, char_count length) -> char_count {
	//reads characters from the serial port into a buffer
	char_count ReadBytes = Serial.readBytes(buffer, length);
	return ReadBytes;
}

/**
 * \brief The read_bytes_until method read from stream until either
 * terminator has been read, timeout or length character have been read.
 * \param terminator is the terminator to look for.
 * \param buffer is a pointer to the return buffer.
 * \param length is the maximum number of bytes to read.
 * \return number of bytes read, not including terminator.
 */
template <typename tag>
auto hardware::serial_api<tag>::read_bytes_until(char terminator, char* buffer, char_count length)->char_count {
	//reads characters from the serial buffer into an array
	char_count ReadBytes = Serial.readBytesUntil(terminator, buffer, length);
	return ReadBytes;
}

/**
 * \brief The timeout_duration method sets the maximum milliseconds to wait
 * for serial data.
 * \param timeout in milliseconds.
 */
template <typename tag>
auto hardware::serial_api<tag>::timeout_duration(units::milliseconds timeout) -> void {
	Serial.setTimeout(timeout);
}

/**
 * \brief The flush method wait until transmission of outgoing serial data
 * is complete.
 */
template <typename tag>
auto hardware::serial_api<tag>::flush() -> void {
	//Waits for the transmission of outgoing serial data to complete
	Serial.flush();
}

/**
 * \brief The clear method clear the command line console. Do nothing in
 * the Arduino version.
 */
template <typename tag>
auto hardware::serial_api<tag>::clear() -> void {
	Serial.print("Command line cleared");
	system("CLS");
}


////////////////////
//DISPLAY
////////////////////
/**
* \brief Initialise the lcd display.
*/

auto hardware::display::enable() -> void {
	DFRobot_LCD::init();
}

/**
 * \brief
 * \param cursor_position method set the cursor position.
 */
auto hardware::display::cursor(coordinate cursor_position) -> void {
	DFRobot_LCD::setCursor(cursor_position.row, cursor_position.column);
}

/**
 * \brief print a string
 * \param string c string to be displayed.
 * \return number of char printed.
 */
auto hardware::display::print(char const* string)->size_t {
	DFRobot_LCD::printstr(string);
	return sizeof(string);
}

/**
 * \brief print number in in decimal format.
 * \param n is the number to be printed.
 * \return number of char printed.
 */
//static auto print(int n)->size_t;

/**
 * \brief print double in in decimal format.
 * \param n is the number to be printed.
 * \return number of char printed.
 */
//static auto print(double n)->size_t;

/**
 * \brief print a message about the layout of a cell (ie is the cell open or
 * closed in the four directions). \param maze is the layout of the maze
 * \param cell is the cell to be printed.
 * \return number of char printed.
 */
//static auto print(maze_layout maze, cell_location cell)->size_t;

/**
 * \brief clear the lcd display.
 */
auto clear() -> void {
	DFRobot_LCD::clear();
}

















///////////////////////////////////////////////////

// NOTE: Explicit Instantiation of Template Classes...
// Template was split into .cpp and .h file. Compiler now knows that it will
// compile the .cpp for the classes using the specified pins

//Serial & Bluetooth
template class hardware::serial_api<hardware::serial_tag<1>>; //bluetooth
template class hardware::serial_api<hardware::serial_tag<0>>; //serial

//LCD