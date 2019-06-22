#include "Arduino.h"
#include "hardware.h"
#include "units.h"
#include "hardware_definition.h"
#include "Wire.h"
#include "external_DFRobot_LCD.h"
#include "type_traits.h"


//Redefine forward declaration of serial_tag class
namespace hardware    // UART section
{
	/**
	 * \brief The serial_tag class instantiate serial_api to the specific serial
	 * port.
	 * \tparam id is the serial number.
	 */
	template <uint8_t id>
	class serial_tag {
	public:
		/**
		   get_id assigned to the instantiation of serial id
		 */
		static constexpr auto get_id = id;
	};
}

///////////////////////////////////////////////////
////////////////////
//SERIAL & BLUETOOTH (DONE)
//Serial = Port 0
//Bluetooth = Port 1 
////////////////////
///////////////////////////////////////////////////

/**
	 * \brief Print a string to serial as is.
	 * \param string is a null terminated string.
	 * \return total number of bytes written.
*/	 
template <typename tag>
auto hardware::serial_api<tag>::print(char const* string) -> char_count {
	char_count NumBytes;
	if (tag::get_id == 0) {
		NumBytes = Serial.print(string); //Port 0
	}
	else if (tag::get_id == 1) {
		NumBytes = Serial1.print(string); //Port 1
	}
	return NumBytes;
}

/*
 * \brief Print a character to serial as is.
 * \param c is the character to print.
 * \return total number of bytes written.
 */
template <typename tag>
auto hardware::serial_api<tag>::print(char c) -> char_count {
	char_count NumBytes;
	if (tag::get_id == 0) {
		NumBytes = Serial.print(c);
	}
	else if (tag::get_id == 1) {
		NumBytes = Serial1.print(c);
	}
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
	char_count NumBytes;
	if (tag::get_id == 0) {
		if (base == 2) {
			NumBytes = Serial.print(i, BIN);
		}
		else if (base == 6) {
			NumBytes = Serial.print(i, HEX);
		}
		else if (base == 8) {
			NumBytes = Serial.print(i, OCT);
		}
		else if (base == 10) {
			NumBytes = Serial.print(i, DEC);
		}
	}
	else if (tag::get_id == 1) {
		if (base == 2) {
			NumBytes = Serial1.print(i, BIN);
		}
		else if (base == 6) {
			NumBytes = Serial1.print(i, HEX);
		}
		else if (base == 8) {
			NumBytes = Serial1.print(i, OCT);
		}
		else if (base == 10) {
			NumBytes = Serial1.print(i, DEC);
		}
	}
	return NumBytes;
}

/**
	 * \brief Print an unsigned int to serial as human readable text.
	 * \param i is the value to be printed.
	 * \param base is the number base to use. Can be 2,6,8,10.
	 * \return total number of bytes written.
*/
template <typename tag>
auto hardware::serial_api<tag>::print(unsigned int i, int base = 10) -> char_count {
	char_count NumBytes;
	if (tag::get_id == 0) {
		if (base == 2) {
			NumBytes = Serial.print(i, BIN);
		}
		else if (base == 6) {
			NumBytes = Serial.print(i, HEX);
		}
		else if (base == 8) {
			NumBytes = Serial.print(i, OCT);
		}
		else if (base == 10) {
			NumBytes = Serial.print(i, DEC);
		}
	}
	else if (tag::get_id == 1) {
		if (base == 2) {
			NumBytes = Serial1.print(i, BIN);
		}
		else if (base == 6) {
			NumBytes = Serial1.print(i, HEX);
		}
		else if (base == 8) {
			NumBytes = Serial1.print(i, OCT);
		}
		else if (base == 10) {
			NumBytes = Serial1.print(i, DEC);
		}
	}
	return NumBytes;
}


/**
 * \brief Print a signed long int to serial as human readable text.
 * \param i is the value to be printed.
 * \param base is the number base to use. Can be 2,6,8,10.
 * \return total number of bytes written.
 */
template <typename tag>
auto hardware::serial_api<tag>::print(long i, int base = 10) -> char_count {
	char_count NumBytes;
	if (tag::get_id == 0) {
		if (base == 2) {
			NumBytes = Serial.print(i, BIN);
		}
		else if (base == 6) {
			NumBytes = Serial.print(i, HEX);
		}
		else if (base == 8) {
			NumBytes = Serial.print(i, OCT);
		}
		else if (base == 10) {
			NumBytes = Serial.print(i, DEC);
		}
	}
	else if (tag::get_id == 1) {
		if (base == 2) {
			NumBytes = Serial1.print(i, BIN);
		}
		else if (base == 6) {
			NumBytes = Serial1.print(i, HEX);
		}
		else if (base == 8) {
			NumBytes = Serial1.print(i, OCT);
		}
		else if (base == 10) {
			NumBytes = Serial1.print(i, DEC);
		}
	}
	return NumBytes;
}

/**
 * \brief Print an unsigned long int to serial as human readable text.
 * \param i is the value to be printed.
 * \param base is the number base to use. Can be 2,6,8,10.
 * \return total number of bytes written.
 */
template <typename tag>
auto hardware::serial_api<tag>::print(unsigned long i, int base = 10) -> char_count {
	char_count NumBytes;
	if (tag::get_id == 0) {
		if (base == 2) {
			NumBytes = Serial.print(i, BIN);
		}
		else if (base == 6) {
			NumBytes = Serial.print(i, HEX);
		}
		else if (base == 8) {
			NumBytes = Serial.print(i, OCT);
		}
		else if (base == 10) {
			NumBytes = Serial.print(i, DEC);
		}
	}
	else if (tag::get_id == 1) {
		if (base == 2) {
			NumBytes = Serial1.print(i, BIN);
		}
		else if (base == 6) {
			NumBytes = Serial1.print(i, HEX);
		}
		else if (base == 8) {
			NumBytes = Serial1.print(i, OCT);
		}
		else if (base == 10) {
			NumBytes = Serial1.print(i, DEC);
		}
	}
	return NumBytes;
}

/**
 * \brief Print an unsigned char to serial as human readable text.
 * \param c is the value to be printed.
 * \param base is the number base to use. Can be 2,6,8,10.
 * \return total number of bytes written.
 */
template <typename tag>
auto hardware::serial_api<tag>::print(unsigned char c, int base = 10) -> char_count {
	char_count NumBytes;
	if (tag::get_id == 0) {
		if (base == 2) {
			NumBytes = Serial.print((uint8_t)c, BIN);
		}
		else if (base == 6) {
			NumBytes = Serial.print((uint8_t)c, HEX);
		}
		else if (base == 8) {
			NumBytes = Serial.print((uint8_t)c, OCT);
		}
		else if (base == 10) {
			NumBytes = Serial.print((uint8_t)c, DEC);
		}
	}
	else if (tag::get_id == 1) {
		if (base == 2) {
			NumBytes = Serial1.print((uint8_t)c, BIN);
		}
		else if (base == 6) {
			NumBytes = Serial1.print((uint8_t)c, HEX);
		}
		else if (base == 8) {
			NumBytes = Serial1.print((uint8_t)c, OCT);
		}
		else if (base == 10) {
			NumBytes = Serial1.print((uint8_t)c, DEC);
		}
	}
	return NumBytes;
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
	unsigned int frac;
	char_count NumBytes;

	if (tag::get_id == 0) {
		Serial.print(int(i));  //print int part
		Serial.print("."); // print decimal point
	}
	else if (tag::get_id == 1) {
		Serial1.print(int(i));  //print int part
		Serial1.print("."); // print decimal point
	}

	if (i >= 0)
		frac = (i - int(i)) * precision;
	else
		frac = (int(i) - i) * precision;

	if (tag::get_id == 0) {
		NumBytes = Serial.print(frac, DEC);
	}
	else if (tag::get_id == 1) {
		NumBytes = Serial1.print(frac, DEC);
	}
	return NumBytes;
}

/**
 * \brief Print a string to serial followed by return character.
 * \param string is a null terminated string.
 * \return total number of bytes written.
 */
template <typename tag>
auto hardware::serial_api<tag>::print_line(char const* string) -> char_count {
	char_count NumBytes;
	if (tag::get_id == 0) {
		NumBytes = Serial.println(string);
	}
	else if (tag::get_id == 1) {
		NumBytes = Serial1.println(string);
	}
	return NumBytes;
}

/**
 * \brief Print a character to serial as is followed by return character.
 * \param c is the character to print.
 * \return total number of bytes written.
 */
template <typename tag>
auto hardware::serial_api<tag>::print_line(char c) -> char_count {
	char_count NumBytes;
	if (tag::get_id == 0) {
		NumBytes = Serial.println(c);
	}
	else if (tag::get_id == 1) {
		NumBytes = Serial1.println(c);
	}
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
	char_count NumBytes;
	if (tag::get_id == 0) {
		if (base == 2) {
			NumBytes = Serial.println(i, BIN);
		}
		else if (base == 6) {
			NumBytes = Serial.println(i, HEX);
		}
		else if (base == 8) {
			NumBytes = Serial.println(i, OCT);
		}
		else if (base == 10) {
			NumBytes = Serial.println(i, DEC);
		}
	}
	else if (tag::get_id == 1) {
		if (base == 2) {
			NumBytes = Serial1.println(i, BIN);
		}
		else if (base == 6) {
			NumBytes = Serial1.println(i, HEX);
		}
		else if (base == 8) {
			NumBytes = Serial1.println(i, OCT);
		}
		else if (base == 10) {
				NumBytes = Serial1.println(i, DEC);
		}
	}
	return NumBytes;
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
	char_count NumBytes;
	if (tag::get_id == 0) {
		if (base == 2) {
			NumBytes = Serial.println(i, BIN);
		}
		else if (base == 6) {
			NumBytes = Serial.println(i, HEX);
		}
		else if (base == 8) {
			NumBytes = Serial.println(i, OCT);
		}
		else if (base == 10) {
			NumBytes = Serial.println(i, DEC);
		}
	}
	else if (tag::get_id == 1) {
		if (base == 2) {
			NumBytes = Serial1.println(i, BIN);
		}
		else if (base == 6) {
			NumBytes = Serial1.println(i, HEX);
		}
		else if (base == 8) {
			NumBytes = Serial1.println(i, OCT);
		}
		else if (base == 10) {
			NumBytes = Serial1.println(i, DEC);
		}
	}
	return NumBytes;
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
	char_count NumBytes;
	if (tag::get_id == 0) {
		if (base == 2) {
			NumBytes = Serial.println(i, BIN);
		}
		else if (base == 6) {
			NumBytes = Serial.println(i, HEX);
		}
		else if (base == 8) {
			NumBytes = Serial.println(i, OCT);
		}
		else if (base == 10) {
			NumBytes = Serial.println(i, DEC);
		}
	}
	else if (tag::get_id == 1) {
		if (base == 2) {
			NumBytes = Serial1.println(i, BIN);
		}
		else if (base == 6) {
			NumBytes = Serial1.println(i, HEX);
		}
		else if (base == 8) {
			NumBytes = Serial1.println(i, OCT);
		}
		else if (base == 10) {
			NumBytes = Serial1.println(i, DEC);
		}
	}
	return NumBytes;
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
	char_count NumBytes;
	if (tag::get_id == 0) {
		if (base == 2) {
			NumBytes = Serial.println(i, BIN);
		}
		else if (base == 6) {
			NumBytes = Serial.println(i, HEX);
		}
		else if (base == 8) {
			NumBytes = Serial.println(i, OCT);
		}
		else if (base == 10) {
			NumBytes = Serial.println(i, DEC);
		}
	}
	else if (tag::get_id == 1) {
		if (base == 2) {
			NumBytes = Serial1.println(i, BIN);
		}
		else if (base == 6) {
			NumBytes = Serial1.println(i, HEX);
		}
		else if (base == 8) {
			NumBytes = Serial1.println(i, OCT);
		}
		else if (base == 10) {
			NumBytes = Serial1.println(i, DEC);
		}
	}
	return NumBytes;
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
	char_count NumBytes;
	if (tag::get_id == 0) {
		if (base == 2) {
			NumBytes = Serial.println((uint8_t)c, BIN);
		}
		else if (base == 6) {
			NumBytes = Serial.println((uint8_t)c, HEX);
		}
		else if (base == 8) {
			NumBytes = Serial.println((uint8_t)c, OCT);
		}
		else if (base == 10) {
			NumBytes = Serial.println((uint8_t)c, DEC);
		}
	}
	else if (tag::get_id == 1) {
		if (base == 2) {
			NumBytes = Serial1.println((uint8_t)c, BIN);
		}
		else if (base == 6) {
			NumBytes = Serial1.println((uint8_t)c, HEX);
		}
		else if (base == 8) {
			NumBytes = Serial1.println((uint8_t)c, OCT);
		}
		else if (base == 10) {
			NumBytes = Serial1.println((uint8_t)c, DEC);
		}
	}
	return NumBytes;
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
	unsigned int frac;
	char_count NumBytes;

	if (tag::get_id == 0) {
		Serial.print(int(i));  //print int part
		Serial.print("."); // print decimal point
	}
	else if (tag::get_id == 1) {
		Serial1.print(int(i));  //print int part
		Serial1.print("."); // print decimal point
	}

	if (i >= 0)
		frac = (i - int(i)) * precision;
	else
		frac = (int(i) - i) * precision;

	if (tag::get_id == 0) {
		NumBytes = Serial.println(frac, DEC);
	}
	else if (tag::get_id == 1) {
		NumBytes = Serial1.println(frac, DEC);
	}
	return NumBytes;
}

/**
	 * \brief write a unsigned long int to serial as a series of bytes.
	 * \param n is the value.
	 * \return total number of bytes written.
*/
template <typename tag>
auto hardware::serial_api<tag>::write(unsigned long n) -> char_count {
	char_count NumBytes;
	if (tag::get_id == 0) {
		NumBytes = Serial.write(n);
	}
	else if (tag::get_id == 1) {
		NumBytes = Serial1.write(n);
	}
	return NumBytes;
}

/**
	 * \brief write a signed long int to serial as a series of bytes.
	 * \param n is the value.
	 * \return total number of bytes written.
	 */
template <typename tag>
auto hardware::serial_api<tag>::write(long n) -> char_count {
	char_count NumBytes;
	if (tag::get_id == 0){
		NumBytes = Serial.write(n);
	}
	else if (tag::get_id == 1) {
		NumBytes = Serial1.write(n);
	}
	return NumBytes;
}

/**
 * \brief write a int to serial as a series of bytes.
 * \param n is the value.
 * \return total number of bytes written.
 */
template <typename tag>
auto hardware::serial_api<tag>::write(int n) -> char_count {
	char_count NumBytes;
	if (tag::get_id == 0) {
		NumBytes = Serial.write(n);
	}
	else if (tag::get_id == 1) {
		NumBytes = Serial1.write(n);
	}
	return NumBytes;
}

/**
 * \brief write a unsigned byte to serial.
 * \param n is the value.
 * \return total number of bytes written.
 */
template <typename tag>
auto hardware::serial_api<tag>::write(uint8_t n) -> char_count {
	char_count NumBytes;
	if (tag::get_id == 0) {
		NumBytes = Serial.write(n);
	}
	else if (tag::get_id == 1) {
		NumBytes = Serial1.write(n);
	}
	return NumBytes;
}

/**
 * \brief write a string to serial as a series of byte.
 * \param str is a null terminated string.
 * \return total number of bytes written.
 */
template <typename tag>
auto hardware::serial_api<tag>::write(const char* str) -> char_count {
	char_count NumBytes;
	if (tag::get_id == 0) {
		NumBytes = Serial.write(str);
	}
	else if (tag::get_id == 1) {
		NumBytes = Serial1.write(str);
	}
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
	char_count NumBytes;
	if (tag::get_id == 0) {
		NumBytes = Serial.write(string, size);
	}
	else if (tag::get_id == 1) {
		NumBytes = Serial1.write(string, size);
	}
	return NumBytes;
}

/**
 * \brief The begin method set the data rate of hardware serial connection
 * and start communication.
 * \param baud in bits per seconds. Common values include 9600,38400,115200,
 */
template <typename tag>
auto hardware::serial_api<tag>::enable(unsigned long baud = 115200) -> void {
	if (tag::get_id == 0) {
		Serial.begin(baud);
	}
	else if (tag::get_id == 1) {
		Serial1.begin(baud);
	}
}

/**
 * \brief The end method stop serial communication.
 */
template <typename tag>
 auto hardware::serial_api<tag>::end() -> void {
	 if (tag::get_id == 0) {
		 Serial.end();
	 }
	 else if (tag::get_id == 1) {
		 Serial1.end();
	 }
}

/**
 * \brief The available method returns number of bytes available for read.
 * \return number of bytes available to read.
 */
template <typename tag>
auto hardware::serial_api<tag>::input_byte_in_buffer() -> int {
	//Get the number of bytes (characters) available for reading from the serial port.
	int NumBytesRead;
	if (tag::get_id == 0) {
		NumBytesRead = Serial.available();
	}
	else if (tag::get_id == 1) {
		NumBytesRead = Serial1.available();
	}
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
	int NumBytesAvaliable;
	if (tag::get_id == 0) {
		NumBytesAvaliable = Serial.availableForWrite();
	}
	else if (tag::get_id == 1) {
		NumBytesAvaliable = Serial1.availableForWrite();
	}
	return NumBytesAvaliable;
}

/**
 * \brief The peek method returns the next byte in the stream without
 * removing it from the internal buffer.
 * \return the next byte
 */
template <typename tag>
auto hardware::serial_api<tag>::peek() -> int {
	int NextByte;
	if (tag::get_id == 0) {
		NextByte = Serial.peek();
	}
	else if (tag::get_id == 1) {
		NextByte = Serial1.peek();
	}
	return NextByte;
}

/**
 * \brief The read method return the next character from stream.
 * \return the next byte
 */
template <typename tag>
auto hardware::serial_api<tag>::read() -> int {
	int NextByte;
	if (tag::get_id == 0) {
		NextByte = Serial.read();
	}
	else if (tag::get_id == 1) {
		NextByte = Serial1.read();
	}
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
	char_count ReadBytes;
	if (tag::get_id == 0) {
		ReadBytes = Serial.readBytes(buffer, length);
	}
	else if (tag::get_id == 1) {
		ReadBytes = Serial1.readBytes(buffer, length);
	}
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
auto hardware::serial_api<tag>::read_bytes_until(char terminator, char* buffer, char_count length) -> char_count {
	//reads characters from the serial buffer into an array
	char_count ReadBytes;
	if (tag::get_id == 0) {
		ReadBytes = Serial.readBytesUntil(terminator, buffer, length);
	}
	else if (tag::get_id == 1) {
		ReadBytes = Serial1.readBytesUntil(terminator, buffer, length);
	}
	return ReadBytes;
}

/**
 * \brief The timeout_duration method sets the maximum milliseconds to wait
 * for serial data.
 * \param timeout in milliseconds.
 */
template <typename tag>
auto hardware::serial_api<tag>::timeout_duration(units::milliseconds timeout) -> void {
	if (tag::get_id == 0) {
		Serial.setTimeout(timeout);
	}
	else if (tag::get_id == 1) {
		Serial1.setTimeout(timeout);
	}
}
 
/**
 * \brief The flush method wait until transmission of outgoing serial data
 * is complete.
 */
template <typename tag>
auto hardware::serial_api<tag>::flush() -> void {
	//Waits for the transmission of outgoing serial data to complete
	if (tag::get_id == 0) {
		Serial.flush();
	}
	else if (tag::get_id == 1) {
		Serial1.flush();
	}
}

/**
 * \brief The clear method clear the command line console. Do nothing in
 * the Arduino version.
 */
template <typename tag>
auto hardware::serial_api<tag>::clear() -> void {
	if (tag::get_id == 0) {
		system("CLS");
	}
	else if (tag::get_id == 1) {
		system("CLS");
	}
}
///////////////////////////////////////////////////

// NOTE: Explicit Instantiation of Template Classes...
// Template was split into .cpp and .h file. Compiler now knows that it will
// compile the .cpp for the classes using the specified pins

//Serial & Bluetooth
template class hardware::serial_api<hardware::serial_tag<1>>; //bluetooth
template class hardware::serial_api<hardware::serial_tag<0>>; //serial

///////////////////////////////////////////////////
////////////////////
//DISPLAY  (DONE)
////////////////////
///////////////////////////////////////////////////

//Create lcd object
DFRobot_LCD lcd(16,2);

/*
brief Initialise the lcd display.
*/
auto hardware::display::enable() -> void {
	lcd.init();
}

/**
 * \brief
 * \param cursor_position method set the cursor position.
 */
auto hardware::display::cursor(coordinate cursor_position) -> void {
	lcd.cursor_on();
	lcd.setCursor(cursor_position.row, cursor_position.column);
	lcd.cursor_off();
}

/**
 * \brief print a string
 * \param string c string to be displayed.
 * \return number of char printed.
 */
auto hardware::display::print(char const* string)->size_t {
	lcd.printstr(string);
	return size_t(sizeof(string));
}

/**
 * \brief print number in in decimal format.
 * \param n is the number to be printed.
 * \return number of char printed.
 */
auto hardware::display::print(int n)->size_t {
	lcd.print(n, DEC);
	return size_t(sizeof(n));
}

/**
 * \brief print double in in decimal format.
 * \param n is the number to be printed.
 * \return number of char printed.
 */
auto hardware::display::print(double n)->size_t {
	lcd.print(n, 5);
	return size_t(sizeof(n));
}

/**
 * \brief print a message about the layout of a cell (ie is the cell open or
 * closed in the four directions). \param maze is the layout of the maze
 * \param cell is the cell to be printed.
 * \return number of char printed.
 */

auto hardware::display::print(maze_layout maze, cell_location cell)->size_t {
	//"N, S ,E, W" -> "1, 0, 0, 0"  (if pointing north direction)
}


/**
 * \brief clear the lcd display.
 */
auto hardware::display::clear() -> void {
	lcd.clear();
}

///////////////////////////////////////////////////
////////////////////
//MAZE
////////////////////
///////////////////////////////////////////////////
//definitions
#define ROWS_wall 5
#define COLS_wall 9
#define H_size 50
#define V_size 54

/**
 * \brief The maze_layout_message class holds the message received from
 * Bluetooth about the maze. Student to design the class.
 */
class maze_layout_message {
	public
		// vertical walls array
		bool Receive_V_Walls[V_size];

		// horizontal walls array
		bool Receive_H_Walls[H_size];
}


/**
 * \brief The maze_layout class holds the parsed layout of the maze.
 */
class maze_layout {
public:
	// vertical walls array
	bool Parsed_V_Walls[ROWS_wall][COLS_wall + 1];

	// horizontal walls array
	bool Parsed_H_Walls[ROWS_wall + 1][COLS_wall];

	bool NORTH;
	bool EAST;
	bool SOUTH;
	bool WEST;

	static auto print_maze() -> void;

};

/**
 * \brief The receive_maze_layout function is called to receive the layout of
 * the maze via Bluetooth. \return the message encoding the maze layout.
 */
auto  maze_layout_message::receive_maze_layout()->maze_layout_message {
	//only receive maze
	maze_layout_message Receive;
	
	while (h < H_size) {
		if (Serial1.available() > 0) {   // something came across serial
			integerValue = 0;         // throw away previous integerValue
			while (1) {            // force into a loop until 'n' is received
				incomingByte = Serial1.read();
				if (incomingByte == '\n') break;   // exit the while(1), we're done receiving
				if (incomingByte == '\r') break;   // exit the while(1), we're done receiving
				if (incomingByte == -1) continue;  // if no characters are in the buffer read() returns -1
				integerValue *= 10;  // shift left 1 decimal place
				// convert ASCII to integer, add, and shift left 1 decimal place
				integerValue = ((incomingByte - 48) + integerValue);
			}
			Receive.Receive_H_Walls[h] = integerValue;
			h++;
		}
	}

	//Start storing vertical array
	while (v < V_size) {
		if (Serial1.available() > 0) {   // something came across serial
			integerValue = 0;         // throw away previous integerValue
			while (1) {            // force into a loop until 'n' is received
				incomingByte = Serial1.read();
				if (incomingByte == '\n') break;   // exit the while(1), we're done receiving
				if (incomingByte == '\r') break;   // exit the while(1), we're done receiving
				if (incomingByte == -1) continue;  // if no characters are in the buffer read() returns -1
				integerValue *= 10;  // shift left 1 decimal place
				// convert ASCII to integer, add, and shift left 1 decimal place
				integerValue = ((incomingByte - 48) + integerValue);
			}
			Receive.Receive_V_Walls[v] = integerValue;
			v++;
		}
	}
}


/**
 * \brief cell_location class describe the position of a cell.
 */
class cell_location {
	public
		bool Robot_ROW;
		bool Robot_COL;
};


/**
 * \brief The parse_maze_layout function parses the message received from
 * Bluetooth into maze_layout. \return the parsed layout of the maze.
 */
*/
auto maze_layout::parse_maze_layout(maze_layout_message maze)->maze_layout {
	//receive
	//only save it

	//Convert array to suitable size array
	//store values into class maze layout
	maze_layout_message Receive;
	maze_layout Parsed;
	  //horizontal
	for (int row = 0; row < 3; row++) {
		for (int col = 0; col < 3; col++) {
			if (x != H_size) {
				Parsed.Parsed_H_Walls[row][col] = Receive.Receive_H_Walls[x++];
			}
		}
	}
	x = 0;
	//vertical
	for (int row = 0; row < 2; row++) {
		for (int col = 0; col < 4; col++) {
			if (x != V_size) {
				Parsed.Parsed_V_Walls[row][col] = Receive.Receive_V_Walls[x++];
			}
		}
	}
}
