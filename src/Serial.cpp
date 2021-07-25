#include "Serial.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/file.h>

#include <array>
#include <iostream>
#include <stdexcept>

// https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
// http://unixwiz.net/techtips/termios-vmin-vtime.html
// https://arduino.stackexchange.com/questions/75594/reading-from-arduino-serial-usb-using-c?newreg=a8fba0c967434da191e00da1833312b9
void Serial::openPortAndAttemptLock() {

	if (access(m_path.c_str(), F_OK) != 0) {
		throw std::runtime_error(strerror(errno));
	}

  m_serialPort = open(m_path.c_str(), O_RDWR);

  if (m_serialPort < 0) {
    throw std::runtime_error(strerror(errno));
  }

	if (flock(m_serialPort, LOCK_EX | LOCK_NB) == -1) {
		throw std::runtime_error(std::to_string(m_serialPort) + " already locked by another process.");
	}

  struct termios tty;
    
  if (tcgetattr(m_serialPort, &tty) != 0) {
    throw std::runtime_error(strerror(errno));
  }
	
	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 19;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, m_baud);
  cfsetospeed(&tty, m_baud);

	if (tcsetattr(m_serialPort, TCSANOW, &tty) != 0) {
		throw std::runtime_error(strerror(errno));
	}

	std::cout << "Serial port open\n";
}

std::string Serial::readData() {
	const auto BUFFER_SIZE{ 19 };
	std::array<char, BUFFER_SIZE> buffer;

	const auto numBytes{ read(m_serialPort, buffer.data(), BUFFER_SIZE) };

	if (numBytes < 0) {
		std::cerr << "Error reading from serial port " << strerror(errno) << '\n';
		return "";
	}

	return { std::begin(buffer), std::end(buffer) };
}

void Serial::closePort() {
	close(m_serialPort);
	std::cout << "Serial port closed.\n";
}
