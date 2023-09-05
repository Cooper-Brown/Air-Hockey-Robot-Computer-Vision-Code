#include <CppLinuxSerial/SerialPort.hpp>

using namespace mn::CppLinuxSerial;

int main() {
	// Create serial port object and open serial port at 57600 baud, 8 data bits, no parity bit, one stop bit (8n1),
	// and no flow control
	SerialPort serialPort("/dev/ttyACM0", BaudRate::B_57600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
	// Use SerialPort serialPort("/dev/ttyACM0", 13000); instead if you want to provide a custom baud rate
	serialPort.SetTimeout(100); // Block for up to 100ms to receive data
	serialPort.Open();

	// WARNING: If using the Arduino Uno or similar, you may want to delay here, as opening the serial port causes
	// the micro to reset!

	// Write some ASCII data
	serialPort.Write("Hello\n");
    std::cout << "Written data" << std::endl;

	// Read some data back (will block for up to 100ms due to the SetTimeout(100) call above)
	//std::string readData;
	//serialPort.Read(readData);
	//std::cout << "Read data = \"" << readData << "\"" << std::endl;

	// Close the serial port
	serialPort.Close();
}