
#include "StmCommunicator.hpp"
#include <stdio.h>
#include <string.h>
#include "MatDrawFunctions.hpp"

StmCommunicator::StmCommunicator(){
    serialPort = SerialPort("/dev/ttyACM0", BaudRate::B_57600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    serialPort.SetTimeout(100); // Block for up to 100ms to receive data
    connectionEstablished = false;
}

bool StmCommunicator::connect(){
    try {
        serialPort.Open();
    }
    catch (mn::CppLinuxSerial::Exception) {
        return false;
    }
    connectionEstablished = true;
    return connectionEstablished;
}

void StmCommunicator::disconnect(){
    if (!connectionEstablished){
        return;
    }
    serialPort.Write("R\n");
    serialPort.Close();
    connectionEstablished = false;
}

void StmCommunicator::setCoordinate(Coordinate p1){
    char buffer[20];
    int x = (int16_t)p1.x;
    int y = (int16_t)p1.y;
    sprintf(buffer, "C:%d:%d:\n", x, y);
    serialPort.Write(buffer);
    return;
}

bool StmCommunicator::enableBoard(){
    if (connectionEstablished){
        serialPort.Write("dummy\n");
        serialPort.Write("E\n");
        return true;
    }
    return false;
    
}
