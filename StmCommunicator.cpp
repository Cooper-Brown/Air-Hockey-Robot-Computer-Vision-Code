
#include "StmCommunicator.hpp"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "MatDrawFunctions.hpp"

StmCommunicator::StmCommunicator(){
    // B_57600
    serialPort = SerialPort("/dev/ttyACM0", BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    serialPort.SetTimeout(100); // Block for up to 100ms to receive data
    connectionEstablished = false;
}

bool StmCommunicator::connect(){
    int failCount = 0;
    int maxFailCount = 10;
    while (!connectionEstablished) {
        std::cout << "Connecting, attempt: " << failCount << std::endl;
        try {
            serialPort.Open();
            connectionEstablished = true;
            break;
        }
        catch (mn::CppLinuxSerial::Exception) {
            if (++failCount == maxFailCount){
                return false;
            }
        }
        sleep(2);
    }
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
    if (!connectionEstablished){
        return;
    }
    char buffer[20];
    int x = (int32_t)p1.x;
    int y = (int32_t)p1.y;
    sprintf(buffer, "C:%d:%d:\n", x, y);
    //std::cout << buffer << std::endl;
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
