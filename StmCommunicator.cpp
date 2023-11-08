
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
    pendingTransmissionSent = true; // do not send anything unless a new coordinate is set
    lastTransmissionTime = GetTickCount();
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
    //char buffer[20];
    int x = (int32_t)p1.x;
    int y = (int32_t)p1.y;;
    sprintf(pendingTransmission, "C:%d:%d:\n", x, y);
    std::cout << "Setting Coordinate:" << pendingTransmission;
    pendingTransmissionSent = false;
    return;
}

void StmCommunicator::setStandbyCoordinate() {
    setCoordinate(Coordinate(TABLE_X_BOUNDARY_MIN + (TABLE_X_BOUNDARY_MAX-TABLE_X_BOUNDARY_MIN)/2.0, TABLE_Y_BOUNDARY_MIN));
}

bool StmCommunicator::processPendingTransmission(){
    //std::cout << "transmission Check" << std::endl;
    if (pendingTransmissionSent){
        return false;
    }

    unsigned int currentTicks = GetTickCount();
    //std::cout << currentTicks - lastTransmissionTime << std::endl;
    if (currentTicks - lastTransmissionTime < 5){
        return false;
    }
    
    //std::cout << "Sending coordinate:" << pendingTransmission << std::endl;
    if (connectionEstablished){
        serialPort.Write(pendingTransmission);
    }
    pendingTransmissionSent = true;
    lastTransmissionTime = currentTicks;
    return true;
}

bool StmCommunicator::enableBoard(){
    if (connectionEstablished){
        serialPort.Write("dummy\n");
        serialPort.Write("E\n");
        return true;
    }
    return false;
    
}
