#include "opto_force_sensor.h"
#include <iostream>


typedef unsigned long long mytime_t;

OptoForceSensor::OptoForceSensor() {

    connect(&timer_daq, SIGNAL(timeout()) , this, SLOT( daq_loop()) );
}

OptoForceSensor::~OptoForceSensor() {
    DisconnectSensor();
}


mytime_t Now()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    mytime_t millisecs = t.tv_sec * 1000;
    millisecs += t.tv_nsec / (1000 * 1000);
    return millisecs;
}


mytime_t NowMicro()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    mytime_t microsecs = t.tv_sec * 1000 * 1000;
    microsecs += t.tv_nsec / (1000);
    return microsecs;
}


mytime_t ElapsedTime(mytime_t p_Time)
{
    return Now() - p_Time;
}


mytime_t ElapsedTimeMicro(mytime_t p_Time)
{
    return NowMicro() - p_Time;
}

/*
 * Opens the desired port
 * it returns true if port could be opened otherwise returns false
 */
bool OptoForceSensor::OpenPort(OptoDAQ & p_optoDAQ, OptoPorts & p_Ports, int p_iIndex)
{
    msleep(10); // We wait some ms to be sure about OptoPorts enumerated PortList
    OPort * portList = p_Ports.listPorts(true);
    int iLastSize = p_Ports.getLastSize();
    if (p_iIndex >= iLastSize) {
        // index overflow
        return false;
    }
    bool bSuccess = p_optoDAQ.open(portList[p_iIndex]);
    if (bSuccess) {
        ShowInformation(p_optoDAQ, portList[p_iIndex]);
    }
    return bSuccess;
}

/*
 * Blocking call to read one 3D package (with one second timeout)
 * it return a non-negative number if it succeeded (p_Package will be the read package)
 * otherwise returns -1
 */
int OptoForceSensor::ReadPackage3D(OptoDAQ & p_optoDAQ, OptoPackage & p_Package)
{
    int iSize = -1;
    mytime_t tNow = Now();
    for (;;) {
        iSize = p_optoDAQ.read(p_Package, 0, false);
        if (iSize < 0 || iSize > 0) {
            break;
        }
        // No packages in the queue so we check the timeout
        if (ElapsedTime(tNow) >= 1000) {
            break;
        }
        msleep(1);
    }
    return iSize;
}


void OptoForceSensor::ShowInformation(OptoDAQ & p_optoDAQ, OPort & p_Port)
{
    std::string deviceName = std::string(p_Port.deviceName);
    std::string name = std::string(p_Port.name);
    std::string serialNumber = std::string (p_Port.serialNumber);
    int version = p_optoDAQ.getVersion();
    std::cout<<"OptoForceSensor Device Name: "<<deviceName<<std::endl;
    std::cout<<"OptoForceSensor Name: "<<name<<std::endl;
    std::cout<<"OptoForceSensor Serial Number: "<<serialNumber<<std::endl;
    std::cout<<"OptoForceSensor Version: "<<version<<std::endl;
}

/*
 * The function determines if the sensor is a 3D sensor
 */
bool OptoForceSensor::Is3DSensor(OptoDAQ & p_optoDAQ)
{
    opto_version optoVersion = p_optoDAQ.getVersion();
    return optoVersion != _95 && optoVersion != _64;
}

void OptoForceSensor::ConnectSensor() {
    int iPortIndex = 0;  // The index of the port which will be opened
    if (!OpenPort(m_daq, m_ports, iPortIndex)) {
        std::cout<<"OptoForceSensor: Could not open port"<<std::endl;
        return;
    }
    std::cout<<"OptoForceSensor: Connected"<<std::endl;
    timer_daq.start(50);
}

void OptoForceSensor::DisconnectSensor() {
    timer_daq.stop();
    msleep(100);
    m_daq.close();
    std::cout<<"OptoForceSensor: Disconnected"<<std::endl;
}

void OptoForceSensor::daq_loop() {
    OptoPackage optoPackage;
    int iReadSize = ReadPackage3D(m_daq, optoPackage);
    if (iReadSize < 0) {
        std::cout<<"OptoForceSensor: Something went wrong, DAQ closed!"<<std::endl;
        return;
    }
    m_curr_force.setX(optoPackage.x);
    m_curr_force.setY(optoPackage.y);
    m_curr_force.setZ(optoPackage.z);

    Q_EMIT newForceReading(m_curr_force);
}

