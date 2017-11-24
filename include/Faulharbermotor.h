#ifndef Faulharbermotor_H
#define Faulharbermotor_H

/*
    Suturing device API functions based on Faulharber Motor R232 Communication Protocol

    2017, Yang Hu
    The Hamlyn Centre for Robotic Surgery,
    Imperial College, London

    Description
        the motor is implemented in absolute encoder postion mode
        two modes: auto stitch/manual control
    Auto Stitch Example:
        Faulharbermotor *myDevice= new Faulharbermotor();
        if (!myDevice->isConnected)
        {
            exit();
        }

        if (! myDevice->isDeviceBusy() )
        {
            myDevice->runSingleStitch();
        }
*/


#include <QThread>
#include <fstream> 
#include <QTime>
#include <QTimer>
#include <QtSerialPort/QSerialPort>
#include <cmath>
#include <QDataStream>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <cmath>
#include <iostream>     // std::cout
#include <algorithm>    // std::max
#include "yumi_suture_def.h"

using namespace std;

class Faulharbermotor: public QObject
{
Q_OBJECT

public:
    // if the motor coupler moves, all the parameters need to calibrate agian
    const double absLockUpPos = 780;
    const double abdLockDownPos = 25822;

    const double absOpenPos=78;
    const double absClosePos=-743;

    Faulharbermotor();
    deviceInfomation getAllCtrlInfomation();

    void start();

    // auto stitching functions
    bool        isConnected();
    void        runSingleStitch();
    // should be check before runSingleStitch()
    bool        isDeviceBusy();

    // manual control functions
    double      getMsrPos(int node);
    double      getMsrTem(int node);
    double      getMsrCur(int node);
    void        setCmdPos(double  pos,  int node);
    void        lockUp();
    void        lockDown();
    void        enableToggle();
    void        enable();
    void        disable();

private:
    QSerialPort *sertialPort1;
    QByteArray  receivedData;
    int         status;
    int         needleDir;
    double      speedScale;
    bool        enableSig;
    double      cmdPos[NODENO];
    double      msrPos[NODENO];
    double      msrTem[NODENO];
    double      msrCur[NODENO];
    int         motorConnect;
    int         speedLimit[NODENO];
    QTime       *sysTime;
    QTimer      controlTick;
    QTimer      timer_msg;
    int         prevDataExchangeTime;
    double      getTemprature(int node);
    double      getPosition(int node);
    double      getCurrent(int node);
    void        setVelocity(double vel,  int node);
    void        setPosition(double pos,  int node);
    int         readData(double &data);
    void        inquireCurrent(int node);
    void        inquirePos(int node);

    void        inquireTemprature(int node);

private slots:
    // Loop for motor control
    void        control_loop();

    // Loop for sending deviceInfomation
    void        msg_loop();

public slots:
    void        EnableMotor(bool flag);

    void        RunStitch();

    // True for increase, false for decrease
    void        SutureSpeed(bool flag);




signals:
    void        newSutureDeviceInfo(deviceInfomation);
};

#endif 
