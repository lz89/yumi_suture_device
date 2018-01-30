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
#include <QTime>
#include <QTimer>
#include <QtSerialPort/QSerialPort>
#include <QByteArray>
#include <cmath>
#include <QDataStream>
#include <iostream>
#include <string>
#include <QApplication>
#include "yumi_suture_def.h"

using namespace std;

class Faulharbermotor: public QThread
{  
    Q_OBJECT  

private:
    QSerialPort *sertialPort1;
    QByteArray  receivedData;
    int         status;
    int         needleDir;
    double      speedScale;
    bool        enableSig;
    int         cmdPos[NODENO];
    int         msrPos[NODENO];
    int         msrTem[NODENO];
    int         msrCur[NODENO];
    int         motorConnect;
    int         speedLimit[NODENO];
    QTime       *sysTime;
    QTimer      controlTick;
    QTimer      timer_msg;
    int         prevDataExchangeTime;
    int         runSingleStitchFunc();
    int         getTemprature(int node);
    int         getPosition(int node);
    int         getCurrent(int node);
    void        setVelocity(int vel,  int node);
    void        setPosition(int pos,  int node);
    void        setPositionWithTargetReachNitofy(int pos, int node);
    void        setRelativePositionWithTargetReachNitofy(int pos,int node);
    int         readData(int &data);
    void        inquireCurrent(int node);
    void        inquirePos(int node);
    void        inquireTemprature(int node);
    void        setHome(int node);
    bool        targetReached(int timeout);
    void        setCurrentLimit(int node, int currentLimit);
public:

    // if the motor coupler moves, all the parameters need to calibrate agian
    const int absLockPos= 8*3000;
    const int absOpenPos= -85;
    const int absClosePos=-935;


    Faulharbermotor();
    ~Faulharbermotor();
    deviceInfomation getAllCtrlInfomation();

    // auto stitching functions
    bool        isConnected();
    bool        isDeviceBusy();
    void        enable();
    void        disable();
    void        enableToggle();
    void        setSpeed(double scale);
    void        start ();

private slots:
    void        controlLoop();

    // Loop for sending deviceInfomation
    void        msg_loop();

public slots:
    void        runSingleStitch();

    void        EnableMotor(bool flag);

    // TODO: NOT USED FOR NOW True for increase, false for decrease
    void        SutureSpeed(bool flag);

signals:
    void        newSutureDeviceInfo(deviceInfomation);

};

#endif 
