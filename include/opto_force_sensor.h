/*
    OptoForceSensor class

    Interface with Opto force sensor

    2018-02-19 Lin Zhang
    The Hamlyn Centre for Robotic Surgery,
    Imperial College, London
*/

#ifndef OPTO_FORCE_SENSOR_H
#define OPTO_FORCE_SENSOR_H

#include <QThread>
#include <QTimer>
#include <QVector3D>
#include "omd/opto.h"

class OptoForceSensor: public QThread {
    Q_OBJECT
public:
    OptoForceSensor();
    ~OptoForceSensor();


public slots:
    void    ConnectSensor ();
    void    DisconnectSensor ();

private:
    OptoDAQ m_daq;
    OptoPorts m_ports;

    QVector3D m_curr_force;

    QTimer      timer_daq;

    bool OpenPort(OptoDAQ & p_optoDAQ, OptoPorts & p_Ports, int p_iIndex);

    int ReadPackage3D(OptoDAQ & p_optoDAQ, OptoPackage & p_Package);

    void ShowInformation(OptoDAQ & p_optoDAQ, OPort & p_Port);

    bool Is3DSensor(OptoDAQ & p_optoDAQ);



private slots:
    // Loop for force data aquisition
    void        daq_loop();

signals:
    void        newForceReading(QVector3D);
};


#endif // OPTO_FORCE_SENSOR_H
