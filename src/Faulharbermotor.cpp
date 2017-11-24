#include "Faulharbermotor.h"
#include "ros_interface.h"

Faulharbermotor::Faulharbermotor()
{
    //ini parameters
    motorConnect=1;
    status=0;
    for (int i=0;i<NODENO;i++)
    {
        speedLimit[i]=800;
        cmdPos[i]=0;
        msrPos[i]=0;
        msrTem[i]=0;
        msrCur[i]=0;
    }
    speedScale=0.5;
    prevDataExchangeTime=0;
    needleDir=1;
    enableSig = false;
    //set time tick
    sysTime= new QTime();
    sysTime->start();

    //open port
    //sertialPort1 =new QSerialPort("Com3");
    sertialPort1 =new QSerialPort("/dev/ttyS0");
    sertialPort1->setBaudRate(QSerialPort::Baud115200);
    sertialPort1->open(QIODevice::ReadWrite);

    // check connection ready or not by inquiring position
    for (int node=0; node<NODENO; node++)
    {
        int tem0=getPosition(node);
        msrPos[node]= tem0;
        cmdPos[node]= tem0;
        if (tem0!=ERROR_INDEX && tem0!=TIMEOUT_INDEX )
        {
            cout<<"motor"<< node <<" is OK"<<endl;
            motorConnect=motorConnect*1;
        }
        else
        {
            cout<<"motor"<< node <<" cannot be connected."<<endl;
            motorConnect=motorConnect*0;
            exit(1);
        }
    }

    // if all motors are connected, setup device
    if ( motorConnect>0 )
    {
        for (int node=0; node<NODENO; node++)
        {
            // disable feedback information to achieve high speed communicaiton
            char ComPos[15];
            sprintf(ComPos, "%dANSW0\n",node);
            sertialPort1->write(ComPos);
            QThread::msleep(100);
            //setup speed limit
            char ComPos1[20];
            sprintf(ComPos1, "%dSP%d\n",node,speedLimit[node]);
            sertialPort1->write(ComPos1);
            QThread::msleep(100);
            //set  motor home position
//            char ComPos2[15];
//            sprintf(ComPos2, "%dHO\n",node);
//            sertialPort1->write(ComPos2);
//            QThread::msleep(100);
        }

        // enable motor
//        enable();

        //start control tick
        connect(&controlTick, SIGNAL(timeout()) , this, SLOT( control_loop()) );

        connect(&timer_msg, SIGNAL(timeout()) , this, SLOT( msg_loop()) );

    }
}

void Faulharbermotor::start() {
    controlTick.start(20);
    // ROS loop timer
    timer_msg.start(100);
    std::cout << "Enable:" << enableSig << std::endl;
}

deviceInfomation Faulharbermotor::getAllCtrlInfomation()
{
    deviceInfomation ctrlInfo;
    for (int i=0;i<NODENO;i++)
    {
        ctrlInfo.msrPos[i]=msrPos[i];
        ctrlInfo.cmdPos[i]=cmdPos[i];
        ctrlInfo.msrTem[i]=msrTem[i];
        ctrlInfo.msrCur[i]=msrCur[i];
    }

        ctrlInfo.connectSig=motorConnect;
        ctrlInfo.enableSig=enableSig;
        ctrlInfo.speedScale=speedScale;
        ctrlInfo.status=status;

        ctrlInfo.speedScale=speedScale;
        ctrlInfo.absLockUpPos=absLockUpPos;
        ctrlInfo.absLockDownPos=abdLockDownPos;
        ctrlInfo.absOpenPos=absOpenPos;
        ctrlInfo.absClosePos=absClosePos;
        return ctrlInfo;
}

bool    Faulharbermotor::isConnected()
{
    if (motorConnect==1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

double  Faulharbermotor::getMsrPos(int node)
{
    return msrPos[node];
}

void    Faulharbermotor::setCmdPos(double pos, int node)
{
    cmdPos[node]=pos;
}

double  Faulharbermotor::getMsrTem(int node)
{
    return msrTem[node];
}

double  Faulharbermotor::getMsrCur(int node)
{
    return msrCur[node];
}

void    Faulharbermotor::lockUp()
{
    cout<<"lock up"<<endl;
    setCmdPos(absLockUpPos, Motor1);
}

void    Faulharbermotor::lockDown()
{
    cout<<"lock down"<<endl;
    setCmdPos(abdLockDownPos, Motor1);
}

void    Faulharbermotor::enable()
{
    enableSig=true;
    sertialPort1->write("EN\n");
}

void    Faulharbermotor::disable()
{
    enableSig=false;
    sertialPort1->write("DI\n");
}

void    Faulharbermotor::enableToggle()
{
    enableSig=!enableSig;
    if (enableSig)
    {
        enable();
    }
    else
    {
        disable();
    }
}

void    Faulharbermotor::runSingleStitch()
{
    enable();
    status=1;
}

bool    Faulharbermotor::isDeviceBusy()
{
    if (status==0)
    {
        // device is free
        return false;
    }
    else
    {
        // device is busy
        return true;
    }
}

void    Faulharbermotor::setPosition(double pos_f, int node)
{
    int pos= pos_f;
    char ComPos[20];
    sprintf(ComPos, "%dLA%d\n%dM\n", node, pos, node);
    sertialPort1->write(ComPos);
}

void    Faulharbermotor::setVelocity(double vel_f, int node)
{
    int vel= vel_f;
    char ComPos[20];
    sprintf(ComPos, "%dV%d\n", node, vel);
    sertialPort1->write(ComPos);
}

double  Faulharbermotor::getTemprature(int node)
{
    int startTime=sysTime->elapsed();
    double curr_temp=0;
    inquireTemprature(node);
    QThread::msleep(10);
    while (sertialPort1->waitForReadyRead(TIMEOUT))
    {
        QByteArray temp=sertialPort1->readAll();
        receivedData.append(temp);
        for (int i=1;i<receivedData.size(); i++ )
        {
            if ( (int(receivedData.at(i-1))==13 )&& (int(receivedData.at(i))==10)  )
            {
                curr_temp=atof( receivedData.toStdString().c_str() );
                receivedData.clear();

                int endTime=sysTime->elapsed();
                int timeElapse= endTime - startTime;
                //cout<< "Motor"<< node<<" temprature:" << curr_temp<< "   Time elapsed:"<<timeElapse<< "ms"<<endl;
                return curr_temp;
            }
        }
        QThread::msleep(5);
    }
    cout<< "get temprature timeout: " << node <<endl;
    return TIMEOUT_INDEX;
}

double  Faulharbermotor::getPosition(int node)
{
    int startTime=sysTime->elapsed();
    double curr_temp=0;
    inquirePos(node);
    QThread::msleep(5);
    while (sertialPort1->waitForReadyRead(TIMEOUT))
    {
        QByteArray temp=sertialPort1->readAll();
        receivedData.append(temp);
        for (int i=1;i<receivedData.size(); i++ )
        {
            if ( (int(receivedData.at(i-1))==13 )&& (int(receivedData.at(i))==10)  )
            {
                curr_temp=atof( receivedData.toStdString().c_str() );
                receivedData.clear();

                int endTime=sysTime->elapsed();
                int timeElapse= endTime - startTime;
                //cout<< "Motor"<< node<<" position:" << curr_temp<< "      Time elapsed:"<<timeElapse<< "ms"<<endl;
                return curr_temp;
            }
        }
        QThread::msleep(1);
    }
    cout<< "get position timeout: " << node <<endl;
    return TIMEOUT_INDEX;
}

double  Faulharbermotor::getCurrent(int node)
{
    int startTime=sysTime->elapsed();
    double curr_temp=0;
    inquireCurrent(node);
    QThread::msleep(10);
    while (sertialPort1->waitForReadyRead(TIMEOUT))
    {
        QByteArray temp=sertialPort1->readAll();
        receivedData.append(temp);
        for (int i=1;i<receivedData.size(); i++ )
        {
            if ( (int(receivedData.at(i-1))==13 )&& (int(receivedData.at(i))==10)  )
            {
                curr_temp=atof( receivedData.toStdString().c_str() );
                receivedData.clear();

                int endTime=sysTime->elapsed();
                int timeElapse= endTime - startTime;
                cout<< "Motor"<< node<<" Current:" << curr_temp<< "      Time elapsed:"<<timeElapse<< "ms"<<endl;
                return curr_temp;
            }
        }
        QThread::msleep(5);
    }
    cout<< "get current timeout: " << node <<endl;
    return TIMEOUT_INDEX;
}

void Faulharbermotor::control_loop ()
{
    // data exchange in manual control mode
    if (status==0)
    {
        for (int i=0;i<NODENO;i++)
        {
            // get position and temprature and current with 10Hz
            if ( sysTime->elapsed()-prevDataExchangeTime >100)
            {
                prevDataExchangeTime=sysTime->elapsed();
                msrTem[i]=getTemprature(i);
                msrPos[i]=getPosition(i);
                msrCur[i]=getCurrent(i);
            }
            // set position
            setPosition(cmdPos[i], i);
            QThread::msleep(2);
        }
    }

    // data exchange in auto stitching mode
    const double motor0Speed=10;
    const double motor1Speed= 600;
    const double stopMargin0= 5;
    const double stopMargin1= 100;

    // moto 0 move forward
    if (status==1)
    {
        setVelocity(-motor0Speed*speedScale, Motor0);
        status=2;
        cout<<"status2: "<<endl;
    }
        // motor 0 reach stop
    else if (status==2)
    {
        inquirePos(Motor0);
        double pos=0;
        int sig=readData(pos);
        msrPos[Motor0]=pos;
        cmdPos[Motor0]=pos;
        if (sig==1 &&   abs(pos-absClosePos)<  stopMargin0 )
        {
            setVelocity(0.0, Motor0);
            cout<<"status3: " <<"pos0 : "<<pos<<endl;
            status=3;
        }
    }
        //motor 1 rotate
    else if (status==3)
    {
        if (needleDir==1)
        {
//            setVelocity(-motor1Speed, Motor1);
            setPosition(absLockUpPos,Motor1);
        }
        else
        {
//            setVelocity(motor1Speed, Motor1);
            setPosition(abdLockDownPos,Motor1);
        }
        cout<<"status 4"<<endl;
        status=4;
    }

    else if (status==4)
    {
        inquirePos(Motor1);
        double pos=0;
        int sig=readData(pos);
        msrPos[Motor1]=pos;
        cmdPos[Motor1]=pos;
        if (needleDir==1)
        {
            if (sig==1 && abs(pos-absLockUpPos)< stopMargin1   )
            {
                cout<<"status512: "<<"pos1 : "<<pos<<endl;
                status=5;
            }
        }
        if (needleDir==-1 )
        {
            if (sig==1 && abs(pos-abdLockDownPos)< stopMargin1 )
            {
                cout<<"status52: "<<"pos1 : "<<pos<<endl;
                status=5;
            }
        }
    }
    else if (status==5 )
    {
        setVelocity(motor0Speed*speedScale, Motor0);
        status=6;
        cout<<"status 6"<<endl;
    }
    else if (status==6)
    {
        inquirePos(Motor0);
        double pos=0;
        int sig=readData(pos);
        msrPos[Motor0]=pos;
        cmdPos[Motor0]=pos;
        if (sig==1 && abs(pos -absOpenPos)<stopMargin0  )
        {
            setVelocity(0, Motor0);
            cout<<"status7: "<<"pos0 : "<<pos<<endl;
            status=7;
            needleDir=-needleDir;
        }
    }
    else if (status==7)
    {
        inquireTemprature(Motor0);
        double tempr=0;
        int sig=readData(tempr);
        if (sig==1)
        {
            msrTem[0]=tempr;
            cout<<"motor0 temprature: "<< msrTem[0]<<endl;
            status=8;
        }
    }
    else if (status==8)
    {
        inquireTemprature(Motor1);
        double tempr=0;
        int sig=readData(tempr);
        if (sig==1)
        {
            msrTem[1]=tempr;
            cout<<"motor1 temprature: "<< msrTem[1]<<endl;
            status=9;
        }
    }
    else if (status==9)
    {
        // update cmdPos to MsrPos when finish stitching
        status=0;
        disable();
        cout<<"stitch finished"<<endl;
    }
}

void    Faulharbermotor::inquireCurrent(int node)
{
    char ComPos[10];
    sprintf(ComPos, "%dGRC\n", node);
    sertialPort1->write(ComPos);
}

void    Faulharbermotor::inquirePos(int node)
{
    char ComPos[10];
    sprintf(ComPos, "%dPOS\n", node);
    sertialPort1->write(ComPos);
}

void    Faulharbermotor::inquireTemprature(int node)
{
    char ComPos[10];
    sprintf(ComPos, "%dTEM\n", node);
    sertialPort1->write(ComPos);
//    cout<<"inquire motor "<<node<<" temprature"<<endl;
}

// successful:  return 1
// timeout:     return 0;
int     Faulharbermotor::readData(double &data)
{
    while (sertialPort1->waitForReadyRead(20))
    {
        QByteArray temp=sertialPort1->readAll();
        receivedData.append(temp);
        for (int i=1;i<receivedData.size(); i++ )
        {
            if ( (int(receivedData.at(i-1))==13 )&& (int(receivedData.at(i))==10)  )
            {
                data=atof( receivedData.toStdString().c_str() );
                receivedData.clear();
                return 1;
            }
        }
        QThread::msleep(1);
    }
    return 0;
}

void Faulharbermotor::msg_loop() {
    deviceInfomation curr_info = getAllCtrlInfomation();
    Q_EMIT newSutureDeviceInfo(curr_info);
}

void Faulharbermotor::EnableMotor(bool flag) {
    if (flag) enable();
    else disable();
}

void Faulharbermotor::RunStitch() {
    // TODO: check if previous stitch finished
    runSingleStitch();
}

void Faulharbermotor::SutureSpeed(bool flag) {
    if (flag && speedScale < 1.0)   speedScale+=0.1;
    if (!flag && speedScale > 0.2)   speedScale-=0.1;
}


