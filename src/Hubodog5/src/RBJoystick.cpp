#include "RBJoystick.h"


int		numAxis;
int		numButton;
char	nameJoy[80];
using namespace std;
#include <iostream>

RBJoystick::RBJoystick(){
    isTerminated = true;
    connection = false;
    fdJoy = 0;

    for(int i=0; i<8; i++){
        JoyAxis[i] = 0;
    }
    for(int i=0; i<12; i++){
        JoyButton[i] = 0;
    }

    int threadID = pthread_create(&JoyThraedHandler, NULL, &RBJoyThread, this);
    if(threadID < 0){
        cout<< "Fail to create JoyThread..!!"<<endl;
    }else{
       cout<<  "Success to create JoyThread..!!"<<endl;
    }
}

RBJoystick::~RBJoystick(){
    isTerminated = false;
    usleep(200*1000);
    close(fdJoy);
}

int RBJoystick::ConnectJoy(const QString _devName){
    for(int i=0; i<8; i++){
        JoyAxis[i] = 0;
    }
    for(int i=0; i<12; i++){
        JoyButton[i] = 0;
    }

    devName = _devName;
    if((fdJoy = open(devName.toStdString().c_str(), O_RDONLY)) == -1){
        cout << "Fail to open the joystick device..!!"<<endl;
        connection = false;
        return false;
    }else{
       cout<< "Success to open the joystick device..!!"<<endl;

//        struct udev *udev = udev_new();
//        if(udev){
//            NAME_LOG(nameJOY, logINFO) << devName.toStdString().c_str();
//            struct udev_device *dev = udev_device_new_from_subsystem_sysname(udev, "input", "js0");
//            dev = udev_device_get_parent_with_subsystem_devtype(dev, "usb", "usb_device");
//            if(!dev){
//                NAME_LOG(nameJOY, logERROR) << "Unable to find parent USB device";
//            }else{
//                NAME_LOG(nameJOY, logSUCCESS) << "Good";
//                std::stringstream ss;
//                ss << std::hex << udev_device_get_sysattr_value(dev, "idVendor");
//                ss >> m_manufacturerID;

//                ss.clear();
//                ss.str("");
//                ss << std::hex << udev_device_get_sysattr_value(dev, "idProduct");
//                ss >> m_productID;

//                NAME_LOG(nameJOY, logSUCCESS) << m_manufacturerID << ", " << m_productID;

//                udev_device_unref(dev);
//                udev_unref(udev);
//            }
//        }else{
//            NAME_LOG(nameJOY, logERROR) << "Cannot create udev";
//        }

        int version;
        ioctl(fdJoy, JSIOCGVERSION, &version);
        ioctl(fdJoy, JSIOCGAXES, &numAxis);
        ioctl(fdJoy, JSIOCGBUTTONS, &numButton);
        ioctl(fdJoy, JSIOCGNAME(80), &nameJoy);


      cout << "Version: " << version<<endl;
        cout << "Joy Connect: " << nameJoy << "(" << numAxis << ", " << numButton << ")"<<endl;

        fcntl(fdJoy, F_SETFL, O_NONBLOCK);	// use non-blocking methods
        connection = true;
        return true;
    }
}

int RBJoystick::DisconnectJoy(){
    connection = false;
    close(fdJoy);
    return true;
}

void *RBJoystick::RBJoyThread(void *_arg){
    RBJoystick *rbJoy = (RBJoystick*)_arg;

    while(rbJoy->isTerminated == true){
        if(rbJoy->connection == true){
            // read the joystick
            if(sizeof(struct js_event) == read(rbJoy->fdJoy, &(rbJoy->JoyEvent), sizeof(struct js_event))){
                switch(rbJoy->JoyEvent.type & ~JS_EVENT_INIT){
                case JS_EVENT_AXIS:
                    if(rbJoy->JoyEvent.number < 8)
                    {
                        (rbJoy->JoyAxis)[rbJoy->JoyEvent.number] = rbJoy->JoyEvent.value;
//                        if(rbJoy->JoyEvent.number==1)
//                        {
//                            double tempV = (rbJoy->JoyAxis)[rbJoy->JoyEvent.number];
//                            tempV = (tempV-(32767+(-12176))/2.0)/(32767+12176)*(32767+32767);
//                            (rbJoy->JoyAxis)[rbJoy->JoyEvent.number]  = tempV;

//                        }
                    }
                    break;
                case JS_EVENT_BUTTON:
                    if(rbJoy->JoyEvent.number < 12)
                    {
                        (rbJoy->JoyButton)[rbJoy->JoyEvent.number] = rbJoy->JoyEvent.value;
                    }
                    break;
                }
            }
        }
        usleep(10*1000);
    }
    close(rbJoy->fdJoy);
}

