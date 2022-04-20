#include "rclcpp/rclcpp.hpp"

#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <unistd.h>  //Used for UART
#include <fcntl.h>   //Used for UART
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <inttypes.h>

class ArduinoSerialNode : public rclcpp::Node
{
public:
    ArduinoSerialNode() : Node("arduino_serial")
    {
        if (setupSerialPort()) {
            //mRadioLinkPublisher = this->create_publisher<my_robot_interfaces::msg::MotorControlData>("/amr/radio_link", 10);

            // We will not use a timer here. We'll just publish ever time a new packet is received from the SBUS.
            /*mTimer = this->create_wall_timer(std::chrono::milliseconds(10),
                                             std::bind(&RadioLinkPublisherNode::publishMotorControlData, this));                                                                                                                                       
            */
            RCLCPP_INFO(this->get_logger(), "Arduino Serial Node has been started.");

            std::thread t1(std::bind(&ArduinoSerialNode::infiniteLoop, this));
            t1.join();        
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Error - Failed to setup serial port");
        }
    }

private:

    /**************************************************************
      infiniteLoop()
    **************************************************************/
    void infiniteLoop() {
        while (true) {
            processArduinoSerialData();
        }
    }
    
    /**************************************************************
      setupSerialPort()
    **************************************************************/
    bool setupSerialPort()
    {
        mSerialPort = open("/dev/ttyAMA1", O_RDWR | O_NOCTTY);

        if (mSerialPort == -1)
        {

            RCLCPP_INFO(this->get_logger(), "Error - Unable to open UART.");
            return false;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Serial Port Opened Successfully");
        }

        struct termios2 options;

        if (ioctl(mSerialPort, TCGETS2, &options))
        {

            RCLCPP_INFO(this->get_logger(), "Error ioctl TCGETS2: %s", strerror(errno));
            return false;
        }

        // 8N1 at 9600 baud
        options.c_cflag &= ~CBAUD;
        options.c_cflag |= BOTHER;
        options.c_ispeed = options.c_ospeed = 115200;
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;

        options.c_lflag = 0;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_oflag = 0;
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 10;
     
        if (ioctl(mSerialPort, TCSETS2, &options))
        {
            RCLCPP_INFO(this->get_logger(), "Error ioctl TCSETS2: %s", strerror(errno));
            return false;
        }

        return true;
    }

    /**************************************************************
       processArduinoSerialData()
     **************************************************************/
    void processArduinoSerialData() {
        /*
        unsigned char rx_buffer[1];

        int rx_length = read(mSerialPort, &rx_buffer, sizeof(rx_buffer));

        if (rx_length < 0)
        {
            RCLCPP_INFO(this->get_logger(), "Error reading: %s", strerror(errno));
            return; // Note early return!!
        }
        else if (rx_length == 0)
        {
            RCLCPP_INFO(this->get_logger(), "No Data");
            return; // Note early return!!
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Value: 0x%02X", rx_buffer[0]);
        }
        */

        unsigned char tx_buffer[1];
        tx_buffer[0] = 0xDE;
        write(mSerialPort, &tx_buffer, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    //rclcpp::Publisher<my_robot_interfaces::msg::MotorControlData>::SharedPtr mRadioLinkPublisher;
    //rclcpp::TimerBase::SharedPtr mTimer;

    int mSerialPort = -1;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArduinoSerialNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}