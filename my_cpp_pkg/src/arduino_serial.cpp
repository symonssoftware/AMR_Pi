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
    bool setupSerialPort() {
        mSerialPort = open("/dev/ttyAMA1", O_RDWR | O_NOCTTY);

        if (mSerialPort == -1) {
            RCLCPP_INFO(this->get_logger(), "Error - Unable to open UART.");
            return false;
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Serial Port Opened Successfully");
        }

        struct termios2 options;

        if (ioctl(mSerialPort, TCGETS2, &options)) {

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
     
        if (ioctl(mSerialPort, TCSETS2, &options)) {
            RCLCPP_INFO(this->get_logger(), "Error ioctl TCSETS2: %s", strerror(errno));
            return false;
        }

        // Flush the UART buffers
        usleep(1000);
        ioctl(mSerialPort, TCFLSH, 2); // flush both rx and tx

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
        

        mTxBuffer[0] = 0xFF; // Header Byte 1
        mTxBuffer[1] = 0xFE; // Header Byte 2
        mTxBuffer[2] = 0x02; // Length Byte
        mTxBuffer[3] = 0x01; // Command Byte
        mTxBuffer[4] = 0xAB; // Data Byte(s)
        mTxBuffer[5] = 0xBC; // Data Byte(s)

        uint16_t crc = crc16(&mTxBuffer[START_OF_DATA_BYTE], mTxBuffer[2]);

        // For data above, should be 0x317f
        mTxBuffer[6] = (unsigned char)((crc >> 8) & 0xff);
        mTxBuffer[7] = (unsigned char)crc & 0xff;

        write(mSerialPort, &mTxBuffer, 8);
        RCLCPP_INFO(this->get_logger(), "Buffer: %02X %02X %02X %02X %02X %02X %02X %02X", 
                                         mTxBuffer[0], mTxBuffer[1], mTxBuffer[2],
                                         mTxBuffer[3], mTxBuffer[4], mTxBuffer[5],
                                         mTxBuffer[6], mTxBuffer[7]);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // Flush the UART buffers
        usleep(1000);
        ioctl(mSerialPort, TCFLSH, 1); // flush tx
    }

    /**************************************************************
       crc16() - MODBUS Little Endian
     **************************************************************/
    unsigned short crc16(const unsigned char* data_p, unsigned char length) {
       
        unsigned int reg_crc = 0xFFFF;

        while (length--) {
            reg_crc ^= *data_p++;

            for (int j = 0; j < 8; j++) {
                if (reg_crc & 0x01) {
                    reg_crc = (reg_crc >> 1) ^ 0xA001;
                }
                else {
                    reg_crc = reg_crc >> 1;
                }
            }
        }

        //RCLCPP_INFO(this->get_logger(), "Value: 0x%04X", reg_crc);

        return reg_crc;
    }

    //rclcpp::Publisher<my_robot_interfaces::msg::MotorControlData>::SharedPtr mRadioLinkPublisher;
    //rclcpp::TimerBase::SharedPtr mTimer;

    static const int START_OF_DATA_BYTE = 4;

    int mSerialPort = -1;
    unsigned char mTxBuffer[256];
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArduinoSerialNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}