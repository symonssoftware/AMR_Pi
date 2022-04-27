#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float32.hpp"
#include "my_robot_interfaces/msg/arduino_serial.hpp"
#include "my_cpp_pkg/arduino_serial_cmds.hpp"

#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <queue>
#include <unistd.h> //Used for UART
#include <fcntl.h>  //Used for UART
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <inttypes.h>

/**************************************************************
   class ArduinoMessage
 **************************************************************/
class ArduinoMessage
{
public:
    ArduinoMessage(unsigned char cmdSent, std::vector<unsigned char> txDataBuffer, unsigned char txDataLength)
    {
        mCmdSent = cmdSent;
        mTxDataBuffer = txDataBuffer;
        mTxDataLength = txDataLength;
    }

    unsigned char mCmdSent;
    unsigned char mTxDataLength;
    std::vector<unsigned char> mTxDataBuffer;
};

class ArduinoSerialNode : public rclcpp::Node
{
public:
    ArduinoSerialNode() : Node("arduino_serial")
    {
        if (setupSerialPort())
        {
            mArduinoSerialSubscriber = this->create_subscription<my_robot_interfaces::msg::ArduinoSerial>(
                "/amr/arduino_serial", 10,
                std::bind(&ArduinoSerialNode::callbackArduinoSerial, this, std::placeholders::_1));

            mTxTimer = this->create_wall_timer(std::chrono::milliseconds(100),
                        std::bind(&ArduinoSerialNode::processArduinoSerialData, this));                                                                                                                                       

            RCLCPP_INFO(this->get_logger(), "Arduino Serial Node has been started.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Error - Failed to setup serial port");
        }
    }

private:

    /**************************************************************
        callbackArduinoSerial()
     **************************************************************/
    void callbackArduinoSerial(const my_robot_interfaces::msg::ArduinoSerial::SharedPtr msg) {

        switch (msg->cmd) {
            case SERIAL_COMMAND_BATTERY_VOLTAGE:
                RCLCPP_INFO(this->get_logger(), "Got a SERIAL_COMMAND_BATTERY_VOLTAGE msg");
           break;
            case SERIAL_COMMAND_HEADING:
               RCLCPP_INFO(this->get_logger(), "Got a SERIAL_COMMAND_HEADING msg");
           break;
            case SERIAL_COMMAND_LED:
               RCLCPP_INFO(this->get_logger(), "Got a SERIAL_COMMAND_LED msg");
            break;
            default:
               RCLCPP_INFO(this->get_logger(), "Got an unknown Arduino Serial msg");
        }

        ArduinoMessage arduinoSerialMessage = ArduinoMessage(msg->cmd, msg->tx_data, msg->tx_data_length);
        mMsgQueue.push(arduinoSerialMessage);
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

        // Flush the UART buffers
        usleep(1000);
        ioctl(mSerialPort, TCFLSH, 2); // flush both rx and tx

        return true;
    }

    /**************************************************************
       processArduinoSerialData()
     **************************************************************/
    void processArduinoSerialData()
    {
        // Running this in a separate thread to allow the queue to
        // drain on its own didn't work because ROS2 subscribers
        // are being ignored. Something to look into in the future
        // because calling this every time we receive a message that
        // needs to be forwarded to the Arduino is likely to cause
        // problems when loading is heavy.
        while (!mMsgQueue.empty())
        {
            if (mMsgQueue.size() > 1)
            {
                RCLCPP_INFO(this->get_logger(), "Message Queue Size: %d", mMsgQueue.size());
            }
            
            ArduinoMessage aMsg = mMsgQueue.front();
            sendMsgAndWaitForResponse(aMsg.mCmdSent, aMsg.mTxDataBuffer, aMsg.mTxDataLength);
            mMsgQueue.pop();
        }
    }

    /**************************************************************
       sendMsgAndWaitForResponse
     **************************************************************/
    void sendMsgAndWaitForResponse(unsigned char cmdSent,
                                   std::vector<unsigned char> txData,
                                   unsigned char txDataLength)
    {
        static int numRetries = 0;

        mTxBuffer[0] = FIRST_HEADER_BYTE;  // Header Byte 1
        mTxBuffer[1] = SECOND_HEADER_BYTE; // Header Byte 2
        mTxBuffer[2] = txDataLength;       // Data Length Byte
        mTxBuffer[3] = cmdSent;            // Command Byte

        for (int i = 0; i < txDataLength; i++)
        {
            mTxBuffer[i + START_OF_DATA_BYTE] = txData[i];
        }

        uint16_t crc = crc16(&mTxBuffer[START_OF_DATA_BYTE], txDataLength);

        mTxBuffer[START_OF_DATA_BYTE + txDataLength] = (unsigned char)((crc >> 8) & 0xff);
        mTxBuffer[START_OF_DATA_BYTE + txDataLength + 1] = (unsigned char)crc & 0xff;

        // RCLCPP_INFO(this->get_logger(), "TX Buffer: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
        //             mTxBuffer[0], mTxBuffer[1], mTxBuffer[2],
        //             mTxBuffer[3], mTxBuffer[4], mTxBuffer[5],
        //             mTxBuffer[6], mTxBuffer[7], mTxBuffer[8],
        //             mTxBuffer[9], mTxBuffer[10], mTxBuffer[11],
        //             mTxBuffer[12], mTxBuffer[13], mTxBuffer[14],
        //             mTxBuffer[15], mTxBuffer[16], mTxBuffer[17]);

        write(mSerialPort, &mTxBuffer, MESSAGE_OVERHEAD_BYTE_SIZE + txDataLength);

        usleep(2000);

        unsigned char rx_buffer[ACK_NACK_MESSAGE_LENGTH];

        int rx_length = read(mSerialPort, &rx_buffer, ACK_NACK_MESSAGE_LENGTH);

        if (rx_length < 0)
        {
            RCLCPP_INFO(this->get_logger(), "Error reading: %s", strerror(errno));
            return; // Note early return!!
        }
        else if (rx_length == 0)
        {
            RCLCPP_INFO(this->get_logger(), "No Response Data from Arduino");
            return; // Note early return!!
        }
        else
        {
            /*
            RCLCPP_INFO(this->get_logger(), "RX Buffer: %02X %02X %02X %02X %02X %02X %02X",
                        rx_buffer[0], rx_buffer[1], rx_buffer[2],
                        rx_buffer[3], rx_buffer[4], rx_buffer[5],
                        rx_buffer[6]);
            */
            unsigned char cmdAcked = rx_buffer[4];
            unsigned char crc1 = rx_buffer[5];
            unsigned char crc2 = rx_buffer[6];

            crc = crc16(&rx_buffer[START_OF_DATA_BYTE], rx_buffer[2]);

            unsigned char calcCrc1 = (unsigned char)((crc >> 8) & 0xff);
            unsigned char calcCrc2 = (unsigned char)crc & 0xff;

            if ((rx_buffer[3] == 0x00) &&
                (crc1 == calcCrc1) &&
                (crc2 == calcCrc2) &&
                (cmdSent == cmdAcked))
            {
                RCLCPP_INFO(this->get_logger(), "Message Acked");
             }
            else
            {
                if (numRetries++ < MAX_NUM_RETRIES)
                {
                    RCLCPP_INFO(this->get_logger(), "Message Nacked - WTF? Retrying...");

                    // Flush the UART buffers
                    ioctl(mSerialPort, TCFLSH, 2); // flush both rx and tx

                    sendMsgAndWaitForResponse(cmdSent, txData, txDataLength);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Message Nacked - Giving Up!!!!");
                }
            }
        }

        // Flush the UART buffers
        ioctl(mSerialPort, TCFLSH, 2); // flush both rx and tx
    }

    /**************************************************************
       crc16() - MODBUS Little Endian
     **************************************************************/
    unsigned short crc16(const unsigned char *data_p, unsigned char length)
    {
        unsigned int reg_crc = 0xFFFF;

        while (length--)
        {
            reg_crc ^= *data_p++;

            for (int j = 0; j < 8; j++)
            {
                if (reg_crc & 0x01)
                {
                    reg_crc = (reg_crc >> 1) ^ 0xA001;
                }
                else
                {
                    reg_crc = reg_crc >> 1;
                }
            }
        }

        //RCLCPP_INFO(this->get_logger(), "Value: 0x%04X", reg_crc);

        return reg_crc;
    }

    rclcpp::Subscription<my_robot_interfaces::msg::ArduinoSerial>::SharedPtr mArduinoSerialSubscriber;
    rclcpp::TimerBase::SharedPtr mTxTimer;
 
    static const unsigned char START_OF_DATA_BYTE = 4;
    static const unsigned char MESSAGE_OVERHEAD_BYTE_SIZE = 6; // Message size minus variable data length
    static const unsigned char FIRST_HEADER_BYTE = 0xFF;
    static const unsigned char SECOND_HEADER_BYTE = 0xFE;
    static const unsigned char MAX_MESSAGE_LENGTH_BYTES = 255;
    static const int MAX_NUM_RETRIES = 3;
    static const unsigned char ACK_NACK_MESSAGE_LENGTH = 7;

    int mSerialPort = -1;
    std::queue<ArduinoMessage> mMsgQueue;
    unsigned char mTxBuffer[MAX_MESSAGE_LENGTH_BYTES];    
};

/**************************************************************
   main
 **************************************************************/
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArduinoSerialNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}