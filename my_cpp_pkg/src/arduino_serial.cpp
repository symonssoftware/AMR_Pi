#include "rclcpp/rclcpp.hpp"

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
    ArduinoMessage(unsigned char cmdSent, unsigned char *txDataBuffer, unsigned char txDataLength)
    {
        mCmdSent = cmdSent;
        mTxDataBuffer = txDataBuffer;
        mTxDataLength = txDataLength;
    }

    unsigned char mCmdSent;
    unsigned char mTxDataLength;
    const unsigned char *mTxDataBuffer;
};

class ArduinoSerialNode : public rclcpp::Node
{
public:
    ArduinoSerialNode() : Node("arduino_serial")
    {
        if (setupSerialPort())
        {
            //mRadioLinkPublisher = this->create_publisher<my_robot_interfaces::msg::MotorControlData>("/amr/radio_link", 10);

            // We will not use a timer here. We'll just publish ever time a new packet is received from the SBUS.
            /*mTimer = this->create_wall_timer(std::chrono::milliseconds(10),
                                             std::bind(&RadioLinkPublisherNode::publishMotorControlData, this));                                                                                                                                       
            */
            RCLCPP_INFO(this->get_logger(), "Arduino Serial Node has been started.");

            std::thread t1(std::bind(&ArduinoSerialNode::infiniteLoop, this));
            t1.join();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Error - Failed to setup serial port");
        }
    }

private:
    /**************************************************************
      infiniteLoop()
    **************************************************************/
    void infiniteLoop()
    {
        unsigned char cmdSent = 0x55;
        static const unsigned char txDataLength = 3;

        unsigned char txDataBuffer[txDataLength];
        txDataBuffer[0] = 0xAA;
        txDataBuffer[1] = 0xBB;
        txDataBuffer[2] = 0xCC;

        ArduinoMessage ledMessage = ArduinoMessage(cmdSent, &txDataBuffer[0], txDataLength);
        mMsgQueue.push(ledMessage);

        unsigned char cmdSent1 = 0x21;
        static const unsigned char txDataLength1 = 10;

        unsigned char txDataBuffer1[txDataLength1];
        txDataBuffer1[0] = 0x11;
        txDataBuffer1[1] = 0x22;
        txDataBuffer1[2] = 0x33;
        txDataBuffer1[3] = 0x44;
        txDataBuffer1[4] = 0x55;
        txDataBuffer1[5] = 0x66;
        txDataBuffer1[6] = 0x77;
        txDataBuffer1[7] = 0x88;
        txDataBuffer1[8] = 0x99;
        txDataBuffer1[9] = 0xAA;

        ArduinoMessage statusMessage = ArduinoMessage(cmdSent1, &txDataBuffer1[0], txDataLength1);
        mMsgQueue.push(statusMessage);

        while (true)
        {
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
        while (!mMsgQueue.empty())
        {
            ArduinoMessage aMsg = mMsgQueue.front();
            sendMsgAndWaitForResponse(aMsg.mCmdSent, &aMsg.mTxDataBuffer[0], aMsg.mTxDataLength);
            mMsgQueue.pop();
        }
    }

    /**************************************************************
       sendMsgAndWaitForResponse
     **************************************************************/
    void sendMsgAndWaitForResponse(unsigned char cmdSent,
                                   const unsigned char *txData,
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

        RCLCPP_INFO(this->get_logger(), "TX Buffer: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                    mTxBuffer[0], mTxBuffer[1], mTxBuffer[2],
                    mTxBuffer[3], mTxBuffer[4], mTxBuffer[5],
                    mTxBuffer[6], mTxBuffer[7], mTxBuffer[8],
                    mTxBuffer[9], mTxBuffer[10], mTxBuffer[11],
                    mTxBuffer[12], mTxBuffer[13], mTxBuffer[14],
                    mTxBuffer[15], mTxBuffer[16], mTxBuffer[17]);

        write(mSerialPort, &mTxBuffer, MESSAGE_OVERHEAD_BYTE_SIZE + txDataLength);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        unsigned char rx_buffer[ACK_NACK_MESSAGE_LENGTH];

        int rx_length = read(mSerialPort, &rx_buffer, ACK_NACK_MESSAGE_LENGTH);

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
            RCLCPP_INFO(this->get_logger(), "RX Buffer: %02X %02X %02X %02X %02X %02X %02X",
                        rx_buffer[0], rx_buffer[1], rx_buffer[2],
                        rx_buffer[3], rx_buffer[4], rx_buffer[5],
                        rx_buffer[6]);

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
                    usleep(1000);
                    ioctl(mSerialPort, TCFLSH, 2); // flush both rx and tx

                    sendMsgAndWaitForResponse(cmdSent, &txData[0], txDataLength);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Message Nacked - Giving Up!!!!");
                }
            }
        }

        // Flush the UART buffers
        usleep(1000);
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

    //rclcpp::Publisher<my_robot_interfaces::msg::MotorControlData>::SharedPtr mRadioLinkPublisher;
    //rclcpp::TimerBase::SharedPtr mTimer;

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