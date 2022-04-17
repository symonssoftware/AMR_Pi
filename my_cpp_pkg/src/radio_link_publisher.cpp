#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/motor_control_data.hpp"

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

class RadioLinkPublisherNode : public rclcpp::Node
{
public:
    RadioLinkPublisherNode() : Node("radio_link_publisher")
    {
        if (setupSerialPort()) {
            mRadioLinkPublisher = this->create_publisher<my_robot_interfaces::msg::MotorControlData>("/amr/radio_link", 10);

            // We will not use a timer here. We'll just publish ever time a new packet is received from the SBUS.
            /*mTimer = this->create_wall_timer(std::chrono::milliseconds(10),
                                             std::bind(&RadioLinkPublisherNode::publishMotorControlData, this));                                                                                                                                       
            */
            RCLCPP_INFO(this->get_logger(), "Radio Link publisher has been started.");

            std::thread t1(std::bind(&RadioLinkPublisherNode::infiniteLoop, this));
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
            processRemoteControllerData();
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

        // 8E2 and a weird 100000 baud

        options.c_cflag |= PARENB;   // enable parity
        options.c_cflag &= ~PARODD;  // even parity
        options.c_cflag |= CSTOPB;   // enable 2 stop bits
        options.c_cflag &= ~CSIZE;   // clear character size mask
        options.c_cflag |= CS8;      // 8 bit characters
        options.c_cflag &= ~CRTSCTS; // disable hardware flow control
        options.c_cflag |= CREAD;    // enable receiver
        options.c_cflag |= CLOCAL;   // ignore modem lines

        options.c_lflag &= ~ICANON; // receive characters as they come in
        options.c_lflag &= ~ECHO;   // do not echo
        options.c_lflag &= ~ISIG;   // do not generate signals
        options.c_lflag &= ~IEXTEN; // disable implementation-defined processing

        options.c_iflag &= ~(IXON | IXOFF | IXANY); // disable XON/XOFF flow control
        options.c_iflag |= IGNBRK;                  // ignore BREAK condition
        options.c_iflag |= INPCK;                   // enable parity checking
        options.c_iflag |= IGNPAR;                  // ignore framing and parity errors
        options.c_iflag &= ~ISTRIP;                 // do not strip off 8th bit
        options.c_iflag &= ~INLCR;                  // do not translate NL to CR
        options.c_iflag &= ~ICRNL;                  // do not translate CR to NL
        options.c_iflag &= ~IGNCR;                  // do not ignore CR

        options.c_oflag &= ~OPOST;            // disable implementation-defined processing
        options.c_oflag &= ~ONLCR;            // do not map NL to CR-NL
        options.c_oflag &= ~OCRNL;            // do not map CR to NL
        options.c_oflag &= ~(ONOCR | ONLRET); // output CR like a normal person
        options.c_oflag &= ~OFILL;            // no fill characters

        options.c_cc[VTIME] = 0;
        options.c_cc[VMIN] = 0;

        options.c_cflag &= ~CBAUD;
        options.c_cflag |= BOTHER;
        options.c_ispeed = options.c_ospeed = 100000;

        if (ioctl(mSerialPort, TCSETS2, &options))
        {

            RCLCPP_INFO(this->get_logger(), "Error ioctl TCSETS2: %s", strerror(errno));
            return false;
        }

        return true;
    }

    /**************************************************************
      stopMotors()
     **************************************************************/
    void stopMotors(){

        auto msg = my_robot_interfaces::msg::MotorControlData();

        msg.front_right_power = 0.0; // Front Right Power
        msg.front_left_power = 0.0;  // Front Left Power
        msg.rear_right_power = 0.0;  // Rear Right Power
        msg.rear_left_power = 0.0;   // Rear Left Power

        mRadioLinkPublisher->publish(msg);
    }

    /**************************************************************
       processSBusBuffer()
     **************************************************************/
    bool processSBusBuffer() {

        // 25 byte packet received is little endian. Details of how the
        // package is explained on this website:
        // http://www.robotmaker.eu/ROBOTmaker/quadcopter-3d-proximity-sensing/sbus-graphical-representation

        mChannels[1] = ((mSBusBuffer[1] | mSBusBuffer[2] << 8) & 0x07FF);
        mChannels[2] = ((mSBusBuffer[2] >> 3 | mSBusBuffer[3] << 5) & 0x07FF);
        mChannels[3] = ((mSBusBuffer[3] >> 6 | mSBusBuffer[4] << 2 | mSBusBuffer[5] << 10) & 0x07FF);
        mChannels[4] = ((mSBusBuffer[5] >> 1 | mSBusBuffer[6] << 7) & 0x07FF);
        mChannels[5] = ((mSBusBuffer[6] >> 4 | mSBusBuffer[7] << 4) & 0x07FF);
        mChannels[6] = ((mSBusBuffer[7] >> 7 | mSBusBuffer[8] << 1 | mSBusBuffer[9] << 9) & 0x07FF);
        mChannels[7] = ((mSBusBuffer[9] >> 2 | mSBusBuffer[10] << 6) & 0x07FF);
        mChannels[8] = ((mSBusBuffer[10] >> 5 | mSBusBuffer[11] << 3) & 0x07FF);

        // DON'T DELETE
        //RCLCPP_INFO(this->get_logger(), "CH1 (R Stick X): %d", mChannels[1]);
        //RCLCPP_INFO(this->get_logger(), "CH2 (L Stick Y): %d", mChannels[2]);
        //RCLCPP_INFO(this->get_logger(), "CH3 (R Stick Y): %d", mChannels[3]);
        //RCLCPP_INFO(this->get_logger(), "CH4 (L Stick X): %d", mChannels[4]);
        //RCLCPP_INFO(this->get_logger(), "CH5 (R Toggle): %d", mChannels[5]);
        //RCLCPP_INFO(this->get_logger(), "CH6 (Button): %d", mChannels[6]);
        //RCLCPP_INFO(this->get_logger(), "CH7 (L Toggle): %d", mChannels[7]);
        //RCLCPP_INFO(this->get_logger(), "CH8 (Dial): %d", mChannels[8]);

        /* 
        if (sBusBuffer[23] & 0x04) {
    
            mSBusPacketsLost++;
            RCLCPP_INFO(this->get_logger(), "Signal Lost: %d", sBusPacketsLost);
       
            // Make sure the robot doesn't move when the signal is lost
            stopMotors();
        }*/

        if (mSBusBuffer[23] & 0x08)
        {

            RCLCPP_INFO(this->get_logger(), "Failsafe Engaged");

            // Make sure the robot doesn't move when the failsafe is engaged.
            stopMotors();

            return false;
        }

        // Everything is okay so process the current values
        mRadioLinkDriveX = mChannels[LEFT_STICK_X];
        mRadioLinkDriveY = mChannels[LEFT_STICK_Y];
        mRadioLinkControlX = mChannels[RIGHT_STICK_X];
        mRadioLinkControlY = mChannels[RIGHT_STICK_Y];
        mRadioLinkToggleRight = mChannels[RIGHT_TOGGLE];
        mRadioLinkToggleLeft = mChannels[LEFT_TOGGLE];
        mRadioLinkButton = mChannels[BUTTON];

        return true;
    }

    /**************************************************************
       map()
     **************************************************************/
    long map(long x, long in_min, long in_max, long out_min, long out_max) {

        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    /**************************************************************
       powerDeadband()
     **************************************************************/
    double powerDeadband(double power) {

        // Upper deadband
        if (power >= POWER_DEADBAND_THRESHOLD)
        {
            return power;
        }

        // Lower deadband
        if (power <= -POWER_DEADBAND_THRESHOLD)
        {
            return power;
        }

        // Inside deadband
        return 0.0;
    }

     /**************************************************************
       processRemoteControllerData()
     **************************************************************/
    void publishMotorControlData() {

        // Only enable drive if button is held down
        if (mRadioLinkButton == RADIOLINK_CONTROLLER_MAXIMUM_VALUE)
        {
            auto msg = my_robot_interfaces::msg::MotorControlData();

            RCLCPP_INFO(this->get_logger(), "FR: %0.2f FL: %0.2f RR: %0.2f RL: %0.2f",
                        mFrontRightPower,
                        mFrontLeftPower,
                        mRearRightPower,
                        mRearLeftPower);

            msg.front_right_power = mFrontRightPower;
            msg.front_left_power = mFrontLeftPower;
            msg.rear_right_power = mRearRightPower;
            msg.rear_left_power = mRearLeftPower;

            mRadioLinkPublisher->publish(msg);
        } else {
            stopMotors();
        }
    }

    /**************************************************************
       processRemoteControllerData()
     **************************************************************/
    void processRemoteControllerData() {

        static int sBusErrors = 0;
        static int sBusByteIndex = 0;

        uint8_t nextSBusByte;

        unsigned char rx_buffer[1];

        int rx_length = read(mSerialPort, &rx_buffer, sizeof(rx_buffer));

        if (rx_length < 0)
        {
            RCLCPP_INFO(this->get_logger(), "Error reading: %s", strerror(errno));
            return; // Note early return!!
        }
        else if (rx_length == 0)
        {
            // Nothing to do here ...
            return; // Note early return!!
        }
        else
        {
            nextSBusByte = rx_buffer[0];
            //RCLCPP_INFO(this->get_logger(), "Byte: 0x%02X", nextSBusByte);
        }

        // This is a new package and it's not the first byte then it's
        // probably the start byte B11110000 (sent MSB) so start reading the
        // 25 byte packet
        if ((sBusByteIndex == 0) && (nextSBusByte != 0x0F))
        {
            // Error - keep waiting for the start byte
        }
        else
        {
            // Fill the buffer with the bytes until the end byte B0000000 is received
            mSBusBuffer[sBusByteIndex++] = nextSBusByte;
        }

        // If we've got 25 bytes then this is a good packet so start to decode
        if (sBusByteIndex == 25)
        {
            sBusByteIndex = 0;

            if (mSBusBuffer[24] == 0x00)
            {
                //printf("Got a valid packet!\n");
                if (processSBusBuffer())
                {
                    // The value returned by RadioLink channels is 200 - 1800 so we
                    // need to normalize it to a percentage.
                    int driveX = map(mRadioLinkDriveX, RADIOLINK_CONTROLLER_MINIMUM_VALUE,
                                     RADIOLINK_CONTROLLER_MAXIMUM_VALUE, -100, 100);
                    int driveY = map(mRadioLinkDriveY, RADIOLINK_CONTROLLER_MINIMUM_VALUE,
                                     RADIOLINK_CONTROLLER_MAXIMUM_VALUE, -100, 100);
                    int controlX = map(mRadioLinkControlX, RADIOLINK_CONTROLLER_MINIMUM_VALUE,
                                       RADIOLINK_CONTROLLER_MAXIMUM_VALUE, -100, 100);

                    double x = ((double)driveX / 100.0) * 1.1;
                    double y = (double)driveY / -100.0;
                    double rx = (double)controlX / 100.0;

                    // Denominator is the largest motor power (absolute value) or 1
                    // This ensures that all powers maintain the same ratio, but only
                    // when at least one is out of the range of [-1, 1]
                    double denominator = std::max(abs(y) + abs(x) + abs(rx), 1.0);
                    mFrontLeftPower = (y + x + rx) / denominator;
                    mRearLeftPower = (y - x + rx) / denominator;
                    mFrontRightPower = (y - x - rx) / denominator;
                    mRearRightPower = (y + x - rx) / denominator;

                    mFrontRightPower = powerDeadband(mFrontRightPower);
                    mFrontLeftPower = powerDeadband(mFrontLeftPower);
                    mRearRightPower = powerDeadband(mRearRightPower);
                    mRearLeftPower = powerDeadband(mRearLeftPower);

                    /*
                    RCLCPP_INFO(this->get_logger(), "FR: %0.2f FL: %0.2f RR: %0.2f RL: %0.2f",
                           mFrontRightPower,
                           mFrontLeftPower,
                           mRearRightPower,
                           mRearLeftPower);
                    */

                    publishMotorControlData();
                }
            }
            else
            {
                //RCLCPP_INFO(this->get_logger(), "sBusErrors = %d", sBusErrors);
                sBusErrors++; // What do these mean?
            }
        }
    }

    rclcpp::Publisher<my_robot_interfaces::msg::MotorControlData>::SharedPtr mRadioLinkPublisher;
    rclcpp::TimerBase::SharedPtr mTimer;

    int mSerialPort = -1;

    // Radio link channels
    // Left = 200, Mid = 1000, Right = 1800
    static const int RIGHT_STICK_X = 1;

    // Up = 200, Mid = 1000, Down = 1800
    static const int LEFT_STICK_Y = 2;

    // Up = 1800, Mid = 1000, Down = 200
    static const int RIGHT_STICK_Y = 3;

    // Left = 200, Mid = 1000, Right = 1800
    static const int LEFT_STICK_X = 4;

    // Down = 200, Mid = 1000, Up = 1800
    static const int RIGHT_TOGGLE = 5;

    // Down = 1800, Up = 200
    static const int BUTTON = 6;

    // Down = 200, Mid = 1000, Up = 1800
    static const int LEFT_TOGGLE = 7;

    // All the way left = 200, All the way right 1800
    static const int DIAL = 8;

    static const int RADIOLINK_CONTROLLER_MINIMUM_VALUE = 200;
    static const int RADIOLINK_CONTROLLER_NEUTRAL_VALUE = 1000;
    static const int RADIOLINK_CONTROLLER_MAXIMUM_VALUE = 1800;

    const double POWER_DEADBAND_THRESHOLD = 0.06;

    int mRadioLinkDriveX = RADIOLINK_CONTROLLER_NEUTRAL_VALUE;
    int mRadioLinkDriveY = RADIOLINK_CONTROLLER_NEUTRAL_VALUE;
    int mRadioLinkControlX = RADIOLINK_CONTROLLER_NEUTRAL_VALUE;
    int mRadioLinkControlY = RADIOLINK_CONTROLLER_NEUTRAL_VALUE;
    int mRadioLinkToggleRight = -1;
    int mRadioLinkToggleLeft = -1;
    int mRadioLinkButton = -1;

    double mFrontLeftPower = 0.0;
    double mRearLeftPower = 0.0;
    double mFrontRightPower = 0.0;
    double mRearRightPower = 0.0;

    int mChannels[18];
    uint8_t mSBusBuffer[25];
    int mSBusPacketsLost;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadioLinkPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}