#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/motor_control_data.hpp"
#include "example_interfaces/msg/float32.hpp"

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/sensors/PigeonIMU.h"
#include "ctre/phoenix/cci/CCI.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::sensors;
using namespace ctre::phoenix::motorcontrol::can;

class CTRENode : public rclcpp::Node
{
public:
    /**************************************************************
       CTRENode()
     **************************************************************/
    CTRENode() : Node("ctre")
    {
        initCTRE();

        mMotorControlSubscriber = this->create_subscription<my_robot_interfaces::msg::MotorControlData>(
            "/amr/radio_link", 10,
            std::bind(&CTRENode::callbackMotorControl, this, std::placeholders::_1));

        mPigeonPublisher = this->create_publisher<example_interfaces::msg::Float32>("heading", 10);

        mHeadingTimer = this->create_wall_timer(std::chrono::milliseconds(10),
                                                std::bind(&CTRENode::publishHeading, this));

        RCLCPP_INFO(this->get_logger(), "CTRE Node has been started.");
    }

private:
    /**************************************************************
       initCTRE()
     **************************************************************/
    void initCTRE() {
        mFrontRight->SetInverted(true);
        mRearRight->SetInverted(true);

        mFrontRight->ConfigOpenloopRamp(OPEN_RAMP_VALUE);
        mFrontLeft->ConfigOpenloopRamp(OPEN_RAMP_VALUE);
        mRearRight->ConfigOpenloopRamp(OPEN_RAMP_VALUE);
        mRearLeft->ConfigOpenloopRamp(OPEN_RAMP_VALUE);

        mPigeon->EnterCalibrationMode(PigeonIMU::CalibrationMode::BootTareGyroAccel);

        setMotorsToBrake();
    }

    /**************************************************************
       resetHeading()
     **************************************************************/
    void resetHeading()
    {
        mPigeon->SetYaw(0.0);
        mPigeon->SetFusedHeading(0.0);
    }

    /**************************************************************
       publishHeading()
     **************************************************************/
    void publishHeading()
    {
        double *ypr = new double[3];
        mPigeon->GetYawPitchRoll(ypr);

        RCLCPP_INFO(this->get_logger(), "Heading: %0.2f\n", std::remainder(ypr[0], 360.0d));

        auto msg = example_interfaces::msg::Float32();
        msg.data = ypr[0];
        mPigeonPublisher->publish(msg);
    }

    /**************************************************************
        setMotorsToBrake()
     **************************************************************/
    void setMotorsToBrake() {
        mFrontRight->SetNeutralMode(NeutralMode::Brake);
        mFrontLeft->SetNeutralMode(NeutralMode::Brake);
        mRearRight->SetNeutralMode(NeutralMode::Brake);
        mRearLeft->SetNeutralMode(NeutralMode::Brake);
    }

    /**************************************************************
        setMotorsToCoast()
     **************************************************************/
    void setMotorsToCoast() {
        mFrontRight->SetNeutralMode(NeutralMode::Coast);
        mFrontLeft->SetNeutralMode(NeutralMode::Coast);
        mRearRight->SetNeutralMode(NeutralMode::Coast);
        mRearLeft->SetNeutralMode(NeutralMode::Coast);
    }

    /**************************************************************
        showPDPInfo()
     **************************************************************/
    void showPDPInfo() {
        //#include "ctre/phoenix/cci/CCI.h"
        //CCIEXPORT ctre::phoenix::ErrorCode c_PDP_GetValues(int deviceID, double *voltage, double currents[], int currentCapacity, int *currentsFilled);

        //c_PDP_GetValues(int deviceID, double *voltage, double currents[], int currentCapacity, int *currentsFilled);
    }

    /**************************************************************
        callbackMotorControl()
     **************************************************************/
    void callbackMotorControl(const my_robot_interfaces::msg::MotorControlData::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "FR: %0.2f FL: %0.2f RR: %0.2f RL: %0.2f\n",
                    msg->front_right_power,
                    msg->front_left_power,
                    msg->rear_right_power,
                    msg->rear_left_power);

        mFrontRight->Set(ControlMode::PercentOutput, msg->front_right_power);
        mFrontLeft->Set(ControlMode::PercentOutput, msg->front_left_power);
        mRearRight->Set(ControlMode::PercentOutput, msg->rear_right_power);
        mRearLeft->Set(ControlMode::PercentOutput, msg->rear_left_power);

        ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
    }

    rclcpp::Subscription<my_robot_interfaces::msg::MotorControlData>::SharedPtr mMotorControlSubscriber;
    rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr mPigeonPublisher;
    rclcpp::TimerBase::SharedPtr mHeadingTimer;

    static const int RIGHT_FRONT_MOTOR_CAN_ID = 11;
    static const int RIGHT_REAR_MOTOR_CAN_ID = 12;
    static const int LEFT_FRONT_MOTOR_CAN_ID = 13;
    static const int LEFT_REAR_MOTOR_CAN_ID = 14;
    static const int PIGEON_CAN_ID = 61;

    const double OPEN_RAMP_VALUE = 1.0;

    PigeonIMU *mPigeon = new PigeonIMU(PIGEON_CAN_ID);

    TalonSRX *mFrontRight = new TalonSRX(RIGHT_FRONT_MOTOR_CAN_ID);
    TalonSRX *mFrontLeft = new TalonSRX(LEFT_FRONT_MOTOR_CAN_ID);
    TalonSRX *mRearRight = new TalonSRX(RIGHT_REAR_MOTOR_CAN_ID);
    TalonSRX *mRearLeft = new TalonSRX(LEFT_REAR_MOTOR_CAN_ID);
};

/**************************************************************
    main()
 **************************************************************/
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CTRENode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}