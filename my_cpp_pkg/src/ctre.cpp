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

        // Calibrate the Pigoen IMU
        mPigeon->EnterCalibrationMode(PigeonIMU::CalibrationMode::BootTareGyroAccel);
        
        // Init motors
        initMotor(mFrontRight);
        initMotor(mFrontLeft);
        initMotor(mRearRight);
        initMotor(mRearLeft);
     
        mFrontRight->SetInverted(true);
        mRearRight->SetInverted(true);

        setMotorsToBrake();
    }

     /**************************************************************
       initMotor()
     **************************************************************/
    void initMotor(TalonFX *motor) {

        motor->ConfigFactoryDefault();
        motor->ConfigOpenloopRamp(OPEN_RAMP_VALUE);
        motor->ConfigVoltageCompSaturation(11);
        motor->EnableVoltageCompensation(true);

        StatorCurrentLimitConfiguration statorCurrentLimitConfig = 
            StatorCurrentLimitConfiguration(true, 20, 25, 1.0);

        motor->ConfigStatorCurrentLimit(statorCurrentLimitConfig);

        SupplyCurrentLimitConfiguration supplyCurrentLimitConfig = 
            SupplyCurrentLimitConfiguration(true, 10, 15, 0.5);

        motor->ConfigSupplyCurrentLimit(supplyCurrentLimitConfig);
    }

    /**************************************************************
       resetHeading()
     **************************************************************/
    void resetHeading() {
        mPigeon->SetYaw(0.0);
        mPigeon->SetFusedHeading(0.0);
    }

    /**************************************************************
       publishHeading()
     **************************************************************/
    void publishHeading() {
        double *ypr = new double[3];
        mPigeon->GetYawPitchRoll(ypr);

        //RCLCPP_INFO(this->get_logger(), "Heading: %0.2f\n", std::remainder(ypr[0], 360.0d));

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
        getAverageBusVoltage()
     **************************************************************/
    double getAverageBusVoltage() {

        // I can't figure out how to get the battery voltage from the PDP 
        // directly so this is a bit of a hack. We'll just average all
        // the voltage from the drive motors for now. need to find a better
        // way of doing this. Maybe just reading one motor's value is fine.
     
        double frontRightVoltage = mFrontRight->GetBusVoltage();
        double frontLeftVoltage = mFrontLeft->GetBusVoltage();
        double rearRightVoltage = mRearRight->GetBusVoltage();
        double rearLeftVoltage = mRearLeft->GetBusVoltage();

        double averageVoltage = (frontRightVoltage + frontLeftVoltage +
            rearRightVoltage + rearLeftVoltage) / 4.0;

        RCLCPP_INFO(this->get_logger(), "Battery Voltage: %0.2fV", averageVoltage);   

        return averageVoltage;
    }

    /**************************************************************
        callbackMotorControl()
     **************************************************************/
    void callbackMotorControl(const my_robot_interfaces::msg::MotorControlData::SharedPtr msg) {
        
        /*RCLCPP_INFO(this->get_logger(), "FR: %0.2f FL: %0.2f RR: %0.2f RL: %0.2f\n",
                    msg->front_right_power,
                    msg->front_left_power,
                    msg->rear_right_power,
                    msg->rear_left_power);*/

        mFrontRight->Set(ControlMode::PercentOutput, msg->front_right_power);
        mFrontLeft->Set(ControlMode::PercentOutput, msg->front_left_power);
        mRearRight->Set(ControlMode::PercentOutput, msg->rear_right_power);
        mRearLeft->Set(ControlMode::PercentOutput, msg->rear_left_power);

        ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);

        getAverageBusVoltage();
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

    TalonFX *mFrontRight = new TalonFX(RIGHT_FRONT_MOTOR_CAN_ID);
    TalonFX *mFrontLeft = new TalonFX(LEFT_FRONT_MOTOR_CAN_ID);
    TalonFX *mRearRight = new TalonFX(RIGHT_REAR_MOTOR_CAN_ID);
    TalonFX *mRearLeft = new TalonFX(LEFT_REAR_MOTOR_CAN_ID);
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