package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;


public class RobotOrientedMecanumDrive {
    private DcMotorEx leftMotorFront;
    private DcMotorEx rightMotorFront;
    private DcMotorEx leftMotorRear;
    private DcMotorEx rightMotorRear;

    private final double STRAFE_CORRECTION_FACTOR = 1.0;

    private final double LEFT_RIGHT_STRAFE_BALANCE = 0.97;


    public void init(HardwareMap hwMap) {
        //map motors and set motors to run on encoder
        leftMotorFront = hwMap.get(DcMotorEx.class, "left_front");
        rightMotorFront = hwMap.get(DcMotorEx.class, "right_front");
        leftMotorRear = hwMap.get(DcMotorEx.class, "left_back");
        rightMotorRear = hwMap.get(DcMotorEx.class, "right_back");

        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotorFront.setDirection(DcMotor.Direction.REVERSE);
        leftMotorRear.setDirection(DcMotor.Direction.REVERSE);

        //map imu and initialize the orientation of the imu
        IMU imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        imu.initialize(new IMU.Parameters(revOrientation));
    }

    public void drive(double forward, double strafe, double rotate) {
        //set up variables to hold calculated power values for each motor

        strafe *= STRAFE_CORRECTION_FACTOR;
        double rightMotorAdjustment = strafe * LEFT_RIGHT_STRAFE_BALANCE;

        double frontLeftPower = forward + rightMotorAdjustment + rotate;
        double backLeftPower = forward - rightMotorAdjustment + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        //constrain power and speed of motors
        double maxPower = 1.0;
        double maxSpeed = 1.0;  //might be useful if working with younger kids

        //establish whether maxPower or one of the motors is the true maxPower which will normalize all motors
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        //sets the power for the motor and normalizes it to maxPower
        leftMotorFront.setPower(maxSpeed * frontLeftPower / maxPower);
        leftMotorRear.setPower(maxSpeed * backLeftPower / maxPower);
        rightMotorFront.setPower(maxSpeed * frontRightPower / maxPower);
        rightMotorRear.setPower(maxSpeed * backRightPower / maxPower);
    }

}

