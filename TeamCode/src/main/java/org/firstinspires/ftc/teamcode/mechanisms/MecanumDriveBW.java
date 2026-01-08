package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDriveBW{
    //set up names of motor and imu object
    private DcMotorEx leftMotorFront;
    private DcMotorEx rightMotorFront;
    private DcMotorEx leftMotorRear;
    private DcMotorEx rightMotorRear;
    private IMU imu;

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
        imu = hwMap.get(IMU.class, "imu");

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

    public void driveFieldRelative(double forward, double strafe, double rotate) {
        //double theta = Math.atan2(forward, strafe);
        //double r = Math.hypot(strafe, forward);
        //theta = AngleUnit.normalizeRadians(theta -
               // imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        //double newForward = r * Math.sin(theta);
        //double newStrafe = r * Math.cos(theta);

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

        double newForward = rotY;
        double newStrafe = rotX;

        this.drive(newForward, -newStrafe, rotate);
    }

    public void resetGyro() {
        imu.resetYaw();
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

    }

    public void setMotors() {
        leftMotorFront.setPower(0.5);
        leftMotorRear.setPower(0.5);
        rightMotorFront.setPower(0.5);
        rightMotorRear.setPower(0.5);
    }

}