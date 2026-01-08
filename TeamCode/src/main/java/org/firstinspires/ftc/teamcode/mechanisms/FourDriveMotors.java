package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FourDriveMotors {
    private DcMotorEx leftMotorFront;
    private DcMotorEx rightMotorFront;
    private DcMotorEx leftMotorRear;
    private DcMotorEx rightMotorRear;
    private IMU imu;

    //initializes DriveMotor objects
    public void init(HardwareMap hdwMap){
        //map the drive motors
        leftMotorFront = hdwMap.get(DcMotorEx.class, "left_front");
        rightMotorFront = hdwMap.get(DcMotorEx.class, "right_front");
        leftMotorRear = hdwMap.get(DcMotorEx.class, "left_back");
        rightMotorRear = hdwMap.get(DcMotorEx.class, "right_back");

        //reset encoders
        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set up encoders for constant velocity
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Make sure the motors brake when power is turned off
        leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reverse the left motors
        leftMotorFront.setDirection(DcMotor.Direction.REVERSE);
        leftMotorRear.setDirection(DcMotor.Direction.REVERSE);

        //Map the imu
        imu = hdwMap.get(IMU.class, "imu");

        //orient and initialize the imu
        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        imu.initialize(new IMU.Parameters
                (revOrientation));
    }

    public void driveForward(int position, double power) {
        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotorFront.setTargetPosition(position);
        rightMotorFront.setTargetPosition(position);
        leftMotorRear.setTargetPosition(position);
        rightMotorRear.setTargetPosition(position);

        leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotorFront.setPower(power);
        rightMotorFront.setPower(power);
        leftMotorRear.setPower(power);
        rightMotorRear.setPower(power);

        while (leftMotorFront.isBusy() && rightMotorFront.isBusy()
            && leftMotorRear.isBusy() && rightMotorRear.isBusy()) {
            //empty
        }
        //wait until motors have reached position
    }

    public void turn(double targetHeading, double power) {

        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double tolerance = 1.0;
        imu.resetYaw();

        double currentHeading;
        do {
            currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = targetHeading - currentHeading;
            while (error > 180) error -= 360;
            while (error <= -180) error += 360;
            if (Math.abs(error) <= tolerance) {
                break;
            }
            double turnDirection = Math.signum(error);
            double turnPower = turnDirection * power;

            leftMotorFront.setPower(turnPower);
            leftMotorRear.setPower(turnPower);
            rightMotorFront.setPower(-turnPower);
            rightMotorRear.setPower(-turnPower);
        } while (true);

        leftMotorFront.setPower(0);
        leftMotorRear.setPower(0);
        rightMotorFront.setPower(0);
        rightMotorRear.setPower(0);
        }

    public void launch() {

    }

    public void intake() {

    }

    public void stopDrive() {
        leftMotorFront.setPower(0);
        leftMotorRear.setPower(0);
        rightMotorFront.setPower(0);
        rightMotorRear.setPower(0);
    }

    public void cameraTurn(double turnPower) {
        //turn any direction
        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotorFront.setPower(turnPower);
        leftMotorRear.setPower(turnPower);
        rightMotorFront.setPower(-turnPower);
        rightMotorRear.setPower(-turnPower);
    }
}

