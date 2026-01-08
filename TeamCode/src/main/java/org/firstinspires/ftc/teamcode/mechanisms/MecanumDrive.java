package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class MecanumDrive {
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private DcMotor rightLiftMotor;

    private DcMotor leftLiftMotor;

    private DcMotorEx spinner;

    private IMU imu;

    private DcMotor intake;
    private Servo stopper;

    public void init(HardwareMap hwMap){
        frontLeftMotor = hwMap.get(DcMotor.class, "left_front");
        backLeftMotor = hwMap.get(DcMotor.class, "left_back");
        frontRightMotor = hwMap.get(DcMotor.class, "right_front");
        backRightMotor = hwMap.get(DcMotor.class, "right_back");
        rightLiftMotor = hwMap.get(DcMotor.class, "right_lift");
        leftLiftMotor = hwMap.get(DcMotor.class, "left_lift");
        spinner = hwMap.get(DcMotorEx.class, "spinner");
        intake = hwMap.get(DcMotor.class, "intake");
        stopper = hwMap.get(Servo.class, "stopper");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        imu.initialize(new IMU.Parameters(RevOrientation));

    }

    public void drive(double forward, double strafe, double rotate){
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;
        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftMotor.setPower(maxSpeed * (frontLeftPower / maxPower));
        backLeftMotor.setPower(maxSpeed * (backLeftPower/maxPower));
        frontRightMotor.setPower(maxSpeed * (frontRightPower/maxPower));
        backRightMotor.setPower(maxPower * (backRightPower/maxPower));

    }
    public void driveFieldRelative(double forward, double strafe, double rotate){
//        double theta = Math.atan2(forward, strafe);
//        double r = Math.hypot(strafe, forward);
//        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
//        double newForward = r * Math. sin (theta);
//        double newStrafe = r * Math.cos (theta);
//        this.drive(newForward, newStrafe, rotate);
        double vx = forward;
        double vy = strafe;
        double omega = rotate;
        double robotAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double new_vx = vx * Math.cos(robotAngle) - vy * Math.sin(robotAngle);
        double new_vy = vx * Math.sin(robotAngle) + vy * Math.cos(robotAngle);
        this.drive(new_vx, new_vy, omega);

    }
    public void raiseLift(){
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setTargetPosition(5800);
        leftLiftMotor.setTargetPosition(-5800);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setPower(0.9);
        leftLiftMotor.setPower(0.9);
    }
    public void lowerLift(){
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setTargetPosition(0);
        leftLiftMotor.setTargetPosition(0);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setPower(0.5);
        leftLiftMotor.setPower(0.5);
    }

    public void testSpinner(){
        spinner.setPower(-1);

    }
    public void turnUsingCameras(double turnPower){
        frontLeftMotor.setPower(turnPower);
        backLeftMotor.setPower(turnPower);
        frontRightMotor.setPower(turnPower);
        backRightMotor.setPower(turnPower);

    }

    public void turnIntakeOn(double power) {
        intake.setPower(power);
    }

    public void turnIntakeOff() {
        intake.setPower(0);
    }

    public void setStopper(int position) {
        stopper.setPosition(position);
    }

    public void setSpinner(int velocity) {
        spinner.setVelocity(velocity);
    }


}

