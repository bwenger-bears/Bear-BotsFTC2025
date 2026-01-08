package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class DriveMotors {

    private DcMotorEx leftMotorFront;
    private DcMotorEx rightMotorFront;
    private DcMotorEx leftMotorRear;
    private DcMotorEx rightMotorRear;
    private IMU imu;

    //initializes DriveMotor objects
    public void init(HardwareMap hdwMap) {
        //map the objects to the name in config file..change to DcMotorEx
        leftMotorFront = hdwMap.get(DcMotorEx.class, "left_front");
        rightMotorFront = hdwMap.get(DcMotorEx.class, "right_front");
        leftMotorRear = hdwMap.get(DcMotorEx.class, "left_rear");
        rightMotorRear = hdwMap.get(DcMotorEx.class, "right_rear");

        imu = hdwMap.get(IMU.class, "imu");

        //reset encoders
        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set up encoders for constant velocity
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Make sure the motors brake when power is turned off
        leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       /* leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    //setSpeed for left motor only
    public void setSpeedLeft(double speed){
        leftDriveMotor.setVelocity(speed);
    }

    //set speed for right motor only
    public void setSpeedRight (double speed){
        rightDriveMotor.setVelocity(speed);
    }

    //sets speed for both left and right motors at the same time for straight line motion
    public void setSpeed(double speed) {
        leftDriveMotor.setVelocity(speed);
        rightDriveMotor.setVelocity(speed);
    }

    //Resets encoders and then sets target position for RUN_TO_POSITION mode.  Use prior to entering a drive State
    public void setPosition(int position) {

        leftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDriveMotor.setTargetPosition(position);
        rightDriveMotor.setTargetPosition(position);

        leftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //Will need to rethink this method. While loops make it unsuitable for a state machine
    public void turnToHeading(double desiredHeading){
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (desiredHeading > 0){
            while (gyro.getHeading() < (desiredHeading - 1)){
                leftDriveMotor.setPower(0.3);
                rightDriveMotor.setPower(-0.3);
            }
        }
        else {
            while (gyro.getHeading() < (desiredHeading + 1)){
                leftDriveMotor.setPower(-0.3);
                rightDriveMotor.setPower(0.3);
            }
        }
        leftDriveMotor.setPower(0);
        rightDriveMotor.setPower(0);
    }

    //Check to see if both drive motors are running.  Useful to check for end of drive states
    public boolean getDriveMotorsIsBusy(){
        if (leftDriveMotor.isBusy() && rightDriveMotor.isBusy()){
            return true;
        }
        else {
            return false;
        }

    //may need a getter for getCurrentPosition



    }*/
    }
}