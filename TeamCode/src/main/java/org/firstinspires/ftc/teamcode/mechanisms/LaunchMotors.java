package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class LaunchMotors {

    private DcMotorEx intakeMotor;
    private DcMotorEx launchMotor;
    private DcMotor leftLiftMotor;
    private DcMotor rightLiftMotor;


    public void init(HardwareMap hdwMap){
        //map motors to name in configuration file of control hub
        intakeMotor = hdwMap.get(DcMotorEx.class, "intake");
        launchMotor = hdwMap.get(DcMotorEx.class, "spinner");
        leftLiftMotor = hdwMap.get(DcMotor.class, "left_lift");
        rightLiftMotor = hdwMap.get(DcMotor.class, "right_lift");

        //set motors to constant speed using encoder
        //may need to change this up so that launch motor is constant speed using DcMotorEx.setVelocity(tics/sec)
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setDirection(DcMotor.Direction.REVERSE);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    //next 2 methods set speed for each of the 2 launch motors
    public void setIntakeSpeed(double speed) {
       intakeMotor.setVelocity(speed);
    }
    public void setLaunchSpeed(double speed) {
        launchMotor.setVelocity(speed);
    }

    public void setLaunchPower (double power){
        launchMotor.setPower(power);
    }

    public double getTicksPerSec() {
        return launchMotor.getVelocity();
    }

    public void setVelocityPIDF (double p, double i, double d, double f) {
        launchMotor.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public PIDFCoefficients getVelocityPIDF(){
        return launchMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void liftBot(int position, double power) {

        leftLiftMotor.setTargetPosition(position);
        rightLiftMotor.setTargetPosition(position);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLiftMotor.setPower(power);
        rightLiftMotor.setPower(power);
    }
}
