package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LaunchServos {
    //declare objects
    private Servo stopperServo;
    //private CRServo leftControlServo;
    //private CRServo rightControlServo;

    public void init(HardwareMap hwdWare){
        //Map the object to its name in the config file on the control hub
        stopperServo = hwdWare.get(Servo.class, "stopper");
        //leftControlServo = hwdWare.get(CRServo.class, "left_servo");
        //rightControlServo = hwdWare.get(CRServo.class, "right_servo");

        // set the limits for the ramp servo.  Need to determine these limits and replace parameters below
        // rampServo.scaleRange(double min, double max);
        stopperServo.setPosition(-20);
        //reverse one of the controlServos
        //leftControlServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //setter function for rampServo angle
    public void setStopperServo(double angle){
        stopperServo.setPosition(angle);
    }

    //setter function for controlServo's speed
    public void setSpeed(double speed){
        //leftControlServo.setPower(speed);
        //rightControlServo.setPower(speed);
    }

}
