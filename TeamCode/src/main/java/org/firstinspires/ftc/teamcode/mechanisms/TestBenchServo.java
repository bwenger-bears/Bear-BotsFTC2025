/*
package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TestBenchServo {

    Servo servoPos;         //will give the servo object a more descriptive name in actual code
    CRServo servoCR;

    public void init(HardwareMap hwMap){
        servoPos = hwMap.get(Servo.class = "servo_pos");   //"exact name of servo" from configuration file
        servoCR = hwMap.get(CRServo.class = "servo_cr");
        servoPos.scaleRange(0.5, 1);                        //min of 0.5 range
        servoPos.setDirection(Servo.Direction.REVERSE);     //reverse direction of servo.  Good for matched pairs
    }
    public void setServoPos(double angle){
        servoPos.setPosition(angle);
    }

    public void setServoPower(double speed){
        servoCR.setPower(speed);
    }
}
*/