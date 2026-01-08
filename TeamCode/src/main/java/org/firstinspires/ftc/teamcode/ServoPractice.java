/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

import org.firstinspires.ftc.teamcode.mechanisms.TestBenchServo;

@Disabled
@TeleOp
public class ServoPractice extends OpMode {

    TestBenchServo bench = new TestBenchServo();

    @Override
    public void init(){
        bench.init(hardwareMap);
    }

    @Override
    public void loop() {
        //position servo set to max when a button is pushed
        if (gamepad1.a){
            bench.setServoPos(1);
        }
        else {
            bench.setServoPos(0);
        }

        //continuous rotation servo to max right power when b button is pushed
        if (gamepad1.b) {
            bench.setServoPower(1);
        }
        else {
            bench.setServoPower(0);
        }
    }
}
*/