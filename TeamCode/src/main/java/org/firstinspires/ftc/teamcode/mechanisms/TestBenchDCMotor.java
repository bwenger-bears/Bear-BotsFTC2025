/*
package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestBenchDCMotor {
    private DcMotor motor1;   //motor name is likely more descriptive if multiple motors are involved

    public void init(HardwareMap hwMap){
        motor1 = hwMap.get(DcMotor.class,"motor1");  //name in quotes must match name in configuration file
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  //RUN_USING_ENCODER keeps a constant speed
    }
    public void setSpeed(double speed){
        motor1.setPower(speed);     //any reason to not use setPower in main code. Seems a bit redundant
    }
}
*/