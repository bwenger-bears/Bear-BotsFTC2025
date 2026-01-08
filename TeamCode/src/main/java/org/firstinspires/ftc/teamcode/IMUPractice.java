/* package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.TestBenchIMU;

@Disabled
@TeleOp
public class IMUPractice extends OpMode {

    public TestBenchIMU benchIMU = new TestBenchIMU();
    double heading;

    @Override
    public void init() {
        benchIMU.init(hardwareMap);
    }

    @Override
    public void loop() {
        heading = benchIMU.getHeading();
        if (heading < 0.5 && heading >-0.5){
            benchIMU.setSpeed(0);
        }
        else if(heading >= 0.5){
            benchIMU.setSpeed(1);
        }
        else{
            benchIMU.setSpeed(-1);
        }
        telemetry.addData("Heading", benchIMU.getHeading());
    }
}*/
