package org.firstinspires.ftc.teamcode.testcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class LimeLight3AOpMode extends OpMode {

    Limelight3A limelight3A;

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        limelight3A.start();    //might want to move this into start function to limit battery drain
    }

    @Override
    public void loop() {
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            telemetry.addData("Target X offset", llResult.getTx());
            telemetry.addData("Target Y Offset", llResult.getTy());
            telemetry.addData("Target Area", llResult.getTa());
        }

    }
}
