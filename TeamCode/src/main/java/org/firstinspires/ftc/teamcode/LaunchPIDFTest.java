package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.LaunchMotors;

@Disabled
@TeleOp
public class LaunchPIDFTest extends OpMode {
    LaunchMotors launchMotor = new LaunchMotors();

    @Override
    public void init() {
        launchMotor.init(hardwareMap);
    }

    @Override
    public void loop() {
        launchMotor.setVelocityPIDF(5.0, .02, 0.0, 14.0);
        launchMotor.setLaunchSpeed(-1500);

        telemetry.addData("TPS", launchMotor.getTicksPerSec());
        telemetry.update();

    }
}


