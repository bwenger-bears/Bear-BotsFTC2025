package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp
public class MecanumFieldOrientedOpMode extends OpMode{
    MecanumDrive drive = new MecanumDrive();

    Limelight3A limelight;

    private double Kp = 0.02;
    private double Kd = 0.000;

    private double minTurnPower = 0.05;

    private double lastError = 0.0;

    private double lastTime = 0.0;
    double forward, strafe, rotate;

    @Override
    public void init() {
        drive.init(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch((0));
        limelight.start();
        lastTime = getRuntime();


    }

    @Override
    public void loop() {
        forward = gamepad1.left_stick_y;
        strafe = -1 * gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        drive.driveFieldRelative(forward, strafe, rotate);
        if(gamepad1.square){
            drive.raiseLift();
        }
        if(gamepad1.circle){
            drive.lowerLift();

        }

        if(gamepad1.triangle){
            LLResult result = limelight.getLatestResult();
            if(result != null && result.isValid()){
                double tx = limelight.getLatestResult().getTx();

                double currentTime = getRuntime();

                double error = tx;

                double deltaTime = currentTime - lastTime;

                double errorCharge = (error - lastError);

                double deriative = 0;
                if(deltaTime > 0){
                    deriative = errorCharge / deltaTime;
                }

                double pTerm = Kp * error;
                double dTerm = Kd * deriative;

                double turnPower = pTerm + dTerm;

                if(Math.abs(turnPower) < minTurnPower && Math.abs(error) > 1.0){
                    turnPower += Math.copySign(minTurnPower, turnPower);
                } else if (Math.abs(turnPower) > 1.0) {
                    turnPower = Math.copySign(1.0, turnPower);
                }
                // drive.turnUsingCameras(turnPower);

                lastError = error;

            }
        }
        if (gamepad2.left_bumper) {
            drive.turnIntakeOn(1);
        }
        if (!gamepad2.left_bumper) {
            drive.turnIntakeOff();
        }
        if (gamepad2.right_bumper) {
            drive.setStopper(20);
            drive.setSpinner(-1400);
        }
        if (!gamepad2.right_bumper) {
            drive.setStopper(0);
            drive.setSpinner(0);
        }

    }
}


