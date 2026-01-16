package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class MacanumFieldOrientatedOpModeAB extends OpMode{
    MecanumDriveAB drive = new MecanumDriveAB();

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
        if(gamepad1.left_trigger > 0.0){
            drive.setVelocityPIDF(10.0,3.0,0.00001,12.0);
            drive.setMotorVelocity(gamepad1.left_trigger);
            telemetry.addData("Front Right: ", drive.getFrontRightVelocity());
            telemetry.addData("Back Right: ", drive.getBackRightVelocity());
            telemetry.addData("Front Left: ", drive.getFrontLeftVelocity());
            telemetry.addData("Back Left: ", drive.getBackLeftVelocity());

        }
        if(gamepad2.right_bumper){
            // PIDF must be modified to stop overshooting and undershooting
            //  Current Best: drive.setVelocityPIDF(4.8,0.02,0.00001,14.0);
             drive.setVelocityPIDF(4.8,0.02,0.000015,14.0);

            drive.setSpinnerVelocity(-1650.0);
            drive.setStopperPosition(90);
            telemetry.addData("Spinner: ", drive.getSpinnerVelocity());


        } else {
            drive.setSpinnerVelocity(0);
            drive.setStopperPosition(0);
        }
        if(gamepad2.x){
       drive.setIntake(1500.0);
        } else if (gamepad2.y) {
         drive.setIntake(-1500.0);
        }
        else {
         drive.setIntake(0);
        }
        if(gamepad1.a){
            drive.resetIMU();
        }
    }
}
