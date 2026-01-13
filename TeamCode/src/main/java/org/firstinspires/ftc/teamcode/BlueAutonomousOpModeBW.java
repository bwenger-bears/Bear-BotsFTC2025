package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.FourDriveMotors;
import org.firstinspires.ftc.teamcode.mechanisms.LaunchMotors;
import org.firstinspires.ftc.teamcode.mechanisms.LaunchServos;

@Autonomous
public class BlueAutonomousOpModeBW extends LinearOpMode {

    FourDriveMotors drive = new FourDriveMotors();
    LaunchMotors launchMotors = new LaunchMotors();
    LaunchServos launchServos = new LaunchServos();
    Limelight3A limelight;

    private double Kp = 0.02;   //proportional constant for turn control
    private double minTurnPower = 0.05; //overcomes friction
    private double lastTime = 0;
    private double tx;          //degree error from target object
    private double lastTx = 0;


    public void runOpMode() {
        drive.init(hardwareMap);
        launchMotors.init(hardwareMap);

        launchServos.init(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);    //pipeline 0 = blue, 1 = red, 8 = purple ball, 9 = green ball
        limelight.start();

        waitForStart();

        //------------autonomous action items-add code below the comment-----------------------------

        // 1. Move to launch point
        drive.driveForward(1000, 0.5);


        // 2. turn using camera to line up launch


        // 3. Spin up launch motors and launch 3 balls then change camera pipeline 0



        // 4. TurnUsingCamara and line up closet green ball
        turnUsingCamera(1);//pipeline 1= purple ball


        // 5. Spin up intake and then drive into balls



        // 6. Switch to camera pipeline 9 and drive backwards to launch zone and then turn to face target


        // 7.  Spin up launch motor and launch at target



        // 8.  Time permitting, lather, rinse, and repeat


        // DON'T FORGET TO PUSH TO GITHUB!!!!!
    }

    //-------------Helper Methods--------------------
    //delivers and fires balls in launcher mechanism
    public void launch() {
        launchMotors.setIntakeSpeed(1000);
        launchMotors.setVelocityPIDF(5.0, .02, 0.0, 14.0);
        for (int i = 0; i<3; i++) {
            launchMotors.setLaunchSpeed(1700);
            sleep(1000);
            launchServos.setStopperServo(20);
            sleep(500);
            launchServos.setStopperServo(90);
        }
        launchMotors.setIntakeSpeed(0);
        launchMotors.setLaunchPower(0);
    }

    public void intake() {
        launchMotors.setIntakeSpeed(1000);
        sleep(5000);
    }
    //acquire and turn towards balls or april tags
    public void turnUsingCamera (int pipeline) {
        lastTx = 0;
        limelight.pipelineSwitch(pipeline); //pipeline 0 = blue base, 1 = red base, 8 = purple ball, 9 = green ball
        sleep(2000);
        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

        // --------------Validity check---------------
            if (result != null && result.isValid()) {
                tx = limelight.getLatestResult().getTx(); // Horizontal Offset

            // Calculate power based on Proportional Gain
                double turnPower = Kp * tx;

            // --- 3. MIN POWER AND DIRECTION ---
                if (Math.abs(turnPower) < minTurnPower && Math.abs(tx) > 1.0) {
                // Only apply min power if error is still large (to start moving)
                    turnPower += Math.copySign(minTurnPower, turnPower);
                } else if (Math.abs(turnPower) > 1.0) {
                // Cap power at 1.0
                    turnPower = Math.copySign(1.0, turnPower);
                }
            //pass turnPower to to our drive motors.
                drive.cameraTurn(turnPower);

            // --- 4. TELEMETRY & UPDATE STATE ---
                telemetry.addData("Status", "Target Locked");
                telemetry.addData("Error (tx)", "%.2f", tx);
                telemetry.addData("Total Power", "%.2f", turnPower);

            // --- 5. STOP CONDITION ---
                if (Math.abs(tx) < 1.0 && Math.abs(tx - lastTx) < 0.5) {
                // Stop when centered AND the error is not changing quickly (settled)
                    drive.stopDrive();
                    telemetry.addData("Goal", "Reached & Settled");
                    telemetry.update();
                    break;
                }
            } else {
                telemetry.addData("Status", "No Target Detected");
                drive.stopDrive();
            }
            lastTx = tx;
            telemetry.update();
        }
    }
}
