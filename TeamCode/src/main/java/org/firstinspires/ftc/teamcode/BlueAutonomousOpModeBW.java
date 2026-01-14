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

    private double Kp = 0.1;   //proportional constant for turn control
    private double minTurnPower = 0.06; //overcomes friction
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

        //drive.turn(90,0.05);
        //------------autonomous action items-add code below the comment-----------------------------

        // 1. Move to launch point
        drive.driveForward(2400, 0.5);


        // 2. turn using camera to line up launch


        // 3. Spin up launch motors and launch 3 balls
        //launch();


        // 4. TurnUsingCamara and line up closet green ball
        //turnUsingCamera(8);//pipeline 8= purple ball
        drive.turn(-30,0.5);

        // 5. Spin up intake and then drive into balls
        //intake():
        drive.driveForward(1000,0.5);

        // 6. drive backwards to launch zone and then turn to face target (pipeline 0)
        drive.driveForward(-1000,-0.5);
        //turnUsingCamera(0);


        // 7.  Spin up launch motor and launch at target
        //launch();


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

            limelight.pipelineSwitch(pipeline);
            sleep(500); // Reduced sleep; 2 seconds is a long time in Auto!

            int settledFrames = 0; // Tracks how long we've been on target

            while (opModeIsActive()) {
                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {
                    tx = result.getTx(); // Horizontal Offset

                    // 1. Calculate base Proportional power
                    double turnPower = tx * Kp;

                    // 2. Apply Minimum Power to overcome friction
                    // Only apply if we aren't "close enough" yet
                    if (Math.abs(tx) > 1.0) {
                        if (Math.abs(turnPower) < minTurnPower) {
                            turnPower = Math.copySign(minTurnPower, turnPower);
                        }
                    } else {
                        turnPower = 0; // Close enough, stop motors
                    }

                    // 3. Cap max power for safety/stability
                    turnPower = Math.max(-0.4, Math.min(0.4, turnPower));

                    drive.cameraTurn(turnPower);

                    // 4. STOP CONDITION: Must be within 1 degree for 5 consecutive frames
                    if (Math.abs(tx) < 1.0) {
                        settledFrames++;
                    } else {
                        settledFrames = 0;
                    }

                    if (settledFrames > 5) {
                        drive.stopDrive();
                        break;
                    }

                    telemetry.addData("Target", "Locked");
                    telemetry.addData("TX", tx);
                    telemetry.update();

                } else {
                    // Target lost: Stop motors so we don't spin wildly
                    drive.stopDrive();
                    telemetry.addData("Target", "LOST");
                    telemetry.update();
                }
            }
        }
    }
