/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.FourDriveMotors;

@Autonomous
public class LimelightOpModePractice extends LinearOpMode {

    Limelight3A limelight;
    FourDriveMotors drive = new FourDriveMotors();

    private double Kp = 0.02;      // Proportional Gain (Tune this first)
    private double Kd = 0.000;     // Derivative Gain (Tune this to reduce overshoot)
    private double minTurnPower = 0.05; // Minimum power to overcome friction

    private double lastError = 0;   // Holds the error from the previous loop iteration
    private double lastTime = 0;    // Holds the time from the previous loop iteration



    @Override
    public void runOpMode() {
        drive.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch((0));    //use pipeline 0 for apriltag detection
        limelight.start();

        waitForStart();

        lastTime = getRuntime(); // Initialize the timer

        //use camera to orient bot for firing
        while(opModeIsActive()){
            LLResult result = limelight.getLatestResult();

            // --------------Validity check---------------
            if (result != null && result.isValid()) {
                double tx = limelight.getLatestResult().getTx(); // Horizontal Offset
                double currentTime = getRuntime();

                // --- 2. PD CALCULATION ---
                double error = tx;
                double deltaTime = currentTime - lastTime;

                // Calculate the rate of change of the error (Derivative)
                double errorChange = (error - lastError);
                double derivative = 0;

                // Avoid division by zero and large initial derivative spike
                if (deltaTime > 0) {
                    derivative = errorChange / deltaTime;
                }

                // Calculate P and D terms
                double pTerm = Kp * error;
                double dTerm = Kd * derivative;

                // Combine terms for the final turn power
                double turnPower = pTerm + dTerm;

                // --- 3. MIN POWER AND DIRECTION ---
                if (Math.abs(turnPower) < minTurnPower && Math.abs(error) > 1.0) {
                    // Only apply min power if error is still large (to start moving)
                    turnPower += Math.copySign(minTurnPower, turnPower);
                } else if (Math.abs(turnPower) > 1.0) {
                    // Cap power at 1.0
                    turnPower = Math.copySign(1.0, turnPower);
                }

                drive.turnUsingCameras(turnPower);

                // --- 4. TELEMETRY & UPDATE STATE ---
                telemetry.addData("Status", "Target Locked");
                telemetry.addData("Error (tx)", "%.2f", tx);
                telemetry.addData("P-Term", "%.3f", pTerm);
                telemetry.addData("D-Term", "%.3f", dTerm);
                telemetry.addData("Total Power", "%.2f", turnPower);

                lastError = error;
                lastTime = currentTime;

                // --- 5. STOP CONDITION ---
                if (Math.abs(tx) < 1.0 && Math.abs(errorChange) < 0.5) {
                    // Stop when centered AND the error is not changing quickly (settled)
                    drive.stopDrive();
                    telemetry.addData("Goal", "Reached & Settled");
                    telemetry.update();
                    break;
                }
                //--------commented out in case I need it in the future-----------
               // telemetry.addData("Camera Status", "Target is locked!");
               // telemetry.addData("X-Offset", tx);
               // double Kp = 0.0125;   //proportional gain
               // double turnPower = tx * Kp; // Proportional control for turning
               // double minTurnPower = 0.025; //needed to overcome friction

                //add on a small amount of power to make sure motors can overcome friction
                //if (Math.abs(turnPower) > 0) {
                  //  turnPower += Math.copySign(minTurnPower, turnPower);
                //}

                // drive.turnUsingCameras(turnPower);

                // Stop when centered
                //if (Math.abs(tx) < 1) {
                  //  drive.stopDrive();
                    //break;
                //}
            } else {
                telemetry.addData("Status", "No Target Detected");
                drive.stopDrive();
            }
            telemetry.update();
        }
    // Code to shoot balls
    }
}*/
