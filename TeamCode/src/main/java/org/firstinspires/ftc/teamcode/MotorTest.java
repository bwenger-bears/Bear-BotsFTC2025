/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.mechanisms.LaunchMotors;
import org.firstinspires.ftc.teamcode.mechanisms.LaunchServos;

@Autonomous
public class MotorTest extends OpMode {

    //instantiate launchMotors object
    LaunchMotors launchMotors = new LaunchMotors();
    LaunchServos launchServos = new LaunchServos();
    //declare MaxMotorSpeed variable to hold return from getTickPerSec() method
    double maxMotorSpeed;
    //add values to the following variables hen ready to determine PIDF values
    double testSpeed = 1500;
    double kp = 8;
    double ki = 0.02;
    double kd = 0.0;
    double kf = 14;

    double currentStateTimer;
    //might need variable for 'state'

    public enum RobotState {
        STATE_1_MOVE_FWD,
        STATE_2_LAUNCH,
        STATE_3_ROTATE,
        STATE_4_MOVE_FWD,
        STATE_5_INTAKE,
        STATE_6_DONE
    }

    RobotState currentState;


    @Override
    public void init() {
        launchMotors.init(hardwareMap);
        launchServos.init(hardwareMap);
        //declare and initialize runTime variable so that motor only runs 10 sec
        double runTime = getRuntime();
        //Uncomment the following 3 lines to change, test and display PIDF coefficients
        launchMotors.setVelocityPIDF(kp, ki, kd, kf);
        launchMotors.setLaunchSpeed(testSpeed);
        //displayCurrentPIDFCoefficients();

        //after determining max velocity for motor, comment out the following line
        //launchMotors.setLaunchPower(1);

        //check for Max TPS and display to telemetry for next 10 seconds
        while ((getRuntime() - runTime) < 1) {
            maxMotorSpeed = launchMotors.getTicksPerSec();
            telemetry.addData("Max Velocity (TPS)", maxMotorSpeed);
            telemetry.update();
            //comment out the telemetry lines when testing PIDF values or they will quickly disappear
        }
        //launchMotors.setLaunchPower(0);
        //launchMotors.setLaunchSpeed(0);
    }

    @Override
    public void start() {
        super.start();
        currentState = RobotState.STATE_2_LAUNCH;
        launchServos.setStopperServo(0);
        currentStateTimer = getRuntime();
    }

    @Override
    public void loop() {
        telemetry.addData("RobotState", currentState);
        switch (currentState) {
            case STATE_2_LAUNCH:
                telemetry.addData("RunState", RobotState.STATE_2_LAUNCH);
                launchMotors.setLaunchSpeed(1100);
                if ((getRuntime() - currentStateTimer) > 2 && (getRuntime() - currentStateTimer) <= 2.3) {
                    launchServos.setSpeed(0.5);
                } else if ((getRuntime() - currentStateTimer) > 2.3 && (getRuntime() - currentStateTimer) <= 4.3) {
                    launchServos.setSpeed(0);
                } else if ((getRuntime() - currentStateTimer) > 4.3 && (getRuntime() - currentStateTimer) <= 4.6) {
                    launchServos.setSpeed(0.5);
                } else if ((getRuntime() - currentStateTimer) > 4.6 && (getRuntime() - currentStateTimer) <= 7.0) {
                    launchServos.setSpeed(0);
                } else if ((getRuntime() - currentStateTimer) > 7.0 && (getRuntime() - currentStateTimer) <= 8.0) {
                    launchServos.setSpeed(0.5);
                }
                //may want to change this an else if
                if ((getRuntime() - currentStateTimer) > 8.0) {
                    launchServos.setSpeed(0);
                    launchServos.setStopperServo(200);
                    launchMotors.setLaunchSpeed(0);
                    currentState = RobotState.STATE_6_DONE;
                    //set up for next state
                    //Will need to think through this.
                }

            case STATE_6_DONE:
                telemetry.addData("RobotState", "All done!");
                break;
        }

    }
        public void displayCurrentPIDFCoefficients () {
            //create PIDFCoefficients object to hold the constants
            PIDFCoefficients currentCoefficients = launchMotors.getVelocityPIDF();

            //extract each constant from object and assign it to a variable
            double pValue = currentCoefficients.p;
            double iValue = currentCoefficients.i;
            double dValue = currentCoefficients.d;
            double fValue = currentCoefficients.f;

            //send to telemetry screen
            telemetry.addData("Kp Value", pValue);
            telemetry.addData("Ki Value", iValue);
            telemetry.addData("Kd Value", dValue);
            telemetry.addData("Kf Value", fValue);
            telemetry.update();
        }


}*/
