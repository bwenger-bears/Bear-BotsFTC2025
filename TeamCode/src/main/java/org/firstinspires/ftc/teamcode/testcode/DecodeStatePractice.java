/*package org.firstinspires.ftc.teamcode;

import android.graphics.SweepGradient;
import android.widget.Switch;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.teamcode.mechanisms.DriveMotors;
import org.firstinspires.ftc.teamcode.mechanisms.Gyro;
import org.firstinspires.ftc.teamcode.mechanisms.LaunchMotors;
import org.firstinspires.ftc.teamcode.mechanisms.LaunchServos;

@Autonomous
public class DecodeStatePractice extends OpMode {
    //instantiate objects, declare variables, and setup enums
    DriveMotors driveMotors = new DriveMotors();
    LaunchMotors launchMotors = new LaunchMotors();
    LaunchServos launchServos = new LaunchServos();
    Gyro gyro = new Gyro();
    double currentStateTimer;
    //might need variable for 'state'

    public enum RobotState {
        MOVE_FWD_1,
        LAUNCH,
        ROTATE,
        MOVE_FWD,
        INTAKE,
        DONE
    }
    RobotState currentState;

    @Override
    public void init() {
        //initialize and map out hardware objects
        driveMotors.init(hardwareMap);
        launchMotors.init(hardwareMap);
        launchServos.init(hardwareMap);
        gyro.init(hardwareMap);
    }

    @Override
    public void start() {
        super.start();          //this is needed to execute some of the essential methods from OpMode start()
        //set up for STATE_1
        currentState = RobotState.MOVE_FWD;
     //   driveMotors.setPosition(1000);
    }

    @Override
    public void loop() {
        telemetry.addData("RobotState", currentState);
        switch (currentState) {
            case MOVE_FWD:
          //      driveMotors.setSpeed(500);
           //     if (!driveMotors.getDriveMotorsIsBusy()){
                    currentState = RobotState.LAUNCH;
                    currentStateTimer = getRuntime();
                    //stop the motors...maybe
                    //set up for STATE_2
                }
                //break;

          //  case LAUNCH:
                launchMotors.setLaunchSpeed(1000);
                if (getRuntime() - currentStateTimer > 2 && getRuntime() - currentStateTimer <= 2.3) {
                    launchServos.setSpeed(0.5);
                } else if (getRuntime() - currentStateTimer > 2.3 && getRuntime() - currentStateTimer <= 4.3) {
                    launchServos.setSpeed(0);
                }else if (getRuntime() - currentStateTimer > 4.3 && getRuntime() - currentStateTimer <= 4.6) {
                    launchServos.setSpeed(0.5);
                }else if (getRuntime() - currentStateTimer > 4.6 && getRuntime() - currentStateTimer <= 6.6) {
                    launchServos.setSpeed(0);
                }else if (getRuntime() - currentStateTimer > 6.6 && getRuntime() - currentStateTimer <= 7) {
                    launchServos.setSpeed(0.5);
                }
                //may want to change this an else if
                if (getRuntime() - currentStateTimer > 7) {
                    launchMotors.setLaunchSpeed(0);
                    currentState = RobotState.DONE;
                    //set up for next state
                    //Will need to think through this.
                }

           // case DONE:
                telemetry.addData("RobotState", "All done!");
                //break;
        }
        //STATE_1_MOVE_FWD
        //STATE_2_LAUNCH
        //STATE_3_ROTATE
        //STATE_4_MOVE_FWD
        //STATE_5_INTAKE
        //STATE_6_ROTATE
        //STATE_7_LAUNCH
        //STATE_8_DONE

    }*/

