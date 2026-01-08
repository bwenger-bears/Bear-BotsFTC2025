/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.DriveMotors;
import org.firstinspires.ftc.teamcode.mechanisms.Gyro;
import org.firstinspires.ftc.teamcode.mechanisms.LaunchMotors;
import org.firstinspires.ftc.teamcode.mechanisms.LaunchServos;

@TeleOp
public class DecodeTeleopPractice extends OpMode {

    DriveMotors driveMotors = new DriveMotors();
    LaunchMotors launchMotors = new LaunchMotors();
    LaunchServos launchServos = new LaunchServos();
    Gyro gyro = new Gyro();

    public enum TeleopState {
        TRACTOR_DRIVE,
        INTAKE,
        LAUNCH_BALL,
        DONE
    }

    TeleopState currentState;
    double currentStateTimer;
    double leftStickY;
    double rightStickY;

    @Override
    public void init() {
        driveMotors.init(hardwareMap);
        launchMotors.init(hardwareMap);
        launchServos.init(hardwareMap);
    }

    @Override
    public void start() {
        super.start();
        currentState = TeleopState.TRACTOR_DRIVE;
        currentStateTimer = getRuntime();
        launchServos.setStopperServo(0);
    }

    @Override
    public void loop() {
        switch (currentState) {
            case TRACTOR_DRIVE:
                telemetry.addData("Teleop State", currentState);
                leftStickY = gamepad1.left_stick_y;
                rightStickY = gamepad1.right_stick_y;
               // driveMotors.setSpeedLeft(-1 * leftStickY * 1500);   //may need to find max rotation for these motors
                //driveMotors.setSpeedRight(-1 * rightStickY * 1500);
                if (leftStickY == 0 && rightStickY == 0 && gamepad1.a) {
                    launchServos.setStopperServo(0);
                    currentState = TeleopState.INTAKE;
                }
                if (leftStickY == 0 && rightStickY == 0 && gamepad1.b) {
                    currentState = TeleopState.LAUNCH_BALL;
                    currentStateTimer = getRuntime();
                    launchServos.setStopperServo(0);
                }
                telemetry.addData("Left Stick", leftStickY);
                telemetry.addData("Right Stick", rightStickY);
                telemetry.addData("Button A", gamepad1.a);
                telemetry.addData("Button B", gamepad1.b);
                break;

            case INTAKE:


                break;

            case LAUNCH_BALL:
                launchMotors.setLaunchSpeed(1000);
                if (getRuntime() - currentStateTimer > 2 && getRuntime() - currentStateTimer <= 2.3) {
                    launchServos.setSpeed(0.5);
                } else if (getRuntime() - currentStateTimer > 2.3 && getRuntime() - currentStateTimer <= 4.3) {
                    launchServos.setSpeed(0);
                } else if (getRuntime() - currentStateTimer > 4.3 && getRuntime() - currentStateTimer <= 4.6) {
                    launchServos.setSpeed(0.5);
                } else if (getRuntime() - currentStateTimer > 4.6 && getRuntime() - currentStateTimer <= 6.6) {
                    launchServos.setSpeed(0);
                } else if (getRuntime() - currentStateTimer > 6.6 && getRuntime() - currentStateTimer <= 7) {
                    launchServos.setSpeed(0.5);
                }
                //may want to change this an else if
                if (getRuntime() - currentStateTimer > 7) {
                    launchServos.setSpeed(0);
                    launchMotors.setLaunchSpeed(0);
                    currentState = TeleopState.TRACTOR_DRIVE;
                    //Will need to think through this.
                }
                telemetry.addData("Left Stick", leftStickY);
                telemetry.addData("Right Stick", rightStickY);
                telemetry.addData("Button A", gamepad1.a);
                telemetry.addData("Button B", gamepad1.b);
                break;

            case DONE:
                launchMotors.setLaunchSpeed(0);
               // driveMotors.setSpeedRight(0);
                //driveMotors.setSpeedLeft(0);
                telemetry.addData("Telemetry State", currentState);
                break;
        }
        telemetry.update();
    }
}*/
