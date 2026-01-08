/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.DriveMotors;
import org.firstinspires.ftc.teamcode.mechanisms.Gyro;
import org.firstinspires.ftc.teamcode.mechanisms.LaunchMotors;
import org.firstinspires.ftc.teamcode.mechanisms.LaunchServos;

@Autonomous
public class DecodePractice extends OpMode {

    //instantiate motor objects
    Gyro gyro = new Gyro();
    DriveMotors driveMotors = new DriveMotors();
    LaunchMotors launchMotors = new LaunchMotors();
    LaunchServos launchServos = new LaunchServos();


    //init function will run once when the init button is pushed
    @Override
    public void init() {
        //initialize motors
        driveMotors.init(hardwareMap);
        launchMotors.init(hardwareMap);
        launchServos.init(hardwareMap);
        gyro.init(hardwareMap);
    }

    //start function will run once when the start button is pushed
    @Override
    public void start() {

    }

    //loop function runs continuously after the start function has been completed until the end of the OpMode
    @Override
    public void loop() {
    }

    //launchObjects function cycles and launches balls already in the hopper.
    public void launchObjects(double velocity, int numberOfObjects) {
        long currentMillis = System.currentTimeMillis();
        launchMotors.setLaunchSpeed(velocity);
        while ((System.currentTimeMillis() - currentMillis) < 3000) {
            //empty
        }
        for (int i = 0; i < numberOfObjects; i++) {
            currentMillis = System.currentTimeMillis();            //using millis to time motors
            while ((System.currentTimeMillis() - currentMillis) < 200) {    //may need to adjust this delay
                launchServos.setSpeed(velocity);
            }
            currentMillis = System.currentTimeMillis();
            while ((System.currentTimeMillis() - currentMillis) < 500) {   //may need to adjust this delay
                launchServos.setSpeed(0);
            }
        }
        launchMotors.setLaunchSpeed(0);
    }

    //intakeObjects functions activates intake motor, positions servo, and drives bot a short distance into ball
    public void intakeObjects() {
        launchServos.setStopperServo(0);
        launchMotors.setIntakeSpeed(1);
        long currentMillis = System.currentTimeMillis();
        while ((System.currentTimeMillis() - currentMillis) < 200) {
            //driveMotors.setSpeed(500);
        }
       // driveMotors.setSpeed(0);
        launchServos.setStopperServo(90);
    }

    public void turnToHeading(double desiredHeading) {

        double heading = gyro.getHeading();
        while ( heading < desiredHeading - 1 || heading > desiredHeading + 1){
            if (desiredHeading < 0) {
              //  driveMotors.setSpeed(500);
            }
            else {
                //turn right
            }
            heading = gyro.getHeading();
        }
    }
        //function to move vehicle...can I add this function into the drivemotors class
    //function to turn... can I add this function into the drivemotor class
    //later on may need correction
}*/