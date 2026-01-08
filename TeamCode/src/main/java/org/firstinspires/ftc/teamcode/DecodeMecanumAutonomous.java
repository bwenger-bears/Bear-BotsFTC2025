package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.FourDriveMotors;

@Autonomous
public class DecodeMecanumAutonomous extends LinearOpMode {

    FourDriveMotors driveMotors = new FourDriveMotors();

    @Override
    public void runOpMode() {
        //place initialization code up here
        driveMotors.init(hardwareMap);

        waitForStart();
        //code from here down runs after start is pressed
        driveMotors.driveForward(1000, 0.5);
        driveMotors.turn(45, 0.5);

        while (opModeIsActive()) {

        }
    }
}
