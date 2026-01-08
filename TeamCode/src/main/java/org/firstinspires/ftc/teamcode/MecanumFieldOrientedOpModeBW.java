/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.LaunchMotors;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp
public class MecanumFieldOrientedOpModeBW extends OpMode {

    MecanumDrive drive = new MecanumDrive();
    LaunchMotors launchMotors = new LaunchMotors();

    double forward, strafe, rotate;

    @Override
    public void init() {
        drive.init(hardwareMap);
        launchMotors.init(hardwareMap);
        drive.resetGyro();
    }

    @Override
    public void loop() {

        //assign gamepad values to variables
        forward = -gamepad1.left_stick_y;
        strafe = -gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        telemetry.addData("Left Stick Y", forward);
        telemetry.addData("Left stick X", strafe);
        telemetry.addData("Right Stick X", rotate);
        telemetry.addData("Current Heading", drive.getHeading());

        //drive.setMotors();
        drive.driveFieldRelative(forward, strafe, rotate);

        if (gamepad1.a) {
            launchMotors.setLaunchSpeed(1900);
            gamepad1.rumble(1000);
        }
        else {
            launchMotors.setLaunchSpeed(0);
        }
    }
}*/
