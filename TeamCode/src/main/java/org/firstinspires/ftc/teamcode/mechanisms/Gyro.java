package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Gyro {

    private IMU gyro;

    public void init(HardwareMap hdwMap) {
        //map imu to control hub
        gyro = hdwMap.get(IMU.class, "imu");
        //Instantiate objects to hold LogoFacingDirection and UsbFacingDirection
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
        //Instantiate an object to combine both orientations
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        //instantiate Parameters object that is needed for the initialize() method
        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);
        //initialize the IMU
        gyro.initialize(parameters);
        //reset gyro to direction facing currently
        gyro.resetYaw();
    }

    public double getHeading() {
        // Yaw is the heading (rotation about the Z-axis)
        return gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}