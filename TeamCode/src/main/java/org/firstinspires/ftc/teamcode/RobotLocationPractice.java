/*package org.firstinspires.ftc.teamcode;


public class RobotLocationPractice {

    double angle;
    double x;
    double y;

    //constructor
    public RobotLocationPractice(double angle) {
        this.angle = angle;
    }

    //Converts all angles to a heading between -180 (left) and +180 (right) degree.
    public double getHeading() {
        double angle = this.angle;
        while (angle > 180) {
            angle -= 360;
        }
        while (angle <= -180) {
            angle += 360;
        }
        return angle;
    }

    //This is my setter function so I can set the heading that I want the robot to maintain.
    public void turnRobot(double angleChange) {
        angle += angleChange;
    }

    // setter function for this.angle
    public void setAngle(double angle) {
        this.angle = angle;
    }

    //getter function for this.angle
    public double getAngle() {
        return this.angle;
    }

    public double changeX(double changeAmount) {
        x += changeAmount;
        return x;
    }
    public void setX(double x) {
        this.x = x;
    }

    //getter function for this.angle
    public double getX() {
        return this.x;
    }
    public double changeY(double changeAmount) {
        y += changeAmount;
        return y;
    }
    public void setY(double y) {
        this.y = y;
    }

    //getter function for this.angle
    public double getY() {
        return this.y;
    }
}*/