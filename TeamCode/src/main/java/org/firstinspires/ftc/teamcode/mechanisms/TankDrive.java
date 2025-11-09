package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TankDrive {
    private DcMotorEx left_motor;
    private DcMotor right_motor;
    private double leftTicksPerRotation;
    public double rightTicksPerRotation;
    private double gearRatio;
    private double opModeDistance;
    public double initialDistance;
    public boolean haveWeSetInitialDistance;
    public boolean areWeNearlyThereYet;

    public void init(HardwareMap hardwareMap) {
        left_motor = hardwareMap.get(DcMotorEx.class, "left_motor");
        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftTicksPerRotation = 537.7;

        right_motor = hardwareMap.get(DcMotor.class, "right_motor");
        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightTicksPerRotation = 537.7;

        gearRatio = 19.2;
        opModeDistance = 0;
        haveWeSetInitialDistance = false;
        areWeNearlyThereYet = false;

    }
    double squareInputWithSign(double input){
        double output = input * input;
        if(input < 0){
            output *= -1;
        }
        return output;
    }

    public void setSpeed(double leftPower, double rightPower) {
        double largest = 1.0;
        largest = Math.max(largest, Math.abs(leftPower));
        largest = Math.max(largest, Math.abs(rightPower));

        left_motor.setPower(leftPower / largest);
        right_motor.setPower(rightPower / largest);
    }

    public void drive(double speed, double turn) {
        setSpeed(speed + turn, speed - turn);
    }

    public double distance() {
        return (((left_motor.getCurrentPosition() / leftTicksPerRotation) + (right_motor.getCurrentPosition() / rightTicksPerRotation)) / 2) * (Math.PI * 3.77952756);
    }
    public void driveForDistance(double tarDistance, double speeed){
        if (!haveWeSetInitialDistance){
            initialDistance = distance();
            haveWeSetInitialDistance = true;
            areWeNearlyThereYet = false;
        }
        if (tarDistance < 0){
            speeed = -Math.abs(speeed);
        }
        if (tarDistance - (distance() - initialDistance) > 0 && tarDistance > 0){
            drive(speeed, 0);
        }
        else if(tarDistance - (distance() - initialDistance) < 0 && tarDistance < 0){
            drive(speeed, 0);
        }
        else {
            haveWeSetInitialDistance = false;
            areWeNearlyThereYet = true;
        }
    }
}
