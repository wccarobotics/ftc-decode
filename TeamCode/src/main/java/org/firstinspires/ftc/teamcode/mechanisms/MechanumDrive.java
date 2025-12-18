package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MechanumDrive {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    RevIMU IMU = new RevIMU();

    public void init(HardwareMap hardwareMap){
        frontLeftMotor = hardwareMap.get(DcMotor.class, "left_front_drive");
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor = hardwareMap.get(DcMotor.class, "left_back_drive");
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor = hardwareMap.get(DcMotor.class,"right_front_drive");
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor = hardwareMap.get(DcMotor.class, "right_back_drive");
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        IMU.init(hardwareMap);
    }
    public void drive(double forward, double strafe, double turn){
        double frontLeftPower = forward + strafe + turn;
        double backLeftPower = forward - strafe + turn;
        double frontRightPower = forward - strafe - turn;
        double backRightPower = forward + strafe - turn;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftMotor.setPower(maxSpeed * (frontLeftPower/maxPower));
        backLeftMotor.setPower(maxSpeed * (backLeftPower/maxPower));
        frontRightMotor.setPower(maxSpeed * (frontRightPower/maxPower));
        backRightMotor.setPower(maxSpeed * (backRightPower/maxPower));

    }
    public void driveFieldRelative(double forward, double strafe, double turn){
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta - IMU.heading(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward, newStrafe, turn);
    }
}
