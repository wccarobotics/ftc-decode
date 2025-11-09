package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.RevIMU;
import org.firstinspires.ftc.teamcode.mechanisms.TankDrive;

@Disabled
@TeleOp
public class TankDriveOneStick extends OpMode {

    TankDrive tankDrive = new TankDrive();
    RevIMU IMU = new RevIMU();

    @Override
    public void init() {
        tankDrive.init(hardwareMap);
        IMU.init(hardwareMap);
    }
    double squareInputWithSign(double input){
        double output = input * input;
            if(input < 0){
                output *= -1;
            }
        return output;
    }

    @Override
    public void loop() {
        tankDrive.drive(-gamepad1.left_stick_y, .5 * gamepad1.left_stick_x);
        telemetry.addData("rotations", tankDrive.distance());
        telemetry.addData("heading", IMU.heading(AngleUnit.DEGREES));
    }
}
