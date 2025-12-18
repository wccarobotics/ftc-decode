package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.mechanisms.MechanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.RevIMU;

@TeleOp
public class MechanumTriggerTurn extends OpMode {
    MechanumDrive mechanumDrive = new MechanumDrive();
    RevIMU IMU = new RevIMU();
    public void init(){
        mechanumDrive.init(hardwareMap);
        IMU.init(hardwareMap);
    }
    public void loop(){
        mechanumDrive.driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x,(gamepad1.right_trigger-gamepad1.left_trigger));
        if(gamepad1.rightBumperWasPressed()){
            IMU.reset();
        }
    }
}
