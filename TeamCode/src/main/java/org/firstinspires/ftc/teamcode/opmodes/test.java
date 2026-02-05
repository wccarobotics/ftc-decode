package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@Disabled
public class test extends OpMode{
    private DcMotorEx launcher;
    double power;
    @Override
    public void init(){
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        power = 500;
    }
    @Override
    public void loop(){
        if (gamepad1.dpadUpWasPressed()){
            power += 50;
        }
        else if (gamepad1.dpadDownWasPressed()) {
            power -= 50;
        }
        else if (gamepad1.b) {
            power = 0;
        }
        else if (gamepad1.a)
        {
            power = 500;
        }
        launcher.setVelocity(power / 60 * 28);
        telemetry.addData("Target RPM", power);
        telemetry.addData("RPM ", (launcher.getVelocity()) * 60 / 28);
    }
}
