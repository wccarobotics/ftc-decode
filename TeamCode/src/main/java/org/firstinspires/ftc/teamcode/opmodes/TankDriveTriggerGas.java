package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp
public class TankDriveTriggerGas extends OpMode {
    private DcMotor left_motor;
    private DcMotor right_motor;

    double left_power;
    boolean last_a;
    boolean last_b;

    @Override
    public void init() {
        left_power = 0;
        last_a = false;
        last_b = false;

        left_motor = hardwareMap.get(DcMotor.class, "left_motor");
        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        right_motor = hardwareMap.get(DcMotor.class, "right_motor");
        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_motor.setDirection(DcMotorSimple.Direction.REVERSE);
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
        double gas = gamepad1.right_trigger - gamepad1.left_trigger;
        left_motor.setPower(squareInputWithSign(gas) - gamepad1.left_stick_x);
        right_motor.setPower(squareInputWithSign(gas) + gamepad1.left_stick_x);
    }
}
