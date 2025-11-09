package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp
public class TankDriveButtonSpeed extends OpMode {
    private DcMotor left_motor;
    private DcMotor right_motor;
    double speed = 0;
    Boolean lastA = false;
    Boolean lastB = false;

    @Override
    public void init() {

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
        double turn = gamepad1.right_trigger - gamepad1.left_trigger;
        if (gamepad1.a && !lastA){
            speed -= 0.1;
            lastA = true;
        }

        if (!gamepad1.a){
            lastA = false;
        }

        if (gamepad1.b && !lastB){
            speed += 0.1;
            lastB = true;
        }

        if (!gamepad1.b){
            lastB = false;
        }

        if (gamepad1.x){
            speed = 0;
        }
        left_motor.setPower(speed - (Math.abs(speed) * squareInputWithSign(turn)));
        right_motor.setPower(speed + (Math.abs(speed) * squareInputWithSign(turn)));
    }
}
