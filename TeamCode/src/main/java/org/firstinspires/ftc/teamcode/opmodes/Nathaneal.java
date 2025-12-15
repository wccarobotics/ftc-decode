package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Nathaneal extends OpMode {
    @Override
    public void init() {
        String myWorld = "the best number is probably ";
        double theNumber = 67;

        telemetry.addData("oh", myWorld+theNumber);
    }

    @Override
    public void loop()
        { double LS = -gamepad1.left_stick_y * 2.0;
            telemetry.addData("Left stick y", gamepad1.left_stick_y);
            telemetry.addData("B Button", gamepad1.b);
            telemetry.addData("X Button", gamepad1.x);
            telemetry.addData("Y Button", gamepad1.y);
            telemetry.addData("A Button", gamepad1.a);
            telemetry.addData("LS*2.0", LS );

    }
}