package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Nathaneal extends OpMode {
    @Override
    public void init() {
        String myWorld = "World";

        telemetry.addData("Hello", myWorld);
    }

    @Override
    public void loop() {


    }
}