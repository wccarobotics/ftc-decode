package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp()
public class Leslytest extends OpMode  {
    @Override
    public void init () {
        telemetry.addData("Hello","world");
        telemetry.addData("I like robotics","world");
    }

    @Override
    public void loop() {

    }
}
