package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp()
public class HelloWorldOpMode extends OpMode
{
    ElapsedTime loopTimer = new ElapsedTime();

    int loopCount = 0;
    @Override
    public void init() {
        loopCount = 0;
        telemetry.addLine("init");
        loopTimer.reset();
    }

    @Override
    public void loop() {
        loopCount++;
        telemetry.addData("loop count", loopCount);
        telemetry.addData("Loop time", loopTimer.milliseconds());
        telemetry.addData("loops per second", 1000.0 * loopCount / loopTimer.milliseconds());
        //loopTimer.reset();
    }
}
