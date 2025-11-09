package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Disabled
@Autonomous
public class ModeSwitcher extends OpMode {
    double mode = 0;
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        if (gamepad1.dpad_left) {
            mode -= 1;
        }
        if (gamepad1.dpad_right) {
            mode += 1;
        }
        if (mode < 0) {
            mode = 3;
        }
        if (mode > 3) {
            mode = 0;
        }
        if (mode == 0) {

        }
    }
}
