package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.ProgrammingBoard;

@Disabled
@TeleOp
public class MotorTest extends OpMode{
    ProgrammingBoard board = new ProgrammingBoard();
    double left_stick = 0;
    @Override
    public void init() {
        board.init(hardwareMap);
    }
    @Override
    public void loop() {
        if(board.getTouchSensorState()){
            telemetry.addData("state","pressed");
        }
        else{
            telemetry.addData("state","not pressed");
        }
        left_stick = gamepad1.left_stick_x;
        if (left_stick < 0){
            left_stick *= -left_stick;
        }
        else {
            left_stick *= left_stick;
        }
        board.setMotorSpeed(left_stick);
    }
}
