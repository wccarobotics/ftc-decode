/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */


package org.firstinspires.ftc.teamcode.opmodes;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.MechanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.PinpointOdometry;
import org.firstinspires.ftc.teamcode.mechanisms.ScoringRI3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsDrawing;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® Robot in 3 Days for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™!
 */

@TeleOp(name = "DECODE Ri3D", group = "StarterBot")
//@Disabled
public class RobotIn3Days extends OpMode {
    // Declare OpMode members.
    MechanumDrive mechanumDrive = new MechanumDrive();
    ScoringRI3D scoring = new ScoringRI3D();
    private Follower follower;
    private TelemetryManager telemetryM;

    private enum Alliance{
        BLUE,
        RED
    }
    private Alliance currentAlliance = Alliance.BLUE;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72,0));
        follower.update();
        follower.startTeleopDrive();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        PanelsDrawing.init();
        scoring.init(hardwareMap, telemetry);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        if (gamepad1.yWasPressed()){
            if (currentAlliance == Alliance.BLUE){
                currentAlliance = Alliance.RED;
            }
            else {
                currentAlliance = Alliance.BLUE;
            }
        }
        telemetry.addData("ALLIANCE", currentAlliance);
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

            double forward = mechanumDrive.squareInputWithSign(-gamepad1.left_stick_y);
            double strafe = mechanumDrive.squareInputWithSign(-gamepad1.left_stick_x);
            double turn = mechanumDrive.squareInputWithSign(-(gamepad1.right_trigger - gamepad1.left_trigger));
            double offsetHeading = currentAlliance == Alliance.BLUE? Math.toRadians(180): 0;
            follower.update();
            telemetryM.update();

            PanelsDrawing.drawRobot(follower.getPose());
            PanelsDrawing.sendPacket();

            double targetX = 0;
            double targetY = 135;

            if (currentAlliance == Alliance.RED)
            {
                targetX = 144 - targetX;
            }

            double kP = 1/Math.toRadians(75);

            Pose currentPose = follower.getPose();
            double targetHeading = Math.atan2(targetY - currentPose.getY(), targetX - currentPose.getX());

            if (gamepad1.right_stick_button){
                double error = targetHeading - currentPose.getHeading();
                if (error > Math.PI){
                    error -= 2 * Math.PI;
                }
                turn = error * kP;
            }

            follower.setTeleOpDrive(forward, strafe, turn, false, offsetHeading);


            telemetry.addData("heading", Math.toDegrees(follower.getHeading()));
            telemetry.addData("Xpose",follower.getPose().getX());
            telemetry.addData("Ypose",follower.getPose().getY());
            telemetry.addData("goal heading", Math.toDegrees(targetHeading));

        /*
         * Here we give the user control of the speed of the launcher motor without automatically
         * queuing a shot.
         */
        if (gamepad1.y) {
            scoring.spinLauncher();
        } else if (gamepad1.b) { // stop flywheel
            scoring.stopLauncher();
        }

        if (gamepad1.dpadDownWasPressed()) {
            scoring.switchDiverter();
        }
        if (gamepad1.dpadLeftWasPressed()) {
            scoring.changeDiverter(-0.05);
        }
        if (gamepad1.dpadRightWasPressed()) {
            scoring.changeDiverter(0.05);
        }

        if (gamepad1.aWasPressed()){
            scoring.runIntake();
        }
        if (gamepad1.xWasPressed()){
            scoring.switchIntake();
        }

        if (gamepad1.dpadUpWasPressed()) {
            scoring.setLaunchDistance();
        }
        /*
         * Now we call our "Launch" function.
         */
        if(gamepad1.leftBumperWasPressed()){
            scoring.shootLeft();
        }
        if (gamepad1.rightBumperWasPressed()){
            scoring.shootRight();
        }

        scoring.updateAll();

        /*
         * Show the state and motor powers
         */
        telemetry.addData("Left State", scoring.getLeftLaunchState());
        telemetry.addData("Right State", scoring.getRightLaunchState());
        telemetry.addData("launch distance", scoring.getLauncherDistance());
        telemetry.addData("Left Launcher Velocity", scoring.flyWheelSpeed());
        telemetry.addData("Diverter position", scoring.diverterPose());
        telemetry.addData("Intake state", scoring.intakeSpeed());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}