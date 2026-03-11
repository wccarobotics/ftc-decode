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

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.LimelightVision;
import org.firstinspires.ftc.teamcode.mechanisms.MechanumDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsDrawing;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® Robot in 3 Days for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™!
 */

@TeleOp(name = "DECODE Ri3D", group = "StarterBot")
//@Disabled
public class JeffTeleOp extends JeffBase {
    // Declare OpMode members.

    double lastTime = 0;
    double lastError = 0;
    double curTime;

    private boolean hasBalls = false;
    private boolean hadBalls = false;
    MechanumDrive mechanumDrive = new MechanumDrive();

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        super.start();

        follower.startTeleopDrive();
        resetRuntime();
        curTime = getRuntime();
        scoring.closeLaunch();

    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
            super.loop();

            double forward = mechanumDrive.squareInputWithSign(-gamepad1.left_stick_y);
            double strafe = mechanumDrive.squareInputWithSign(-gamepad1.left_stick_x);
            double turn = mechanumDrive.squareInputWithSign(-(gamepad1.right_trigger - gamepad1.left_trigger));
            double offsetHeading = currentAlliance == Alliance.BLUE? Math.toRadians(180): 0;
            follower.update();
            telemetryM.update();

            PanelsDrawing.drawRobot(follower.getPose());

            vision.update();
            Pose limelightPose = vision.getLatestPose2(LimelightVision.HeadingSource.VISION);
            //Pose limelightPose = vision.getLatestPose();
            if (limelightPose != null) {
                PanelsDrawing.drawRobot(limelightPose, PanelsDrawing.limelightLook);
                telemetry.addData("camera", limelightPose.toString());
            }

            PanelsDrawing.sendPacket();

            double targetX = 0;
            double targetY = 135;

            if (currentAlliance == Alliance.RED)
            {
                targetX = 144 - targetX;
            }


            // auto aim
            Pose currentPose = follower.getPose();
            double targetHeading = Math.atan2(targetY - currentPose.getY(), targetX - currentPose.getX());

            double error = targetHeading - currentPose.getHeading();


            if (gamepad1.right_stick_button){
                if (error > Math.PI){
                    error -= 2 * Math.PI;
                }
                double pTerm = error * JeffConfig.AimConfig.kP;

                double curTime = getRuntime();
                double dT = curTime - lastTime;
                double dTerm;

                if (dT == 0) {
                    dTerm = 0;
                }
                else {
                    dTerm = ((error - lastError) / dT) * JeffConfig.AimConfig.kD;
                }
                turn = pTerm + dTerm;

                lastError = error;
                lastTime = curTime;
            }
            else {
                lastTime = getRuntime();
                lastError = 0;
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
        } else if (gamepad1.b) {
            scoring.stopLauncher();
        }

        // diverter logic
        if (gamepad1.dpadDownWasPressed()) {
            scoring.switchDiverter();
        }
        if (gamepad1.dpadLeftWasPressed()) {
            scoring.changeDiverter(-0.05);
        }
        if (gamepad1.dpadRightWasPressed()) {
            scoring.changeDiverter(0.05);
        }

        // intake
        if (gamepad1.aWasPressed()){
            scoring.runIntake();
        }
        if (gamepad1.xWasPressed()){
            scoring.switchIntake();
        }

        if (gamepad1.dpadUpWasPressed()) { // launch distance
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

        // Auto diverter switching
        if (scoring.rightBall() || scoring.leftBall()) {
            hasBalls = true;
            if (!hadBalls && (scoring.rightBall() ^ scoring.leftBall()) && (scoring.intakeSpeed() > 0)) {
                if (scoring.rightBall()) {
                    scoring.diverterRight();
                }
                if (scoring.leftBall()) {
                    scoring.diverterLeft();
                }
                hadBalls = true;
            }
        }
        else {
            hasBalls = false;
            hadBalls = false;
        }

        /*
         * Show the state and motor powers
         */
        telemetry.addData("launch distance", scoring.getLauncherDistance());
        telemetry.addData("Diverter position", scoring.diverterPose());
        telemetry.addData("Intake state", scoring.intakeSpeed());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        super.stop();
    }
}