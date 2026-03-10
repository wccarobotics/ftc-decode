package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.commands.LineToCommand;
import org.firstinspires.ftc.teamcode.commands.SequentialCommand;
import org.firstinspires.ftc.teamcode.commands.ShootAllCommand;
import org.firstinspires.ftc.teamcode.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsDrawing;

@Autonomous
public class JeffAuto extends JeffBase {

    private Pose nearScorePose = new Pose(50, 90, Math.toRadians(144)); // near scoring Pose of our robot.
    private Pose farScorePose = new Pose(72, 15, 0);
    private Pose endPose = new Pose(30, 72, Math.toRadians(180)); // Ending Pose of robot

    private enum Auto{
        NEAR,
        FAR
    }
    private Auto auto = Auto.NEAR;

    private CommandScheduler scheduler = new CommandScheduler();

    @Override
    public void init(){
        super.init();
    }
    @Override
    public void init_loop(){
        super.init_loop();
        if (gamepad1.dpadRightWasPressed()){
            if (auto == Auto.NEAR){
                auto = Auto.FAR;
            }
            else if (auto == Auto.FAR){
                auto = Auto.NEAR;
            }
        }
        telemetry.addData("Auto", auto);
    }



    @Override
    public void start(){

        super.start();

        if (currentAlliance == Alliance.RED){
            nearScorePose = nearScorePose.mirror();
            endPose = endPose.mirror();
        }

        if (auto == Auto.NEAR) {
//            scheduler.schedule(new SequentialCommand(
//                    new LineToCommand(follower, AimAt(nearScorePose, goalTarget)),
//                    new WaitCommand(2),
//                    new LineToCommand(follower, getSpikePose(1, -1)),
//                    new WaitCommand(2),
//                    new LineToCommand(follower, AimAt(nearScorePose, goalTarget)),
//                    new WaitCommand(2),
//                    new LineToCommand(follower, getSpikePose(3, -1)),
//                    new WaitCommand(2),
//                    new LineToCommand(follower, AimAt(nearScorePose, goalTarget)),
//                    new WaitCommand(2),
//                    new LineToCommand(follower, getSpikePose(1, -1)),
//                    new WaitCommand(2),new LineToCommand(follower, AimAt(nearScorePose, goalTarget)),
//                    new WaitCommand(2),
//                    new LineToCommand(follower, getSpikePose(3, -1)),
//                    new WaitCommand(2),new LineToCommand(follower, AimAt(nearScorePose, goalTarget)),
//                    new WaitCommand(2),
//                    new LineToCommand(follower, getSpikePose(1, -1)),
//                    new WaitCommand(2),new LineToCommand(follower, AimAt(nearScorePose, goalTarget)),
//                    new WaitCommand(2),
//                    new LineToCommand(follower, getSpikePose(3, -1)),
//                    new WaitCommand(2),new LineToCommand(follower, AimAt(nearScorePose, goalTarget)),
//                    new WaitCommand(2),
//                    new LineToCommand(follower, getSpikePose(1, -1)),
//                    new WaitCommand(2)
//            ));

            // Build the autonomous command sequence
            scheduler.schedule(new SequentialCommand(
//                new AimCommand(follower,vision, telemetry),
//                new InstantCommand(()-> scoring.intakeOn())
                    new LineToCommand(follower, AimAt(nearScorePose, goalTarget)),
                    new ShootAllCommand(scoring, vision),
                    new LineToCommand(follower, getSpikePose(1, -1)),
                    new InstantCommand(() -> scoring.intakeOn()),
                    new LineToCommand(follower, getSpikePose(1, 1), 0.25, true),
                    new WaitCommand(.25),
                    new InstantCommand(() -> scoring.switchDiverter()),
                    new WaitCommand(.25),
                    new LineToCommand(follower, getSpikePose(1, 3), 0.25, true),
                    new InstantCommand(() -> scoring.intakeOff()),
                    new LineToCommand(follower, AimAt(nearScorePose, goalTarget)),
                    new ShootAllCommand(scoring, vision),
                    new LineToCommand(follower, getSpikePose(2, -1)),
                    new InstantCommand(() -> scoring.intakeOn()),
                    new LineToCommand(follower, getSpikePose(2, 1), 0.25, true),
                    new WaitCommand(.25),
                    new InstantCommand(() -> scoring.switchDiverter()),
                    new WaitCommand(.25),
                    new LineToCommand(follower, getSpikePose(2, 3), 0.25, true),
                    new InstantCommand(() -> scoring.intakeOff()),
                    new LineToCommand(follower, AimAt(nearScorePose, goalTarget)),
                    new ShootAllCommand(scoring, vision),
                    new LineToCommand(follower, endPose)
            ));
        }
        else if (auto == Auto.FAR){
            scheduler.schedule( new SequentialCommand(
                    new InstantCommand(()-> scoring.setLaunchDistance()),
                    new LineToCommand(follower, AimAt(farScorePose, goalTarget)),
                    new ShootAllCommand(scoring,vision),
                    new InstantCommand(() -> scoring.intakeOn()),
                    new LineToCommand(follower, getSpikePose(3,-1)),
                    new LineToCommand(follower, getSpikePose(3,1), .25, true),
                    new InstantCommand(() -> scoring.switchDiverter()),
                    new WaitCommand(.25),
                    new LineToCommand(follower, getSpikePose(3,1),.25, true),
                    new InstantCommand(() -> scoring.intakeOff()),
                    new ShootAllCommand(scoring, vision)
            ));
        }
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();
        vision.update();
        scoring.updateAll();
        PanelsDrawing.drawDebug(follower);
        scheduler.run();
    }
    private Pose getSpikePose(double spike, double ball){
        double x = 29.5;
        double y = 84;
        if(ball == 1){
            x = 36;
        }
        if (ball == 2){
            x = 30;
        }
        if (ball == 3){
            x = 25.5;
        }
        if (spike == 1){
            y = 84;
        }
        if (spike == 2){
            y = 60;
        }
        if (spike == 3){
            y = 34;
        }
        if (ball == -1){
            x = 41;
        }
        Pose pose = new Pose(x, y, Math.toRadians(180));
        if (currentAlliance == Alliance.RED){
            pose = pose.mirror();
        }
        return pose;
    }
}
