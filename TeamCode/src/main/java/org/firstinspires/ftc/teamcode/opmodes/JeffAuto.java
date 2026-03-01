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

    private Pose scorePose = new Pose(61.4, 85, Math.toRadians(144)); // Scoring Pose of our robot. It is facing the goal at a -37 degree angle.
    private Pose endPose = new Pose(72, 30, Math.toRadians(180)); // Ending Pose of robot

    private CommandScheduler scheduler = new CommandScheduler();



    @Override
    public void start(){

        super.start();

        if (currentAlliance == Alliance.RED){
            scorePose = scorePose.mirror();
            endPose = endPose.mirror();
        }


        // Build the autonomous command sequence
        scheduler.schedule(new SequentialCommand(
                new LineToCommand(follower, AimAt(scorePose, goalTarget)),
                new ShootAllCommand(scoring),
                new LineToCommand(follower, getSpikePose(1,-1)),
                new InstantCommand(() -> scoring.intakeOn()),
                new LineToCommand(follower, getSpikePose(1,1), 0.25),
                new WaitCommand(.5),
                new InstantCommand(() -> scoring.switchDiverter()),
                new LineToCommand(follower, getSpikePose(1,3)),
                new InstantCommand(() -> scoring.intakeOff()),
                new LineToCommand(follower, AimAt(scorePose, goalTarget)),
                new ShootAllCommand(scoring),
                new LineToCommand(follower, getSpikePose(2,-1)),
                new InstantCommand(() -> scoring.intakeOn()),
                new LineToCommand(follower, getSpikePose(2,1), 0.25),
                new WaitCommand(.5),
                new InstantCommand(() -> scoring.switchDiverter()),
                new LineToCommand(follower, getSpikePose(2,3)),
                new InstantCommand(() -> scoring.intakeOff()),
                new LineToCommand(follower, AimAt(scorePose, goalTarget)),
                new ShootAllCommand(scoring),
                new LineToCommand(follower, endPose)
        ));
    }

    @Override
    public void loop() {
        follower.update();
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
            x = 50;
        }
        Pose pose = new Pose(x, y, Math.toRadians(180));
        if (currentAlliance == Alliance.RED){
            pose = pose.mirror();
        }
        return pose;
    }
}
