package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchCommand;
import org.firstinspires.ftc.teamcode.commands.LineToCommand;
import org.firstinspires.ftc.teamcode.commands.SequentialCommand;
import org.firstinspires.ftc.teamcode.commands.WaitCommand;
import org.firstinspires.ftc.teamcode.mechanisms.ScoringRI3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsDrawing;

@Autonomous
public class JeffAuto extends JeffBase {

    private Pose scorePose = new Pose(61.4, 85, Math.toRadians(144)); // Scoring Pose of our robot. It is facing the goal at a -37 degree angle.
    private Pose midPose = new Pose(46, 81.6, Math.toRadians(180)); // Ending Pose of robot
    private Pose Grab1PoseA = new Pose(37.3, 84, Math.toRadians(180));
    private Pose Grab1PoseB = new Pose(26, 84, Math.toRadians(180));

    private CommandScheduler scheduler = new CommandScheduler();



    @Override
    public void start(){

        super.start();

        if (currentAlliance == Alliance.RED){
            scorePose = scorePose.mirror();
            midPose = midPose.mirror();
        }


        // Build the autonomous command sequence
        scheduler.schedule(new SequentialCommand(
                new LineToCommand(follower, AimAt(scorePose, goalTarget)),
                new LaunchCommand(scoring, LaunchCommand.Side.RIGHT),
                new LaunchCommand(scoring, LaunchCommand.Side.LEFT),
                new InstantCommand(() -> scoring.switchDiverter()),
                new InstantCommand(()-> scoring.runIntake()),
                new WaitCommand(0.5),
                new LaunchCommand(scoring, LaunchCommand.Side.RIGHT),
                new LineToCommand(follower, midPose),
                new LineToCommand(follower, Grab1PoseA, 0.25),
                new InstantCommand(() -> scoring.switchDiverter()),
                new LineToCommand(follower, Grab1PoseB),
                new LineToCommand(follower, AimAt(scorePose, goalTarget)),
                new LaunchCommand(scoring, LaunchCommand.Side.RIGHT),
                new LaunchCommand(scoring, LaunchCommand.Side.LEFT),
                new InstantCommand(() -> scoring.switchDiverter()),
                new InstantCommand(()-> scoring.runIntake()),
                new WaitCommand(0.5),
                new LaunchCommand(scoring, LaunchCommand.Side.RIGHT)
        ));
    }

    @Override
    public void loop() {
        follower.update();
        scoring.updateAll();
        PanelsDrawing.drawDebug(follower);
        scheduler.run();
    }
}
