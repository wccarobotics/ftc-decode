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

    private Pose startPose = new Pose(22.6, 128.4, Math.toRadians(139.9)); // Start Pose of our robot.
    private Pose scorePose = new Pose(61.4, 85, Math.toRadians(144)); // Scoring Pose of our robot. It is facing the goal at a -37 degree angle.
    private Pose endPose = new Pose(46, 81.6, Math.toRadians(180)); // Ending Pose of robot

    private CommandScheduler scheduler = new CommandScheduler();



    @Override
    public void start(){

        if (currentAlliance == Alliance.RED){
            startPose = startPose.mirror();
            scorePose = scorePose.mirror();
            endPose = endPose.mirror();
        }
        follower.setStartingPose(startPose);
        follower.update();

        // Build the autonomous command sequence
        scheduler.schedule(new SequentialCommand(
                new LineToCommand(follower, scorePose),
                new LaunchCommand(scoring, LaunchCommand.Side.RIGHT),
                new LaunchCommand(scoring, LaunchCommand.Side.LEFT),
                new InstantCommand(() -> scoring.switchDiverter()),
                new InstantCommand(()-> scoring.runIntake()),
                new WaitCommand(0.5),
                new LaunchCommand(scoring, LaunchCommand.Side.RIGHT),
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
}
