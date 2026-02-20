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
import org.firstinspires.ftc.teamcode.commands.SequentialCommand;
import org.firstinspires.ftc.teamcode.mechanisms.ScoringRI3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsDrawing;

@Autonomous
public class JeffAuto extends OpMode {

    private final Pose startPose = new Pose(28, 133, Math.toRadians(127)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(61.4, 85, Math.toRadians(133)); // Scoring Pose of our robot. It is facing the goal at a -37 degree angle.
    private final Pose endPose = new Pose(46, 81.6, Math.toRadians(180)); // Ending Pose of robot
    private Path scoringPath;
    private Path endPath;

    ScoringRI3D scoring = new ScoringRI3D();
    private Follower follower;
    private TelemetryManager telemetryM;
    private CommandScheduler scheduler = new CommandScheduler();

    private enum Alliance{
        BLUE,
        RED
    }
    private Alliance currentAlliance = Alliance.BLUE;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        PanelsDrawing.init();
        scoring.init(hardwareMap, telemetry);

        // make paths
        scoringPath = new Path(new BezierLine(startPose, scorePose));
        scoringPath.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        endPath = new Path(new BezierLine(scorePose, endPose));
        scoringPath.setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading());

        // Build the autonomous command sequence
        scheduler.schedule(new SequentialCommand(
            new FollowPathCommand(follower, scoringPath),
            new LaunchCommand(scoring, LaunchCommand.Side.RIGHT),
            new LaunchCommand(scoring, LaunchCommand.Side.LEFT),
            new InstantCommand(() -> scoring.switchDiverter()),
            new LaunchCommand(scoring, LaunchCommand.Side.RIGHT),
            new FollowPathCommand(follower, endPath)
        ));

        telemetry.addData("Status", "Initialized");
    }

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

    @Override
    public void loop() {
        follower.update();
        scoring.updateAll();
        PanelsDrawing.drawDebug(follower);
        scheduler.run();
    }
}
