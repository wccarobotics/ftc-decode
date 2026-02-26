package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

public class LineToCommand extends Command {
    private final Follower follower;
    private final Pose endPose;

    private final double maxPower;

    public LineToCommand(Follower follower, Pose endPose)
    {
        this(follower, endPose, 1.0);
    }
    public LineToCommand(Follower follower, Pose endPose, double maxPower) {
        this.follower = follower;
        this.endPose = endPose;
        this.maxPower = maxPower;
    }

    @Override
    public void initialize() {
        Pose startPose = follower.getPose();

        Path path = new Path(new BezierLine(startPose, endPose));
        path.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());

        PathChain pathChain = follower.pathBuilder().addPath(path).build();

        follower.followPath(pathChain, maxPower, false);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}
