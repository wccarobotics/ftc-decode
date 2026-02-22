package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

public class LineToCommand extends Command {
    private final Follower follower;
    private final Pose endPose;

    public LineToCommand(Follower follower, Pose endPose) {
        this.follower = follower;
        this.endPose = endPose;
    }

    @Override
    public void initialize() {
        Pose startPose = follower.getPose();
        Path path = new Path(new BezierLine(startPose, endPose));
        path.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());
        follower.followPath(path);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}
