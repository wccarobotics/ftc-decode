package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;

/**
 * A command that follows a Pedro Pathing Path.
 * The OpMode is responsible for calling follower.update() in its loop.
 */
public class FollowPathCommand extends Command {

    private final Follower follower;
    private final Path path;

    public FollowPathCommand(Follower follower, Path path) {
        this.follower = follower;
        this.path = path;
    }

    @Override
    public void initialize() {
        follower.followPath(path);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}
