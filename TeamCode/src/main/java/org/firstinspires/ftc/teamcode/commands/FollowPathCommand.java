package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;

/**
 * A command that follows a Pedro Pathing Path.
 * Calls follower.update() each loop, so do NOT also call it externally
 * while this command is active.
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
    public void execute() {
        follower.update();
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}
