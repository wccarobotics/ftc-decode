package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

/**
 * A command that follows a Pedro Pathing PathChain (multiple path segments).
 * The OpMode is responsible for calling follower.update() in its loop.
 */
public class FollowPathChainCommand extends Command {

    private final Follower follower;
    private final PathChain pathChain;

    public FollowPathChainCommand(Follower follower, PathChain pathChain) {
        this.follower = follower;
        this.pathChain = pathChain;
    }

    @Override
    public void initialize() {
        follower.followPath(pathChain);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}
