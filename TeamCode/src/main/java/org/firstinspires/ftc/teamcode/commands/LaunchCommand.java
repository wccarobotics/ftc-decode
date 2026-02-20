package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.mechanisms.ScoringRI3D;

/**
 * Initiates a launch on either the left or right launcher,
 * then waits until the corresponding launch state returns to IDLE.
 */
public class LaunchCommand extends Command {

    public enum Side { LEFT, RIGHT }

    private final ScoringRI3D scoring;
    private final Side side;

    public LaunchCommand(ScoringRI3D scoring, Side side) {
        this.scoring = scoring;
        this.side = side;
    }

    @Override
    public void initialize() {
        if (side == Side.LEFT) {
            scoring.shootLeft();
        } else {
            scoring.shootRight();
        }
    }

    @Override
    public boolean isFinished() {
        if (side == Side.LEFT) {
            return scoring.getLeftLaunchState() == ScoringRI3D.LaunchState.IDLE;
        } else {
            return scoring.getRightLaunchState() == ScoringRI3D.LaunchState.IDLE;
        }
    }
}
