package org.firstinspires.ftc.teamcode.commands;

/**
 * A command that runs a single action immediately and then finishes.
 * Useful for one-shot operations like setting a servo position or toggling a flag.
 */
public class InstantCommand extends Command {

    private final Runnable action;

    public InstantCommand(Runnable action) {
        this.action = action;
    }

    @Override
    public void initialize() {
        action.run();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
