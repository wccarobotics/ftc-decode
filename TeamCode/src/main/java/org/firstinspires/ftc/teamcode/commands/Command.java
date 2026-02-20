package org.firstinspires.ftc.teamcode.commands;

/**
 * Abstract base class for all commands in the framework.
 * Provides a lifecycle of initialize → execute (repeated) → end,
 * with isFinished() controlling when execution stops.
 */
public abstract class Command {

    /**
     * Called once when the command is first started.
     * Override to set up initial state.
     */
    public void initialize() {}

    /**
     * Called repeatedly while the command is active.
     * Override to perform the command's main work.
     */
    public void execute() {}

    /**
     * Returns true when the command should stop executing.
     * By default returns false (runs forever until interrupted).
     */
    public boolean isFinished() {
        return false;
    }

    /**
     * Called once when the command ends, either by finishing or being interrupted.
     * @param interrupted true if the command was cancelled before isFinished() returned true
     */
    public void end(boolean interrupted) {}

    /**
     * Creates a SequentialCommand that runs this command followed by the given commands.
     */
    public Command andThen(Command... next) {
        Command[] all = new Command[next.length + 1];
        all[0] = this;
        System.arraycopy(next, 0, all, 1, next.length);
        return new SequentialCommand(all);
    }

    /**
     * Creates a ParallelCommand that runs this command alongside the given commands.
     * Finishes when all commands are done.
     */
    public Command alongWith(Command... others) {
        Command[] all = new Command[others.length + 1];
        all[0] = this;
        System.arraycopy(others, 0, all, 1, others.length);
        return new ParallelCommand(all);
    }

    /**
     * Creates a ParallelRaceCommand that runs this command alongside the given commands.
     * Finishes when any command is done.
     */
    public Command raceWith(Command... others) {
        Command[] all = new Command[others.length + 1];
        all[0] = this;
        System.arraycopy(others, 0, all, 1, others.length);
        return new ParallelRaceCommand(all);
    }

    /**
     * Wraps this command with a timeout. If the command doesn't finish
     * within the given number of seconds, it will be interrupted.
     */
    public Command withTimeout(double seconds) {
        return raceWith(new WaitCommand(seconds));
    }
}
