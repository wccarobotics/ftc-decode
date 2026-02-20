package org.firstinspires.ftc.teamcode.commands;

import java.util.Arrays;
import java.util.List;

/**
 * Runs a list of commands in order. When one finishes, the next one starts.
 * The group is finished when the last command completes.
 */
public class SequentialCommand extends Command {

    private final List<Command> commands;
    private int currentIndex = -1;

    public SequentialCommand(Command... commands) {
        this.commands = Arrays.asList(commands);
    }

    @Override
    public void initialize() {
        currentIndex = 0;
        if (!commands.isEmpty()) {
            commands.get(0).initialize();
        }
    }

    @Override
    public void execute() {
        if (currentIndex >= commands.size()) {
            return;
        }

        Command current = commands.get(currentIndex);
        current.execute();

        if (current.isFinished()) {
            current.end(false);
            currentIndex++;
            if (currentIndex < commands.size()) {
                commands.get(currentIndex).initialize();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return currentIndex >= commands.size();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted && currentIndex >= 0 && currentIndex < commands.size()) {
            commands.get(currentIndex).end(true);
        }
    }
}
