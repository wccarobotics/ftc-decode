package org.firstinspires.ftc.teamcode.commands;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Runs multiple commands at the same time.
 * Finishes when ALL commands have completed.
 * Commands that finish early have end(false) called immediately.
 */
public class ParallelCommand extends Command {

    private final List<Command> commands;
    private final Set<Integer> finished = new HashSet<>();

    public ParallelCommand(Command... commands) {
        this.commands = Arrays.asList(commands);
    }

    @Override
    public void initialize() {
        finished.clear();
        for (Command command : commands) {
            command.initialize();
        }
    }

    @Override
    public void execute() {
        for (int i = 0; i < commands.size(); i++) {
            if (finished.contains(i)) {
                continue;
            }
            Command command = commands.get(i);
            command.execute();
            if (command.isFinished()) {
                command.end(false);
                finished.add(i);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return finished.size() >= commands.size();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            for (int i = 0; i < commands.size(); i++) {
                if (!finished.contains(i)) {
                    commands.get(i).end(true);
                }
            }
        }
    }
}
