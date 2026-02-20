package org.firstinspires.ftc.teamcode.commands;

import java.util.Arrays;
import java.util.List;

/**
 * Runs multiple commands at the same time.
 * Finishes when ANY command completes — remaining commands are interrupted.
 */
public class ParallelRaceCommand extends Command {

    private final List<Command> commands;
    private boolean done = false;
    private int winnerIndex = -1;

    public ParallelRaceCommand(Command... commands) {
        this.commands = Arrays.asList(commands);
    }

    @Override
    public void initialize() {
        done = false;
        winnerIndex = -1;
        for (Command command : commands) {
            command.initialize();
        }
    }

    @Override
    public void execute() {
        for (int i = 0; i < commands.size(); i++) {
            Command command = commands.get(i);
            command.execute();
            if (command.isFinished()) {
                done = true;
                winnerIndex = i;
                return;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        for (int i = 0; i < commands.size(); i++) {
            // The winner finished naturally; all others are interrupted
            commands.get(i).end(i != winnerIndex || interrupted);
        }
    }
}
