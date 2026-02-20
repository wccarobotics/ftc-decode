package org.firstinspires.ftc.teamcode.commands;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * Manages the execution of commands from an OpMode loop.
 * <p>
 * Typical usage:
 * <pre>
 *   CommandScheduler scheduler = new CommandScheduler();
 *   // in init or start:
 *   scheduler.schedule(myCommand);
 *   // in loop:
 *   scheduler.run();
 * </pre>
 */
public class CommandScheduler {

    private final List<Command> scheduledCommands = new ArrayList<>();
    private final List<Command> activeCommands = new ArrayList<>();

    /**
     * Schedules a command to be started on the next run() call.
     */
    public void schedule(Command command) {
        scheduledCommands.add(command);
    }

    /**
     * Call this once per loop iteration. Initializes newly scheduled commands,
     * executes active commands, and ends finished commands.
     */
    public void run() {
        // Initialize newly scheduled commands
        for (Command command : scheduledCommands) {
            command.initialize();
            activeCommands.add(command);
        }
        scheduledCommands.clear();

        // Execute active commands and remove finished ones
        Iterator<Command> it = activeCommands.iterator();
        while (it.hasNext()) {
            Command command = it.next();
            command.execute();
            if (command.isFinished()) {
                command.end(false);
                it.remove();
            }
        }
    }

    /**
     * Interrupts and removes all active commands.
     */
    public void cancelAll() {
        for (Command command : activeCommands) {
            command.end(true);
        }
        activeCommands.clear();
        scheduledCommands.clear();
    }

    /**
     * Returns true when there are no active or scheduled commands.
     */
    public boolean isFinished() {
        return activeCommands.isEmpty() && scheduledCommands.isEmpty();
    }
}
