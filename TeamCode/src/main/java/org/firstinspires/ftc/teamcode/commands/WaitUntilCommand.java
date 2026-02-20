package org.firstinspires.ftc.teamcode.commands;

import java.util.function.BooleanSupplier;

/**
 * A command that waits until a condition becomes true.
 * Useful for polling mechanism state, such as waiting for a launcher to finish.
 */
public class WaitUntilCommand extends Command {

    private final BooleanSupplier condition;

    public WaitUntilCommand(BooleanSupplier condition) {
        this.condition = condition;
    }

    @Override
    public boolean isFinished() {
        return condition.getAsBoolean();
    }
}
