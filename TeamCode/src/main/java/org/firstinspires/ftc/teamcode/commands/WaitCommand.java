package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A command that does nothing for a specified duration, then finishes.
 * Useful as a delay between other commands, or as a timeout via raceWith().
 */
public class WaitCommand extends Command {

    private final double seconds;
    private final ElapsedTime timer = new ElapsedTime();

    public WaitCommand(double seconds) {
        this.seconds = seconds;
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= seconds;
    }
}
