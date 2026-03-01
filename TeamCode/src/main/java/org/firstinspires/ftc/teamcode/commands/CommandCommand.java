package org.firstinspires.ftc.teamcode.commands;

import java.util.ArrayList;

public class CommandCommand extends Command {
    protected ArrayList<Command> scheduledCommands = new ArrayList<>();

    private Command currentCommand = null;

    @Override
    public void execute(){
        if (currentCommand != null)
        {
            currentCommand.execute();

            if (currentCommand.isFinished()) {
                currentCommand.end(false);
                currentCommand = null;
            }
        }

        if (currentCommand == null && scheduledCommands.size() > 0)
        {
            currentCommand = scheduledCommands.get(0);
            scheduledCommands.remove(0);

            currentCommand.initialize();
        }
    }

    @Override
    public boolean isFinished() {
        return currentCommand == null;
    }

    public void addCommand(Command command){
        scheduledCommands.add(command);
    }
}
