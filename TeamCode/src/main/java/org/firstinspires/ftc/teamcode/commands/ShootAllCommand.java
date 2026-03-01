package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.mechanisms.ScoringRI3D;

public class ShootAllCommand extends CommandCommand{

    private ScoringRI3D scoring;

    public ShootAllCommand(ScoringRI3D scoring){
        this.scoring = scoring;
    }

    @Override
    public void initialize(){
        boolean diverterWasLeft = scoring.diverterPose() > 0.5;

        addCommand(new LaunchCommand(scoring, diverterWasLeft ? LaunchCommand.Side.RIGHT : LaunchCommand.Side.LEFT));
        addCommand(new LaunchCommand(scoring, diverterWasLeft ? LaunchCommand.Side.LEFT : LaunchCommand.Side.RIGHT));
        addCommand(new InstantCommand(() -> {
            scoring.switchDiverter();
            scoring.intakeOn();
        }));
        addCommand(new WaitCommand(0.5));
        addCommand(new LaunchCommand(scoring, diverterWasLeft ? LaunchCommand.Side.RIGHT : LaunchCommand.Side.LEFT));
        addCommand(new InstantCommand(() -> scoring.intakeOff()));



    }
}
