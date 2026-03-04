package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.mechanisms.LimelightVision;
import org.firstinspires.ftc.teamcode.mechanisms.ScoringRI3D;
import org.firstinspires.ftc.teamcode.opmodes.JeffBase;

public class ShootAllCommand extends CommandCommand{

    private ScoringRI3D scoring;
    private LimelightVision limelight;

    public ShootAllCommand(ScoringRI3D scoring, LimelightVision limelight){
        this.scoring = scoring;
        this.limelight = limelight;
    }

    @Override
    public void initialize(){
        boolean diverterWasLeft = scoring.diverterPose() > 0.5;
        diverterWasLeft = true;

        addCommand(new LaunchCommand(scoring, diverterWasLeft ? LaunchCommand.Side.RIGHT : LaunchCommand.Side.LEFT));
        addCommand(new WaitCommand(.75));
        addCommand(new LaunchCommand(scoring, diverterWasLeft ? LaunchCommand.Side.LEFT : LaunchCommand.Side.RIGHT));
        addCommand(new InstantCommand(() -> {
            scoring.switchDiverter();
            scoring.intakeOn();
        }));
        addCommand(new WaitCommand(0.75));
        addCommand(new LaunchCommand(scoring, diverterWasLeft ? LaunchCommand.Side.RIGHT : LaunchCommand.Side.LEFT));
        addCommand(new InstantCommand(() -> scoring.intakeOff()));



    }
}
