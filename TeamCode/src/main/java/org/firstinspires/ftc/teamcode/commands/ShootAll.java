package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.ScoringRI3D;

public class ShootAll extends Command {
    private ScoringRI3D scoring;
    private boolean diverterWasLeft = false;
    private ElapsedTime time = new ElapsedTime();
    private enum ShotState{
        FIRSTLAUNCH,
        SECONDLAUNCH,
        SWITCHDIVERTER,
        THIRDLAUNCH,
        END,
        REALEND
    }
    private ShotState shotState = ShotState.FIRSTLAUNCH;

    public  ShootAll(ScoringRI3D scoring){
        this.scoring = scoring;
    }


    @Override
    public void initialize()
    {
        if (scoring.diverterPose() > .60){
            diverterWasLeft = false;
        }
        else diverterWasLeft = true;
    }

    @Override
    public void execute() {
        scoring.updateAll();
        if (diverterWasLeft){
            switch (shotState){
                case FIRSTLAUNCH:
                    scoring.shootRight();
                    shotState = ShotState.SECONDLAUNCH;
                    break;
                case SECONDLAUNCH:
                    if (scoring.getRightLaunchState() == ScoringRI3D.LaunchState.IDLE) {
                        scoring.shootLeft();
                        shotState = ShotState.SWITCHDIVERTER;
                        time.reset();
                    }
                    break;
                case SWITCHDIVERTER:
                    if (scoring.getLeftLaunchState() == ScoringRI3D.LaunchState.IDLE) {
                        scoring.switchDiverter();
                        scoring.runIntake();
                        shotState = ShotState.THIRDLAUNCH;
                    }
                    break;
                case THIRDLAUNCH:
                    if (time.seconds() >= .5) {
                        scoring.shootRight();
                        shotState = ShotState.END;
                    }
                    break;
                case END:
                    if (scoring.getRightLaunchState() == ScoringRI3D.LaunchState.IDLE) {
                        scoring.switchIntake();
                        shotState = ShotState.REALEND;
                    }
                    break;
            }
        }
        else {
            switch (shotState){
                case FIRSTLAUNCH:
                    scoring.shootLeft();
                    if (scoring.getLeftLaunchState() == ScoringRI3D.LaunchState.IDLE){
                        shotState = ShotState.SECONDLAUNCH;
                    }
                    break;
                case SECONDLAUNCH:
                    scoring.shootRight();
                    if (scoring.getRightLaunchState() == ScoringRI3D.LaunchState.IDLE){
                        shotState = ShotState.SWITCHDIVERTER;
                        time.reset();
                    }
                    break;
                case SWITCHDIVERTER:
                    scoring.switchDiverter();
                    scoring.runIntake();
                    if (time.seconds() >= .5){
                        shotState = ShotState.THIRDLAUNCH;
                    }
                    break;
                case THIRDLAUNCH:
                    scoring.shootLeft();
                    if (scoring.getLeftLaunchState() == ScoringRI3D.LaunchState.IDLE){
                        scoring.switchIntake();
                        shotState = ShotState.END;
                    }
                    break;
            }
        }
    }
    @Override
    public boolean isFinished(){
        return shotState == ShotState.REALEND;
    }
}
