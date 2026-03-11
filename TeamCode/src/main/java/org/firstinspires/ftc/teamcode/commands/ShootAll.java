package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.JeffScoring;

public class ShootAll extends Command {
    private JeffScoring scoring;
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

    public  ShootAll(JeffScoring scoring){
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
                    if (scoring.getRightLaunchState() == JeffScoring.LaunchState.IDLE) {
                        scoring.shootLeft();
                        shotState = ShotState.SWITCHDIVERTER;
                        time.reset();
                    }
                    break;
                case SWITCHDIVERTER:
                    if (scoring.getLeftLaunchState() == JeffScoring.LaunchState.IDLE) {
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
                    if (scoring.getRightLaunchState() == JeffScoring.LaunchState.IDLE) {
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
                    if (scoring.getLeftLaunchState() == JeffScoring.LaunchState.IDLE){
                        shotState = ShotState.SECONDLAUNCH;
                    }
                    break;
                case SECONDLAUNCH:
                    scoring.shootRight();
                    if (scoring.getRightLaunchState() == JeffScoring.LaunchState.IDLE){
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
                    if (scoring.getLeftLaunchState() == JeffScoring.LaunchState.IDLE){
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
