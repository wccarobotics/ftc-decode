package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.mechanisms.JeffScoring;
import org.firstinspires.ftc.teamcode.opmodes.JeffBase;

public class YummyArtifacts extends CommandCommand{
    private JeffScoring scoring;
    private Follower follower;
    private double spike;

    private Pose getSpikePose(double spike, double ball){
        double x = 29.5;
        double y = 84;
        if(ball == 1){
            x = 36;
        }
        if (ball == 2){
            x = 30;
        }
        if (ball == 3){
            x = 25.5;
        }
        if (spike == 1){
            y = 84;
        }
        if (spike == 2){
            y = 60;
        }
        if (spike == 3){
            y = 36;
        }
        if (ball == -1){
            x = 45;
        }
        Pose pose = new Pose(x, y, Math.toRadians(180));
        if (JeffBase.currentAlliance == JeffBase.Alliance.RED){
            pose = pose.mirror();
        }
        return pose;
    }

    public  YummyArtifacts(JeffScoring scoring, Follower follower, Double spike){
        this.scoring = scoring;
        this.follower = follower;
        this.spike = spike;
    }

    @Override
    public void initialize(){
        addCommand( new InstantCommand(() -> scoring.intakeOn()));
        addCommand(new LineToCommand(follower, getSpikePose(spike, -1)));
        addCommand(new LineToCommand(follower, getSpikePose(spike, 1), 0.5, true));
        addCommand( new WaitCommand(.25));
        addCommand( new InstantCommand(() -> scoring.switchDiverter()));
        addCommand( new WaitCommand(.25));
        addCommand( new LineToCommand(follower, getSpikePose(spike, 3), 0.25, true));
        addCommand( new InstantCommand(() -> scoring.intakeOff()));
    }
}
