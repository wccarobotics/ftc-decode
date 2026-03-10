package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmodes.JeffBase;

public class Lights {
    private Servo centerLight = null;
    private Servo rightLight = null;
    private Servo leftLight = null;
    private JeffBase.Alliance alliance = null;
    private ScoringRI3D scoring = null;
    public void init(HardwareMap hardwareMap, ScoringRI3D scoring){
        centerLight = hardwareMap.get(Servo.class, "center_status_light");
        rightLight = hardwareMap.get(Servo.class, "right_status_light");
        leftLight = hardwareMap.get(Servo.class, "left_status_light");
        this.scoring = scoring;
    }
    public void start(JeffBase.Alliance alliance){ // we put this in the start of the opmode because we need to know the alliance
        this.alliance = alliance;
    }
    public void centerColor(double color){
        centerLight.setPosition(color);
    }
    public void rightColor(double color){
        rightLight.setPosition(color);
    }
    public void leftColor(double color){
        leftLight.setPosition(color);
    }
    public void setAll(double color){
        centerColor(color);
        rightColor(color);
        leftColor(color);
    }
    public void ballColors(){
        if (scoring.leftBall()){ // changes the color of the light for the left ball
            if (Math.abs(158 - scoring.leftBallColor()) < 10){
                leftColor(.47);
            }
            else if (Math.abs(227 - scoring.leftBallColor()) < 10) {
                leftColor(.72);
            }
        }
        else leftColor(alliance == JeffBase.Alliance.BLUE? .611: .28);
        if (scoring.rightBall()){
            if (Math.abs(158 - scoring.rightBallColor()) < 10){
                rightColor(.47);
            }
            else if (Math.abs(227 - scoring.rightBallColor()) < 10) {
                rightColor(.72);
            }
        }
        else rightColor(alliance == JeffBase.Alliance.BLUE? .611: .28);
    }
}
