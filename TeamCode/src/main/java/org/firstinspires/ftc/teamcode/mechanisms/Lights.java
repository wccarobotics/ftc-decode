package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmodes.JeffBase;

public class Lights {
    private SingleLight centerLight = null;
    private SingleLight rightLight = null;
    private SingleLight leftLight = null;
    private JeffBase.Alliance alliance = null;
    private JeffScoring scoring = null;
    public void init(HardwareMap hardwareMap, JeffScoring scoring){
        centerLight = new SingleLight(hardwareMap.get(Servo.class, "center_status_light"));
        rightLight = new SingleLight(hardwareMap.get(Servo.class, "right_status_light"));
        leftLight = new SingleLight(hardwareMap.get(Servo.class, "left_status_light"));
        this.scoring = scoring;
    }
    public void start(JeffBase.Alliance alliance){ // we put this in the start of the opmode because we need to know the alliance
        this.alliance = alliance;
    }
    public void centerColor(double color){
        centerLight.setColor(color);
    }
    public void rightColor(double color){
        rightLight.setColor(color);
    }
    public void leftColor(double color){
        leftLight.setColor(color);
    }
    public void setAll(double color){
        centerColor(color);
        rightColor(color);
        leftColor(color);
    }
    public void ballColors(){
        if (scoring.leftBall()){ // changes the color of the light for the left ball
            double leftBallColor = scoring.leftBallColor();
            if (Math.abs(158 - leftBallColor) < 10){
                leftColor(.47);
            }
            else if (Math.abs(227 - leftBallColor) < 10) {
                leftColor(.72);
            }
        }
        else {
            leftColor(alliance == JeffBase.Alliance.BLUE ? .611 : .28);
        }
        if (scoring.rightBall()){
            double rightBallColor = scoring.rightBallColor();
            if (Math.abs(158 - rightBallColor) < 10){
                rightColor(.47);
            }
            else if (Math.abs(227 - rightBallColor) < 10) {
                rightColor(.72);
            }
        }
        else {
            rightColor(alliance == JeffBase.Alliance.BLUE ? .611 : .28);
        }
    }

    class SingleLight
    {
        private Servo light;
        private double currentColor = -1;

        public SingleLight(Servo light)
        {
            this.light = light;
        }

        public void setColor(double newColor)
        {
            if (newColor != currentColor)
            {
                light.setPosition(newColor);
                currentColor = newColor;
            }
        }
    }
}
