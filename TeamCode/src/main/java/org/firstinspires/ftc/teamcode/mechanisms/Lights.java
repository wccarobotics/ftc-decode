package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lights {
    private Servo centerLight = null;
    private Servo rightLight = null;
    private Servo leftLight = null;
    public void init(HardwareMap hardwareMap){
        centerLight = hardwareMap.get(Servo.class, "center_status_light");
        rightLight = hardwareMap.get(Servo.class, "right_status_light");
        leftLight = hardwareMap.get(Servo.class, "left_status_light");
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
}
