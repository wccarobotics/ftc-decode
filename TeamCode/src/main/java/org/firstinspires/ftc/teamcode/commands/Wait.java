package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.TankDrive;

public class Wait implements ICommand {
    TankDrive _tankDrive;
    ElapsedTime timer = new ElapsedTime();
    double _time;
    public Wait(TankDrive tankDrive, double time){
        _time = time;
        _tankDrive = tankDrive;
    }
    public void Start(){
        timer.reset();
    }
    public void Loop(){
        _tankDrive.drive(0,0);
    }
    public boolean IsDone(){
        if (timer.seconds() >=_time){
            return true;
        }
        else {
            return false;
        }
    }
}
