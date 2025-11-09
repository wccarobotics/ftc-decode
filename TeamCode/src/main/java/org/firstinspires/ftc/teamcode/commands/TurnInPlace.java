package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.RevIMU;
import org.firstinspires.ftc.teamcode.mechanisms.TankDrive;

public class TurnInPlace implements ICommand{
    TankDrive _tankDrive;
    RevIMU _IMU;
    public double _degrees;
    public double tarDegrees;

    public TurnInPlace(TankDrive tankDrive,RevIMU IMU, double degrees){
        _tankDrive = tankDrive;
        _IMU = IMU;
        _degrees = degrees;
    }
    public void Start(){
        _tankDrive.drive(0,0);
        tarDegrees = _IMU.heading(AngleUnit.DEGREES) + _degrees;
        tarDegrees = normalizeDegrees(tarDegrees);
    }

    double normalizeDegrees(double degrees)
    {
        while (degrees > 180)
        {
            degrees -= 360;
        }
        while (degrees < -180)
        {
            degrees += 360;
        }
        return degrees;
    }

    public void Loop(){

        double error = normalizeDegrees(tarDegrees - _IMU.heading(AngleUnit.DEGREES));

        if (error >0){
            _tankDrive.drive(0,.5);
        } else if (error < 0) {
            _tankDrive.drive(0,-.5);
        }
    }
    public boolean IsDone() {
        if (Math.abs(tarDegrees - _IMU.heading(AngleUnit.DEGREES)) < 5)
        {
            return true;
        }
        return false;

//        if (tarDegrees >= _IMU.heading(AngleUnit.DEGREES) && _degrees>0){
//            return true;
//        } else if (tarDegrees <= _IMU.heading(AngleUnit.DEGREES) && _degrees<0) {
//            return true;
//        } else {
//            return false;
//        }
    }
}
