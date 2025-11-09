package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.TankDrive;

public class DriveStraight implements ICommand{
    TankDrive _tankDrive;
    public double _distanceInInches;

    public double _startDistance;

    public DriveStraight(TankDrive tankDrive, double distanceInInches){
        _tankDrive = tankDrive;
        _distanceInInches = distanceInInches;
    }

    public void Start()
    {
        _startDistance = _tankDrive.distance();
    }

    public void Loop()
    {
        if (_distanceInInches > 0) {
            _tankDrive.drive(0.5, 0);
        } else {
            _tankDrive.drive(-0.5, 0);
        }
    }

    public boolean IsDone()
    {
        if (_distanceInInches > 0) {
            return _tankDrive.distance() - _startDistance >= _distanceInInches;
        } else {
            return _tankDrive.distance() - _startDistance <= _distanceInInches;
        }
    }

}
