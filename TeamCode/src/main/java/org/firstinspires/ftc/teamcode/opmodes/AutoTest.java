package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.DriveStraight;
import org.firstinspires.ftc.teamcode.commands.ICommand;
import org.firstinspires.ftc.teamcode.commands.TurnInPlace;
import org.firstinspires.ftc.teamcode.commands.Wait;
import org.firstinspires.ftc.teamcode.mechanisms.RevIMU;
import org.firstinspires.ftc.teamcode.mechanisms.TankDrive;

import java.util.ArrayList;

@Autonomous
@Disabled
public class AutoTest extends OpMode {

    ElapsedTime autoTime = new ElapsedTime();
    TankDrive tankDrive = new TankDrive();
    RevIMU IMU = new RevIMU();
    double state;
    ArrayList<ICommand> _commands = new ArrayList<>();
    int _currentCommand;

    @Override
    public void init() {
        tankDrive.init(hardwareMap);
        IMU.init(hardwareMap);

        state = 0;
        _commands.clear();

        _commands.add(new DriveStraight(tankDrive,12));
        _commands.add(new Wait(tankDrive, 5));
        _commands.add(new TurnInPlace(tankDrive, IMU, 90));
        _commands.add(new DriveStraight(tankDrive,-24));
        _commands.add(new Wait(tankDrive, 5));
        _commands.add(new TurnInPlace(tankDrive, IMU, -90));
        _commands.add(new DriveStraight(tankDrive,36));

        _currentCommand = -1;

    }

    @Override
    public void loop() {

        telemetry.addData("current command", _currentCommand);
        telemetry.addData("heading", IMU.heading(AngleUnit.DEGREES));
        if (_currentCommand == -1 ||
            (_currentCommand < _commands.size() && _commands.get(_currentCommand).IsDone()))
        {
            _currentCommand += 1;
            if (_currentCommand >= _commands.size()) {
                tankDrive.drive(0,0);
            }
            else {
                _commands.get(_currentCommand).Start();
            }
        }

        if (_currentCommand < _commands.size()) {
            _commands.get(_currentCommand).Loop();
//            telemetry.addData("starting distance", _commands.get(_currentCommand)._startDistance);
//            telemetry.addData("target distance", _commands.get(_currentCommand)._distanceInInches);
        }


//        telemetry.addData("state", state);
//        telemetry.addData("nearlyThere", tankDrive.areWeNearlyThereYet);
//        telemetry.addData("setDistance", tankDrive.haveWeSetInitialDistance);
//        telemetry.addData("initialDistance", tankDrive.initialDistance);
//        telemetry.addData("distance", tankDrive.distance());
//
//        if (state == 0) {
//            tankDrive.driveForDistance(12, .5);
//            if (tankDrive.areWeNearlyThereYet) {
//                state = 1;
//                backInMyDay = autoTime.seconds();
//            }
//        }
//        if (state == 1) {
//            tankDrive.drive(0,0);
//            if (autoTime.seconds() - backInMyDay > 2)
//                state = 2;
//        }
//        if (state == 2) {
//            tankDrive.driveForDistance(-12, .5);
//            if (tankDrive.areWeNearlyThereYet) {
//                state = 3;
//                tankDrive.drive(0, 0);
//            }
//        }

//        tankDrive.initDrive(12);
//        if (state == 0) {
//            if (tankDrive.done()) {
//                tankDrive.initDrive(-12);
//                state = 1;
//            }
//        }
    }
}
