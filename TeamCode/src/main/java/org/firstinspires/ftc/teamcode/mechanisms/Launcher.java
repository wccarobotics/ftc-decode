package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {
    private DcMotorEx launcher;
    private CRServo leftFeeder;
    private CRServo rightFeeder;
    private Servo flap;
    private final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    private final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    private final double FULL_SPEED = 1.0;
    public double flapAngle;
    ElapsedTime feederTimer = new ElapsedTime();
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;
    public double LAUNCHER_TARGET_VELOCITY;
    public void init(HardwareMap hardwareMap){
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        flap = hardwareMap.get(Servo.class, "angle_servo");
        launchState = LaunchState.IDLE;
        // flywheel setup
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(BRAKE);
        LAUNCHER_TARGET_VELOCITY = 2400;
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        // feeder setup
        stopFeeder();
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        // flap setup
        flapAngle = 0;
    }
    public void stopFeeder(){
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
    }
    public void updateState(){
        switch (launchState) {
            case IDLE:
                break;
            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY / 60 * 28);
                if (launcher.getVelocity() > .95 * LAUNCHER_TARGET_VELOCITY / 60 * 28) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    stopFeeder();
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }
    public void startLauncher(){
        if (launchState == LaunchState.IDLE){
            launchState = LaunchState.SPIN_UP;
        }
    }
    public void stopLauncher(){
        stopFeeder();
        launcher.setVelocity(STOP_SPEED);
        launchState = LaunchState.IDLE;
    }
    public String getState(){
        return launchState.toString();
    }
    public double launcherSpeed(){
        return launcher.getVelocity();
    }
    public void adjustLauncherTarget(double change){
        LAUNCHER_TARGET_VELOCITY += change;
        if (LAUNCHER_TARGET_VELOCITY > (5500)){
            LAUNCHER_TARGET_VELOCITY = 5500;
        }
        else if (LAUNCHER_TARGET_VELOCITY < (100)){
            LAUNCHER_TARGET_VELOCITY = 100;
        }
    }
    public void setLauncherTarget(double speed){
        LAUNCHER_TARGET_VELOCITY = speed;
        if (LAUNCHER_TARGET_VELOCITY > (5500)){
            LAUNCHER_TARGET_VELOCITY = 5500;
        }
        else if (LAUNCHER_TARGET_VELOCITY < (100)){
            LAUNCHER_TARGET_VELOCITY = 100;
        }
    }
    public void spinLauncher(){
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY/60 *28);
    }
    public void flapIncement(double change){
        flapAngle += change;
        if (flapAngle < 0){
            flapAngle = 0;
        }
        if (flapAngle > 60){
            flapAngle = 60;
        }
        flap.setPosition((flapAngle/300) + 0.5);
    }
    public void setFlap(double target){
        flapAngle = target;
        if (flapAngle < 0){
            flapAngle = 0;
        }
        if (flapAngle > 60){
            flapAngle = 60;
        }
        flap.setPosition((flapAngle/300) + 0.5);
    }
}
