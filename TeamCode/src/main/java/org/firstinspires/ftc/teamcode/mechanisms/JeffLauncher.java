package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class JeffLauncher {

    static final double FEED_TIME_SECONDS = 1; //The feeder servos run this long when a shot is requested.
    static final double FEED_DELAY = .25; //time before feeders turn off after ball is gone
    static final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    static final double FULL_SPEED = 1.0;

    private JeffScoring.Flywheel flywheel = null;

    private CRServo feeder = null;

    ElapsedTime ballTimer = new ElapsedTime();

    private double distance;

    private JeffScoring.LaunchState launchState = JeffScoring.LaunchState.IDLE;

    ElapsedTime feederTimer = new ElapsedTime();

    public void init(JeffScoring.Flywheel flywheel, CRServo feeder)
    {
        this.flywheel = flywheel;
        this.feeder = feeder;

        feeder.setPower(STOP_SPEED);
    }

    public void update(double ballDistance)
    {
        this.distance = ballDistance;

        switch (launchState) {
            case IDLE:
                break;
            case SPIN_UP:
                flywheel.start();
                if (flywheel.hasReachedMinSpeed()) {
                    launchState = JeffScoring.LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                feeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = JeffScoring.LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if ((feederTimer.seconds() > FEED_TIME_SECONDS * 3) || ballDelay()) {
                    launchState = JeffScoring.LaunchState.IDLE;
                    feeder.setPower(STOP_SPEED);
                }
                break;
        }
    }

    public void shoot(){
        if (launchState == JeffScoring.LaunchState.IDLE){
            launchState = JeffScoring.LaunchState.SPIN_UP;
        }
    }

    public boolean ballDelay(){
        boolean canShoot = false;
        if (seesBall()){
            ballTimer.reset();
            canShoot = false;
        }
        else if (!seesBall() && ballTimer.seconds() >= FEED_DELAY){
            canShoot = true;
        }
        return canShoot;
    }

    public double ballDistance(){
        return distance;
    }
    public boolean seesBall(){
        return (ballDistance() < 6);
    }

    public JeffScoring.LaunchState getLaunchState()
    {
        return launchState;
    }
}
