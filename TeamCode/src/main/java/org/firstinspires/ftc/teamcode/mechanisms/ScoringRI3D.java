package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ScoringRI3D {
    final double FEED_TIME_SECONDS = 1; //The feeder servos run this long when a shot is requested.
    final double FEED_DELAY = .25; //time before feeders turn off after ball is gone
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1200; //in ticks/second for the close goal.
    final double LAUNCHER_CLOSE_MIN_VELOCITY = 1175; //minimum required to start a shot for close goal.

    final double LAUNCHER_FAR_TARGET_VELOCITY = 1350; //Target velocity for far goal
    final double LAUNCHER_FAR_MIN_VELOCITY = 1325; //minimum required to start a shot for far goal.

    double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY; //These variables allow
    double launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;

    boolean reverseIntake = false;

    boolean launcherOn = false;

    final double LEFT_POSITION = .35; //the left and right position for the diverter servo
    final double RIGHT_POSITION = .64;

    Telemetry telemetry = null;
    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;
    private DcMotorEx intake = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private Servo diverter = null;

    private RevColorSensorV3 leftColorSensor = null;
    private RevColorSensorV3 rightColorSensor = null;
    private RevColorSensorV3 leftFrontColorSensor = null;
    private RevColorSensorV3 rightFrontColorSensor = null;

    ElapsedTime leftFeederTimer = new ElapsedTime();
    ElapsedTime rightFeederTimer = new ElapsedTime();

    ElapsedTime rightBallTimer = new ElapsedTime();
    ElapsedTime leftBallTimer = new ElapsedTime();

    public enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState leftLaunchState;
    private LaunchState rightLaunchState;

    private enum DiverterDirection {
        LEFT,
        RIGHT;
    }
    private DiverterDirection diverterDirection = DiverterDirection.LEFT;

    private enum IntakeState {
        ON,
        OFF;
    }
    private IntakeState intakeState = IntakeState.OFF;

    private enum LauncherDistance {
        CLOSE,
        FAR;
    }


    private LauncherDistance launcherDistance = LauncherDistance.CLOSE;

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        leftLaunchState = LaunchState.IDLE;
        rightLaunchState = LaunchState.IDLE;

        leftLauncher = hardwareMap.get(DcMotorEx.class, "left_flywheel");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "right_flywheel");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        diverter = hardwareMap.get(Servo.class, "diverter");
        leftColorSensor = hardwareMap.get(RevColorSensorV3.class, "color_left");
        rightColorSensor = hardwareMap.get(RevColorSensorV3.class, "color_right");
        leftFrontColorSensor = hardwareMap.get(RevColorSensorV3.class, "color_left_front");
        rightFrontColorSensor = hardwareMap.get(RevColorSensorV3.class, "color_left_front");


        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */

        leftLauncher.setZeroPowerBehavior(BRAKE);
        rightLauncher.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void switchDiverter(){
        switch (diverterDirection) {
            case LEFT:
                diverterDirection = DiverterDirection.RIGHT;
                diverter.setPosition(RIGHT_POSITION);
                break;
            case RIGHT:
                diverterDirection = DiverterDirection.LEFT;
                diverter.setPosition(LEFT_POSITION);
                break;
        }
    }
    public void diverterLeft(){
        diverterDirection = DiverterDirection.LEFT;
        diverter.setPosition(LEFT_POSITION);
    }
    public void diverterRight(){
        diverterDirection = DiverterDirection.RIGHT;
        diverter.setPosition(RIGHT_POSITION);
    }
    public void changeDiverter(double change){
        diverter.setPosition(diverter.getPosition() + change);
    }
    public double diverterPose(){
        return diverter.getPosition();
    }
    public void runIntake(){
        double intakePower = reverseIntake? -1: 1;
        switch (intakeState){
            case ON:
                intakeState = IntakeState.OFF;
                intake.setPower(0);
                break;
            case OFF:
                intakeState = IntakeState.ON;
                intake.setPower(intakePower);
                break;
        }
    }
    public void switchIntake(){
        reverseIntake = !reverseIntake;
    }
    public void forwardIntake(){}
    public void intakeOff(){
        intake.setPower(0);
        intakeState = IntakeState.OFF;
    }
    public void intakeOn() {
        double intakePower = reverseIntake ? -1 : 1;
        intake.setPower(intakePower);
        intakeState = IntakeState.ON;
    }
    public void setIntakeSpeed(double speed){
        intake.setPower(speed);
    }
    public double intakeSpeed(){
        return intake.getVelocity();
    }
    public void setLaunchDistance(){
        switch (launcherDistance) {
            case CLOSE:
                launcherDistance = LauncherDistance.FAR;
                launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY;
                launcherMin = LAUNCHER_FAR_MIN_VELOCITY;
                break;
            case FAR:
                launcherDistance = LauncherDistance.CLOSE;
                launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
                launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;
                break;
        }
    }
    public void closeLaunch(){
        launcherDistance = LauncherDistance.CLOSE;
        launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
        launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;
    }
    public LauncherDistance getLauncherDistance() {
        return launcherDistance;
    }

    public void spinLauncher(){
        launcherOn = true;
    }
    public void stopLauncher(){
        launcherOn = false;
        if (((leftLaunchState != LaunchState.LAUNCH) || (leftLaunchState != LaunchState.LAUNCHING)) && ((rightLaunchState != LaunchState.LAUNCH) || (rightLaunchState != LaunchState.LAUNCHING))){
            leftLauncher.setVelocity(0);
            rightLauncher.setVelocity(0);
        }
    }
    public void updateFlyWheels(){
        if(launcherOn && ((leftLaunchState != LaunchState.LAUNCH) || (leftLaunchState != LaunchState.LAUNCHING)) && ((rightLaunchState != LaunchState.LAUNCH) || (rightLaunchState != LaunchState.LAUNCHING))){
            leftLauncher.setVelocity(launcherTarget);
            rightLauncher.setVelocity(launcherTarget);
        }
    }

    public void shootLeft(){
        if (leftLaunchState == LaunchState.IDLE){
            leftLaunchState = LaunchState.SPIN_UP;
        }
    }
    public void updateLeftLauncher(){
        switch (leftLaunchState) {
            case IDLE:
                break;
            case SPIN_UP:
                launcherOn = true;
                leftLauncher.setVelocity(launcherTarget);
                rightLauncher.setVelocity(launcherTarget);
                if (leftLauncher.getVelocity() > launcherMin) {
                    leftLaunchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                leftFeederTimer.reset();
                leftLaunchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if ((leftFeederTimer.seconds() > FEED_TIME_SECONDS * 3) || leftBallDelay()) {
                    leftLaunchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }
    public void shootRight(){
        if (rightLaunchState == LaunchState.IDLE){
            rightLaunchState = LaunchState.SPIN_UP;
        }
    }
    public void updateRightLauncher(){
        switch (rightLaunchState) {
            case IDLE:
                break;
            case SPIN_UP:
                launcherOn = true;
                leftLauncher.setVelocity(launcherTarget);
                rightLauncher.setVelocity(launcherTarget);
                if (leftLauncher.getVelocity() > launcherMin) {
                    rightLaunchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                rightFeeder.setPower(FULL_SPEED);
                rightFeederTimer.reset();
                rightLaunchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if ((rightFeederTimer.seconds() > FEED_TIME_SECONDS * 3) || rightBallDelay()) {
                    rightLaunchState = LaunchState.IDLE;
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }

    public LaunchState getLeftLaunchState() {
        return leftLaunchState;
    }
    public LaunchState getRightLaunchState() {
        return rightLaunchState;
    }

    public boolean leftBallDelay(){
        boolean canShoot = false;
        if (leftBall()){
            leftBallTimer.reset();
            canShoot = false;
        }
        else if (!leftBall() && leftBallTimer.seconds() >= FEED_DELAY){
            canShoot = true;
        }
        return canShoot;
    }
    public double flyWheelSpeed(){
        return (leftLauncher.getVelocity() + rightLauncher.getVelocity()) / 2;
    }

    public double leftBallDistance(){
        return leftColorSensor.getDistance(DistanceUnit.CM);
    }
    public boolean leftBall(){
        return (leftBallDistance() < 6);
    }
    public boolean rightBallDelay(){
        boolean canShoot = false;
        if (rightBall()){
            rightBallTimer.reset();
            canShoot = false;
        }
        else if (!rightBall() && rightBallTimer.seconds() >= FEED_DELAY){
            canShoot = true;
        }
        return canShoot;
    }
    public double rightBallDistance(){
        return rightColorSensor.getDistance(DistanceUnit.CM);
    }
    public boolean rightBall(){
        return (rightBallDistance() < 6);
    }
    public void updateAll(){
        updateFlyWheels();
        updateLeftLauncher();
        updateRightLauncher();

        NormalizedRGBA normColor = leftColorSensor.getNormalizedColors();

        telemetry.addData("left_color", "" + leftColorSensor.red() + ", " +
                leftColorSensor.green() + ", " + leftColorSensor.blue() + ", " +
                leftColorSensor.alpha());
        telemetry.addData("norm_color", "" + normColor.red + ", " +
                normColor.green + ", " + normColor.blue+ ", " +
                normColor.alpha);
        telemetry.addData("Left proximity", leftColorSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Right Proximity", rightColorSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Left Front Proximity", leftFrontColorSensor.getDistance(DistanceUnit.CM));
    }
}
