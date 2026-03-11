package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class JeffScoring {


    static final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1200; //in ticks/second for the close goal.
    static final double LAUNCHER_CLOSE_MIN_VELOCITY = 1175; //minimum required to start a shot for close goal.

    static final double LAUNCHER_FAR_TARGET_VELOCITY = 1350; //Target velocity for far goal
    static final double LAUNCHER_FAR_MIN_VELOCITY = 1325; //minimum required to start a shot for far goal.

    boolean reverseIntake = false;

    double intakeVelocity;

    final double LEFT_POSITION = .35; //the left and right position for the diverter servo
    final double RIGHT_POSITION = .64;

    Telemetry telemetry = null;
    private DcMotorEx leftLauncherMotor = null;
    private DcMotorEx rightLauncherMotor = null;
    private DcMotorEx intake = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private Servo diverter = null;

    private RevColorSensorV3 leftColorSensor = null;
    private RevColorSensorV3 rightColorSensor = null;
    private RevColorSensorV3 leftFrontColorSensor = null;
    private RevColorSensorV3 rightFrontColorSensor = null;

    public enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private enum DiverterDirection {
        LEFT,
        RIGHT,
    }
    private DiverterDirection diverterDirection = DiverterDirection.LEFT;

    private double diverterLocation;

    private enum IntakeState {
        ON,
        OFF,
    }
    private IntakeState intakeState = IntakeState.OFF;

    private enum LauncherDistance {
        CLOSE,
        FAR,
    }

    private final JeffLauncher leftLauncher = new JeffLauncher();
    private final JeffLauncher rightLauncher = new JeffLauncher();

    private final Flywheel flywheel = new Flywheel();



    private LauncherDistance launcherDistance = LauncherDistance.CLOSE;

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        leftLauncherMotor = hardwareMap.get(DcMotorEx.class, "left_flywheel");
        rightLauncherMotor = hardwareMap.get(DcMotorEx.class, "right_flywheel");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        diverter = hardwareMap.get(Servo.class, "diverter");
        leftColorSensor = hardwareMap.get(RevColorSensorV3.class, "color_left");
        rightColorSensor = hardwareMap.get(RevColorSensorV3.class, "color_right");
        leftFrontColorSensor = hardwareMap.get(RevColorSensorV3.class, "color_left_front");
        rightFrontColorSensor = hardwareMap.get(RevColorSensorV3.class, "color_left_front");


        leftLauncherMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLauncherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */

        leftLauncherMotor.setZeroPowerBehavior(BRAKE);
        rightLauncherMotor.setZeroPowerBehavior(BRAKE);


        leftLauncherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        rightLauncherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel.init(leftLauncherMotor, rightLauncherMotor);

        leftLauncher.init(flywheel, leftFeeder);
        rightLauncher.init(flywheel, rightFeeder);
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
        return diverterLocation;
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
        return intakeVelocity;
    }
    public void setLaunchDistance(){
        switch (launcherDistance) {
            case CLOSE:
                launcherDistance = LauncherDistance.FAR;
                flywheel.setLaunchSpeed(LAUNCHER_FAR_TARGET_VELOCITY, LAUNCHER_FAR_MIN_VELOCITY);
                break;
            case FAR:
                launcherDistance = LauncherDistance.CLOSE;
                flywheel.setLaunchSpeed(LAUNCHER_CLOSE_TARGET_VELOCITY, LAUNCHER_CLOSE_MIN_VELOCITY);
                break;
        }
    }
    public void closeLaunch(){
        launcherDistance = LauncherDistance.CLOSE;
        flywheel.setLaunchSpeed(LAUNCHER_CLOSE_TARGET_VELOCITY, LAUNCHER_CLOSE_MIN_VELOCITY);
    }
    public LauncherDistance getLauncherDistance() {
        return launcherDistance;
    }

    public void spinLauncher(){
        flywheel.start();
    }
    public void stopLauncher() {
        flywheel.stop();

    }

    public void shootLeft(){
        leftLauncher.shoot();
    }
    public void shootRight(){
        rightLauncher.shoot();
    }

    public LaunchState getLeftLaunchState() {
        return leftLauncher.getLaunchState();
    }
    public LaunchState getRightLaunchState() {
        return rightLauncher.getLaunchState();
    }

    public boolean leftBall(){
        return leftLauncher.seesBall();
    }
    public double leftBallColor(){ // green averages 160 in home at night, purple is 227 at home during night
        float [] hsv = new float[3];
        Color.RGBToHSV(leftColorSensor.red(), leftColorSensor.green(), leftColorSensor.blue(), hsv);
        return hsv[0];
    }

    public boolean rightBall(){
        return rightLauncher.seesBall();
    }
    public double rightBallColor(){ // green averages 160 in home at night, purple is 227 at home during night
        float [] hsv = new float[3];
        Color.RGBToHSV(rightColorSensor.red(), rightColorSensor.green(), rightColorSensor.blue(), hsv);
        return hsv[0];
    }
    public double frontBallDistance(){
        if (diverterDirection == DiverterDirection.LEFT){
            return rightFrontColorSensor.getDistance(DistanceUnit.CM);
        }
        else return 1;
    }
    public void updateAll(){
        double leftDistance = leftColorSensor.getDistance(DistanceUnit.CM);
        double rightDistance = rightColorSensor.getDistance((DistanceUnit.CM));

        flywheel.update();
        leftLauncher.update(leftDistance);
        rightLauncher.update(rightDistance);

        intakeVelocity = intake.getVelocity();
        diverterLocation = diverter.getPosition();

//        NormalizedRGBA normColor = leftColorSensor.getNormalizedColors();
//
//        telemetry.addData("left_color", "" + leftColorSensor.red() + ", " +
//                leftColorSensor.green() + ", " + leftColorSensor.blue() + ", " +
//                leftColorSensor.alpha());
//        telemetry.addData("norm_color", "" + normColor.red + ", " +
//                normColor.green + ", " + normColor.blue+ ", " +
//                normColor.alpha);
        telemetry.addData("Left proximity", leftDistance);
        telemetry.addData("Right Proximity", rightDistance);
        telemetry.addData("Left Front Proximity", leftFrontColorSensor.getDistance(DistanceUnit.CM));
    }

    static class Flywheel
    {
        private DcMotorEx leftLauncher = null;
        private DcMotorEx rightLauncher = null;

        public void init(DcMotorEx leftLauncher, DcMotorEx rightLauncher)
        {
            this.leftLauncher = leftLauncher;
            this.rightLauncher = rightLauncher;
        }

        private boolean _launcherOn = false;

        private double _launcherTarget = JeffScoring.LAUNCHER_CLOSE_TARGET_VELOCITY; //These variables allow
        private double _launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;

        public void start()
        {
            _launcherOn = true;
            leftLauncher.setVelocity(_launcherTarget);
            rightLauncher.setVelocity(_launcherTarget);
        }

        public void stop()
        {
            _launcherOn = false;

            leftLauncher.setVelocity(0);
            rightLauncher.setVelocity(0);

        }

        public boolean hasReachedMinSpeed()
        {
            return leftLauncher.getVelocity() >= _launcherMin;
        }

        public void setLaunchSpeed(double target, double min)
        {
            _launcherTarget = target;
            _launcherMin = min;
        }

        public void update(){
            if(_launcherOn){
                leftLauncher.setVelocity(_launcherTarget);
                rightLauncher.setVelocity(_launcherTarget);
            }
        }

    }

}
