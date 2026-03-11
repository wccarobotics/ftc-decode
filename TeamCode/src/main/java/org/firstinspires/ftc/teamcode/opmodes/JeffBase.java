package org.firstinspires.ftc.teamcode.opmodes;

import android.content.Context;
import android.content.SharedPreferences;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.LoopTimer;
import org.firstinspires.ftc.teamcode.mechanisms.Lights;
import org.firstinspires.ftc.teamcode.mechanisms.LimelightVision;
import org.firstinspires.ftc.teamcode.mechanisms.JeffScoring;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsDrawing;

import java.util.ArrayList;
import java.util.List;

public abstract class JeffBase extends OpMode {

    protected SharedPreferences prefs;
    protected List<LynxModule> allHubs = null;
    protected JeffScoring scoring = new JeffScoring();
    protected Follower follower;
    protected Lights lights = new Lights();
    protected boolean hasBallBase;
    boolean pastBall = false;
    protected TelemetryManager telemetryM;

    protected LimelightVision vision = new LimelightVision();
    protected LoopTimer loopTimer = new LoopTimer();

    public enum Alliance{
        BLUE,
        RED
    }
    public static Alliance currentAlliance = Alliance.BLUE;

    protected Pose goalTarget = new Pose(0, 130);

    protected Pose savedPose;

    class PoseOption
    {
        public String name;
        public Pose pose;
        public Boolean shouldMirror;

        public PoseOption (String name, Pose pose, Boolean shouldMirror)
        {
            this.name = name;
            this.pose = pose;
            this.shouldMirror = shouldMirror;
        }
    }

    ArrayList<PoseOption> poseOptions;
    int selectedPoseIndex = 0;

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        follower = Constants.createFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        PanelsDrawing.init();
        scoring.init(hardwareMap, telemetry);
        vision.init(hardwareMap, follower, telemetry);
        lights.init(hardwareMap, scoring);
        prefs = hardwareMap.appContext.getSharedPreferences("FTCData", Context.MODE_PRIVATE);
        load();
        if (currentAlliance == Alliance.BLUE){
            lights.setAll(.611);
        }
        else lights.setAll(.28);

        poseOptions = new ArrayList<>();
        poseOptions.add(new PoseOption("Saved Pose", savedPose, false));
        poseOptions.add(new PoseOption("Center of Field", new Pose(72, 72, 0), false));
        poseOptions.add(new PoseOption("Near", new Pose(31.6,131.2, Math.toRadians(-180)), true));
        poseOptions.add(new PoseOption("Far", new Pose(56, 8, Math.toRadians(90)), true));


        telemetry.addData("Status", "Initialized");
    }

    public void save(){
        prefs.edit()
                .putString("alliance", currentAlliance == Alliance.BLUE? "blue": "red")
                .putFloat("PoseX",(float)follower.getPose().getX())
                .putFloat("PoseY", (float)follower.getPose().getY())
                .putFloat("Heading", (float)follower.getPose().getHeading())
                .apply();
    }
    public void load(){
        currentAlliance = prefs.getString("alliance", "blue").equals("blue")? Alliance.BLUE: Alliance.RED;
        savedPose = new Pose(
            (double)prefs.getFloat("PoseX", (float)22.6),
            (double)prefs.getFloat("PoseY", (float)128.4),
            (double)prefs.getFloat("Heading", (float)Math.toRadians(144))
        );
    }

    @Override
    public void init_loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        if (gamepad1.yWasPressed()){
            if (currentAlliance == Alliance.BLUE){
                currentAlliance = Alliance.RED;
                lights.setAll(.28);
            }
            else {
                currentAlliance = Alliance.BLUE;
                lights.setAll(.611);
            }
        }

        if (gamepad1.dpadUpWasPressed()){
            selectedPoseIndex++;
        }
        else if (gamepad1.dpadDownWasPressed()) {
            selectedPoseIndex--;
        }

        selectedPoseIndex = selectedPoseIndex % poseOptions.size();
        if (selectedPoseIndex < 0)
        {
            selectedPoseIndex += poseOptions.size();
        }

        PoseOption selectedPose = poseOptions.get(selectedPoseIndex);
        telemetry.addData("ALLIANCE", currentAlliance);
        telemetry.addData("Starting pose name", selectedPose.name);
        telemetry.addData("Starting pose", "%f %f %f",
                selectedPose.pose.getX(), selectedPose.pose.getY(), Math.toDegrees(selectedPose.pose.getHeading()));
    }

    @Override
    public void start()
    {
        lights.start(currentAlliance);
        PoseOption startingPoseOption = poseOptions.get(selectedPoseIndex);

        Pose startingPose = startingPoseOption.pose;

        if (currentAlliance == Alliance.RED){
            goalTarget = goalTarget.mirror();
            if (startingPoseOption.shouldMirror){
                startingPose = startingPose.mirror();
            }
        }

        follower.setStartingPose(startingPose);
        follower.update();

        save();
    }

    public Pose AimAt(Pose pose, Pose target)
    {
        double targetHeading = Math.atan2(target.getY() - pose.getY(), target.getX() - pose.getX());
        return pose.withHeading(targetHeading);
    }

    @Override
    public void loop(){
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        scoring.updateAll();
        loopTimer.update();
        if(scoring.rightBall() || scoring.leftBall()){
            hasBallBase = true;
        }
        if (hasBallBase ^ pastBall) {
            lights.ballColors();
        }
        if(scoring.rightBall() || scoring.leftBall()) {
            pastBall = true;
        }
        else if (!scoring.rightBall() && ! scoring.leftBall()){
            pastBall = false;
            }
        telemetry.addData("Loop (ms)", "avg %.1f / min %.1f / max %.1f",
            loopTimer.getAvgMs(), loopTimer.getMinMs(), loopTimer.getMaxMs());
        if (!scoring.leftBall() && !scoring.rightBall()){
            hasBallBase = false;
        }
    }

    @Override
    public void stop()
    {
        save();
    }
}
