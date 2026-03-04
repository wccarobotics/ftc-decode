package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.LimelightVision;
import org.firstinspires.ftc.teamcode.opmodes.JeffBase;

public class AimCommand extends Command {
    private Follower follower;
    private LimelightVision limelight;
    private double targetHeading;
    private Pose targetPose;
    private Telemetry telemetry;
    private boolean sawTag = false;
    private ElapsedTime timeOut = new ElapsedTime();
    double relativeheading = 0;
    public AimCommand(Follower follower, LimelightVision limelight, Telemetry telemetry){
        this.follower = follower;
        this.limelight = limelight;
        this.telemetry = telemetry;
    }
    @Override
    public void initialize(){
        timeOut.reset();
    }
    @Override
    public void execute(){
        if(!sawTag) {
            telemetry.addLine("looking");
            int goalTag = JeffBase.currentAlliance == JeffBase.Alliance.BLUE ? LimelightVision.BLUE_GOAL_TAG_ID : LimelightVision.RED_GOAL_TAG_ID;
            for (LLResultTypes.FiducialResult fr : limelight.getFiducialResults()) {
                telemetry.addData("saw", fr.getFiducialId());
                if (fr.getFiducialId() == goalTag) {
                    relativeheading = Math.atan2(fr.getTargetPoseCameraSpace().getPosition().z, fr.getTargetPoseCameraSpace().getPosition().x) - Math.toRadians(90);

                    targetHeading = follower.getPose().getHeading() + relativeheading;
                    targetPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), targetHeading);
                    double error = targetHeading;
                    sawTag = true;
                    telemetry.addData("sawtag",sawTag);
//                    Pose startPose = follower.getPose();
//
//                    Path path = new Path(new BezierLine(startPose, targetPose));
//
//                    path.setLinearHeadingInterpolation(startPose.getHeading(), targetPose.getHeading());
//
//                    PathChain pathChain = follower.pathBuilder().addPath(path).build();
//
//
//
//                    follower.followPath(pathChain, 1, false);
                }
            }
        }
        else {
            telemetry.addData("turning", Math.toDegrees(relativeheading));
            telemetry.addData("heading error", Math.toDegrees(follower.getHeadingError()));
        }
    }
    @Override
    public boolean isFinished() {
        if (sawTag){
            return !follower.isBusy();
        }
        else {
            if (timeOut.seconds() > 5) return true;
            else return false;
        }
    }
}
