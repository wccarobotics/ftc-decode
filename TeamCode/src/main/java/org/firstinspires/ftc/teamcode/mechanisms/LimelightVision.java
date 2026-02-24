package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Mechanism wrapper for the Limelight 3A camera.
 * Provides MegaTag2-based localization and DECODE obelisk motif detection.
 */
public class LimelightVision {

    //  Camera forward: -5 inches, -0.127 m
    //  Camera right: 1 inch, 0.0254 m
    //  Camera Up: 16 inches, 0.4064 m

    // DECODE field AprilTag IDs
    public static final int BLUE_GOAL_TAG_ID = 20;
    public static final int OBELISK_TAG_ID_1 = 21;
    public static final int OBELISK_TAG_ID_2 = 22;
    public static final int OBELISK_TAG_ID_3 = 23;
    public static final int RED_GOAL_TAG_ID = 24;

    /** The three obelisk motif patterns, mapped to AprilTag IDs 21-23. */
    public enum Motif {
        GPP,     // tag 21
        PGP,     // tag 22
        PPG,     // tag 23
        UNKNOWN
    }

    private static final double METERS_TO_INCHES = 39.3701;
    // Pedro Pathing origin is bottom-left corner; Limelight origin is field center.
    // FTC field is 144 inches, so offset is 72 inches.
    private static final double FIELD_CENTER_OFFSET_INCHES = 144.0;

    private Limelight3A limelight;
    private Follower follower;
    private LLResult latestResult;

    private Telemetry telemetry;

    public void init(HardwareMap hardwareMap, Follower follower, Telemetry telemetry) {
        this.follower = follower;
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    /**
     * Feeds the follower's heading to the Limelight for MegaTag2 accuracy
     * and refreshes the cached result. Call once per loop.
     */
    public void update() {
        //  Subtract 90 degrees to convert from Pedro coordinate system to FTC coordinate system, which
        //  the camera uses.
        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()) - 90);
        latestResult = limelight.getLatestResult();
    }

    /**
     * Returns true if the latest result contains valid data.
     */
    public boolean isResultValid() {
        return latestResult != null && latestResult.isValid();
    }

    public Pose getLatestPose2()
    {
        if (!isResultValid())
        {
            return null;
        }


        List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducials) {
            if (fr.getFiducialId() == BLUE_GOAL_TAG_ID)
            {
                fr.getCameraPoseTargetSpace();
            }
        }
        return null;
    }

    /**
     * Returns the MegaTag2 robot pose converted to Pedro Pathing coordinates
     * (inches from bottom-left corner, heading in radians), or null if unavailable.
     */
    public Pose getLatestPose() {
        if (!isResultValid()) {
            return null;
        }
        Pose3D pose3d = latestResult.getBotpose_MT2();

        if (pose3d == null) {
            return null;
        }
        Pose2D pose2d = new Pose2D(pose3d.getPosition().unit, pose3d.getPosition().x, pose3d.getPosition().y,
                AngleUnit.RADIANS, pose3d.getOrientation().getYaw(AngleUnit.RADIANS));

        telemetry.addData("FTC Camera heading", pose2d.getHeading(AngleUnit.DEGREES));
        telemetry.addData("FTC Camera x", pose2d.getX(DistanceUnit.INCH));
        telemetry.addData("FTC Camera y", pose2d.getY(DistanceUnit.INCH));

        Pose ftcPose = PoseConverter.pose2DToPose(pose2d, InvertedFTCCoordinates.INSTANCE);
        telemetry.addData("FTC pose heading", Math.toDegrees(ftcPose.getHeading()));

        Pose pedroPose = ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        telemetry.addData("Pedro pose heading", Math.toDegrees(pedroPose.getHeading()));

        return pedroPose;
//
//        double xInches = pose3d.getPosition().x * METERS_TO_INCHES + FIELD_CENTER_OFFSET_INCHES;
//        double yInches = pose3d.getPosition().y * METERS_TO_INCHES + FIELD_CENTER_OFFSET_INCHES;
//        double headingRadians = Math.toRadians(pose3d.getOrientation().getYaw());
//        return new Pose(xInches, yInches, headingRadians);
    }

    /**
     * Checks the visible fiducial results for an obelisk AprilTag (IDs 21-23)
     * and returns the corresponding Motif.
     */
    public Motif getDetectedMotif() {
        if (!isResultValid()) {
            return Motif.UNKNOWN;
        }
        List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducials) {
            switch (fr.getFiducialId()) {
                case OBELISK_TAG_ID_1: return Motif.GPP;
                case OBELISK_TAG_ID_2: return Motif.PGP;
                case OBELISK_TAG_ID_3: return Motif.PPG;
            }
        }
        return Motif.UNKNOWN;
    }

    /** Horizontal offset to the primary target in degrees. */
    public double getTx() {
        return isResultValid() ? latestResult.getTx() : 0;
    }

    /** Vertical offset to the primary target in degrees. */
    public double getTy() {
        return isResultValid() ? latestResult.getTy() : 0;
    }

    /** Returns the raw fiducial results list, or an empty list if invalid. */
    public List<LLResultTypes.FiducialResult> getFiducialResults() {
        if (!isResultValid()) {
            return java.util.Collections.emptyList();
        }
        return latestResult.getFiducialResults();
    }

    /** Switch to a different Limelight pipeline by index. */
    public void setPipeline(int index) {
        limelight.pipelineSwitch(index);
    }

    /** Stop Limelight polling. Call from OpMode stop(). */
    public void stop() {
        limelight.stop();
    }
}
