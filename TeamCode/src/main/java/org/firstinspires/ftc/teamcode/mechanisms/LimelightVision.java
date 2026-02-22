package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Mechanism wrapper for the Limelight 3A camera.
 * Provides MegaTag2-based localization and DECODE obelisk motif detection.
 */
public class LimelightVision {

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

    private Limelight3A limelight;
    private LLResult latestResult;

    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    /**
     * Feed the robot's current IMU heading to the Limelight for MegaTag2 accuracy.
     * Call this every loop before reading the pose.
     */
    public void updateRobotHeading(double headingDegrees) {
        limelight.updateRobotOrientation(headingDegrees);
    }

    /**
     * Refreshes the cached result from the Limelight. Call once per loop.
     */
    public void update() {
        latestResult = limelight.getLatestResult();
    }

    /**
     * Returns true if the latest result contains valid data.
     */
    public boolean isResultValid() {
        return latestResult != null && latestResult.isValid();
    }

    /**
     * Returns the MegaTag2 robot pose on the field, or null if unavailable.
     */
    public Pose3D getLatestPose() {
        if (!isResultValid()) {
            return null;
        }
        return latestResult.getBotpose_MT2();
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
