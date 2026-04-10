package org.firstinspires.ftc.teamcode.mechanisms;

import com.pedropathing.ftc.FTCCoordinates;
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

    // Goal tag positions from DECODE .fmap (meters, FTC center-origin)
    private static final double BLUE_GOAL_X_M = -1.4827;
    private static final double BLUE_GOAL_Y_M = -1.4133;
    private static final double RED_GOAL_X_M = -1.4827;
    private static final double RED_GOAL_Y_M = 1.4133;

    // Goal tag facing angles from .fmap rotation matrices (radians, FTC field coords)
    private static final double BLUE_GOAL_HEADING_RAD = Math.atan2(0.8090169943749473, 0.5877852522924731);
    private static final double RED_GOAL_HEADING_RAD = Math.atan2(-0.8090169943749473, 0.5877852522924731);

    // Camera position relative to robot center (meters)
    private static final double CAMERA_FORWARD_M = -0.127;   // -5 inches behind center
    private static final double CAMERA_RIGHT_M = 0.0254;     // 1 inch right of center

    private Limelight3A limelight;
    private Follower follower;
    private LLResult latestResult;

    private Telemetry telemetry;

    // Pre-computed tag positions in Pedro coordinates
    private Pose blueGoalPedro;
    private Pose redGoalPedro;

    public enum HeadingSource {
        FOLLOWER,  // Use the Pedro follower's IMU-fused heading (more stable)
        VISION     // Derive heading from the detected tag orientation (vision-only)
    }


    static Pose ftcToPedro(Pose2D ftcPose) {
//        final double FIELD_SIZE = 144.0;  // inches
//        final double FIELD_CENTER = 72.0;  // inches
//        final double INCHES_PER_METER = 39.3701;
//
//        // Convert meters to inches and shift origin
//        double pedroX = (ftcPose.getX(DistanceUnit.METER) * INCHES_PER_METER) + FIELD_CENTER;
//        double pedroY = (ftcPose.getY(DistanceUnit.METER) * INCHES_PER_METER) + FIELD_CENTER;
//        double headingRad = Math.toRadians(ftcPose.getHeading(AngleUnit.DEGREES));
//
//        return new Pose(pedroX, pedroY, headingRad);
        return PoseConverter.pose2DToPose(ftcPose, FTCCoordinates.INSTANCE)
                .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }


    public void init(HardwareMap hardwareMap, Follower follower, Telemetry telemetry) {
        this.follower = follower;
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        // Pre-compute goal tag positions in Pedro coordinates
        Pose2D blueGoalFTC = new Pose2D(DistanceUnit.METER, BLUE_GOAL_X_M, BLUE_GOAL_Y_M, AngleUnit.DEGREES, 54);
        blueGoalPedro = ftcToPedro(blueGoalFTC);

        Pose2D redGoalFTC = new Pose2D(DistanceUnit.METER, RED_GOAL_X_M, RED_GOAL_Y_M, AngleUnit.DEGREES, -54);
        redGoalPedro = ftcToPedro(redGoalFTC);
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

    /**
     * Computes robot pose from a single detected goal AprilTag using manual 2D transforms.
     * Does not require a .fmap file on the Limelight — tag positions are hard-coded.
     * @param headingSource whether to use the follower heading or derive it from the tag
     * @return a Pedro Pose, or null if no goal tag is visible.
     */
    public Pose getLatestPose2(HeadingSource headingSource)
    {
        if (!isResultValid())
        {
            return null;
        }

        List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducials) {
            int id = fr.getFiducialId();
            Pose tagPedro;
            double tagFtcHeadingRad;
            if (id == BLUE_GOAL_TAG_ID) {
                tagPedro = blueGoalPedro;
                tagFtcHeadingRad = BLUE_GOAL_HEADING_RAD;
            } else if (id == RED_GOAL_TAG_ID) {
                tagPedro = redGoalPedro;
                tagFtcHeadingRad = RED_GOAL_HEADING_RAD;
            } else {
                continue;
            }

            telemetry.addData("Goal: ", "" + tagPedro.getX() + " " + tagPedro.getY());

            Pose3D targetInCam = fr.getTargetPoseCameraSpace();
            if (targetInCam == null) continue;

            // Camera space: z = forward, x = right (meters). Ignore y (height).
            double camForwardM = targetInCam.getPosition().z;
            double camRightM = targetInCam.getPosition().x;

            // Determine robot heading
            double heading;
            if (headingSource == HeadingSource.VISION) {
                // Tag yaw in camera space: how the tag is rotated relative to the camera.
                // Robot faces opposite the tag face, so add 180°.
                // Convert from FTC tag heading to Pedro heading by adding 90°.
                double tagYawInCamRad = Math.toRadians(targetInCam.getOrientation().getYaw());
                telemetry.addData("tagYawInCam", Math.toDegrees(tagYawInCamRad));
                heading = tagFtcHeadingRad - tagYawInCamRad + Math.PI + Math.PI / 2;
                // Normalize to [0, 2π)
                heading = ((heading % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
            } else {
                heading = follower.getHeading();
            }

            // Add camera-to-robot-center offset, then convert to inches
            double forwardIn = (camForwardM + CAMERA_FORWARD_M) * METERS_TO_INCHES;
            double rightIn = (camRightM + CAMERA_RIGHT_M) * METERS_TO_INCHES;

            telemetry.addData("Camera forward", forwardIn);
            telemetry.addData("Camera right", rightIn);

            // Rotate from robot frame to Pedro field frame using heading
            // Pedro: heading 0 = +X, CCW positive
            // Forward direction = (cos θ, sin θ), Right direction = (sin θ, -cos θ)
            double offsetX = forwardIn * Math.cos(heading) + rightIn * Math.sin(heading);
            double offsetY = forwardIn * Math.sin(heading) - rightIn * Math.cos(heading);

            double robotX = tagPedro.getX() - offsetX;
            double robotY = tagPedro.getY() - offsetY;

            return new Pose(robotX, robotY, heading);
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

        Pose ftcPose = PoseConverter.pose2DToPose(pose2d, FTCCoordinates.INSTANCE);
        telemetry.addData("FTC pose heading", Math.toDegrees(ftcPose.getHeading()));

        Pose pedroPose = ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        //Pose pedroPose = ftcToPedro(pose2d);
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

    public LLResultTypes.FiducialResult getAprilTag(int id)
    {
        if (!isResultValid()) {
            return null;
        }
        List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();
        for (LLResultTypes.FiducialResult fr : fiducials) {
            if (fr.getFiducialId() == id)
            {
                return fr;
            }
        }
        return null;
    }

//    public double getAngleToId(int id)
//    {
//        LLResultTypes.FiducialResult fr = getAprilTag(id);
//        if (fr == null)
//        {
//            return 0;
//        }
//
//    }

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
