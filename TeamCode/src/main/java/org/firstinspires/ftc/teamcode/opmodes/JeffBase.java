package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.LimelightVision;
import org.firstinspires.ftc.teamcode.mechanisms.ScoringRI3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PanelsDrawing;

public abstract class JeffBase extends OpMode {

    protected ScoringRI3D scoring = new ScoringRI3D();
    protected Follower follower;
    protected TelemetryManager telemetryM;

    protected LimelightVision vision = new LimelightVision();

    protected enum Alliance{
        BLUE,
        RED
    }
    protected Alliance currentAlliance = Alliance.BLUE;

    protected Pose goalTarget = new Pose(0, 138);

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        PanelsDrawing.init();
        scoring.init(hardwareMap, telemetry);
        vision.init(hardwareMap, follower);


        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        if (gamepad1.yWasPressed()){
            if (currentAlliance == Alliance.BLUE){
                currentAlliance = Alliance.RED;
            }
            else {
                currentAlliance = Alliance.BLUE;
            }
        }
        telemetry.addData("ALLIANCE", currentAlliance);
    }

    @Override
    public void start()
    {
        if (currentAlliance == Alliance.RED){
            goalTarget = goalTarget.mirror();
        }
    }
}
