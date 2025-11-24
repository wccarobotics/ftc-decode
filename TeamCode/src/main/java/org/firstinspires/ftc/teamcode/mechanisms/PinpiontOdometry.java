package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

public class PinpiontOdometry {
    GoBildaPinpointDriver odo;
    public void init(HardwareMap hardwareMap){
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-59.1, -120.7, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
    }
    public void newUpdateOutNow(){
        odo.update();
    }
    public double getX(){
        Pose2D pos = odo.getPosition();
        return pos.getX(DistanceUnit.MM);
    }
    public double getY(){
        Pose2D pos = odo.getPosition();
        return pos.getY(DistanceUnit.MM);
    }
    public double getHeading(){
        Pose2D pos = odo.getPosition();
        return pos.getHeading(AngleUnit.DEGREES);
    }
    public void resetImu(){
        odo.recalibrateIMU();
    }
    public void resetEverything(){
        odo.resetPosAndIMU();
    }
}
