package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class JeffConfig
{
    public static class AimConfig
    {
        public static double kP = 1/Math.toRadians(75);
        public static double kD = 0.0001;
    }
}
