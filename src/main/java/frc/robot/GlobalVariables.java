package frc.robot;

import edu.wpi.first.math.MathUtil;

public class GlobalVariables {
    public static OI.DriveSystem driveSystem = OI.DriveSystem.TWO_SWITCH;

    public static boolean autoTarget = false;
    public static boolean TankDrive = false;
    public static double LastColorSensorDetectTime = 0;
    public static double TargetDistance = 1;
    public static double DistanceOffset = -0.5;
    public static boolean isReadyToFire = false;
    public static boolean readyOverride = true;
    public static boolean offsetOverride = true;
    public static boolean goodBall = true;

    public static void setDistance(double distance) {
        if (distance > 0) {
            TargetDistance = 1.43881 * Math.log(distance + 0.976185) + 1.0281 + DistanceOffset;
        }
        TargetDistance = MathUtil.clamp(TargetDistance, 0, 35);
    }


    // High goal at Buckeye
    public static double getVertSetpoint() {
        return TargetDistance >= 0 ? 522.242 * TargetDistance - 249.601 : 3000;
    }

    public static double getBackShooterSpeed() {
        return TargetDistance >= 0 ? MathUtil.clamp(-361.564 * TargetDistance - 2074.3, -5700, 0) : -1500;
    }

    public static double getFrontShooterSpeed() {
        return TargetDistance >= 0 ? MathUtil.clamp(480.824 * TargetDistance + 1682.58, 0, 5700) : 1500;
    }

    // Low goal at vaul
/*
    public static double getVertSetpoint() {
        return 7500;
    }

    public static double getBackShooterSpeed() {
        return MathUtil.clamp(-603.854 * TargetDistance - 320.35, -6500, 0);
    }

    public static double getFrontShooterSpeed() {
        return MathUtil.clamp(603.854 * TargetDistance + 320.35, 0, 6500);
    }
*/
}
