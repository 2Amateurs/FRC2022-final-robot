package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

public class ColorSensorSubsystem extends SubsystemBase {
    private static ColorSensorSubsystem instance = null;

    final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    public ColorSensorSubsystem() {
        SmartDashboard.putBoolean("Offset Override", GlobalVariables.offsetOverride);
    }

    @Override
    public void periodic() {
        Color detectedColor = colorSensor.getColor();
        if (Constants.DEBUG_MODE) {
            SmartDashboard.putNumber("Red", detectedColor.red);
            SmartDashboard.putNumber("Green", detectedColor.green);
            SmartDashboard.putNumber("Blue", detectedColor.blue);
        }

        DriverStation.Alliance color = DriverStation.getAlliance();
//        SmartDashboard.putString("Alliance Color", color.toString());
        GlobalVariables.offsetOverride = SmartDashboard.getBoolean("Offset Override", true);

        //accurate red values - r: 0.56, g: 0.33, b: 0.10
        if (GlobalVariables.offsetOverride) {
            if ((detectedColor.red > 0.36 && detectedColor.red < 0.76) &&
                    (detectedColor.green > 0.13 && detectedColor.green < 0.53) &&
                    (detectedColor.blue > 0 && detectedColor.blue < 0.30)) {
                SmartDashboard.putString("Detected Color", "Red");
                SmartDashboard.putString("Match", (color == DriverStation.Alliance.Red) ? "Good Ball" : "Bad Ball");
                if (DriverStation.Alliance.Red == color) {
//                    TurretSubsystem.offsetYaw = 0;
                    GlobalVariables.goodBall = true;
                } else if (DriverStation.Alliance.Blue == color) {
//                    TurretSubsystem.offsetYaw = Constants.WrongColorOffset;
                    GlobalVariables.goodBall = false;
                }
                GlobalVariables.LastColorSensorDetectTime = Timer.getFPGATimestamp();
            } //accurate blue values - r: 0.15, g: 0.41, b: 0.45
            else if ((detectedColor.red > 0 && detectedColor.red < 0.35) &&
                    (detectedColor.green > 0.21 && detectedColor.green < 0.61) &&
                    (detectedColor.blue > 0.25 && detectedColor.blue < 0.65)) {
                SmartDashboard.putString("Detected Color", "Blue");
                SmartDashboard.putString("Match", (color == DriverStation.Alliance.Blue) ? "Good Ball" : "Bad Ball");
                if (DriverStation.Alliance.Blue == color) {
//                    TurretSubsystem.offsetYaw = 0;
                    GlobalVariables.goodBall = true;
                } else if (DriverStation.Alliance.Red == color) {
//                    TurretSubsystem.offsetYaw = Constants.WrongColorOffset;
                    GlobalVariables.goodBall = false;
                }
                GlobalVariables.LastColorSensorDetectTime = Timer.getFPGATimestamp();
            } else {
                SmartDashboard.putString("Detected Color", "N/A");
                if ((Timer.getFPGATimestamp() - GlobalVariables.LastColorSensorDetectTime) > Constants.MinStorageFeedDelay) {
//                    TurretSubsystem.offsetYaw = 0;
                    GlobalVariables.goodBall = true;
                }
            }
        }
        else {
//            TurretSubsystem.offsetYaw = 0;
            GlobalVariables.goodBall = true;
            GlobalVariables.LastColorSensorDetectTime = Timer.getFPGATimestamp();
        }
    }

    public static ColorSensorSubsystem getInstance() {
        if (instance == null) {
            instance = new ColorSensorSubsystem();
        }

        return instance;
    }
}
