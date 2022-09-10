package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

public class StorageSubsystem extends SubsystemBase {
    public static StorageSubsystem instance = null;

    private final CANSparkMax feeder = new CANSparkMax(Constants.feederMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);

    private StorageSubsystem() {
        feeder.setSmartCurrentLimit(20);
    }

    public void feed(Direction direction) {
        double timeSinceLastDetect = Timer.getFPGATimestamp() - GlobalVariables.LastColorSensorDetectTime;
        boolean recentDetect = timeSinceLastDetect < Constants.MinStorageFeedDelay;
        SmartDashboard.putBoolean("Recent Ammo Detect", recentDetect);
        switch (direction) {
            case IN:
                if (GlobalVariables.isReadyToFire || recentDetect) {
                    feeder.set(Constants.feederInSpeed);
                }
                break;
            case OUT:
                feeder.set(Constants.feederOutSpeed);
                break;
            case STOP:
                feeder.set(0);
                break;
        }
    }

    public static StorageSubsystem getInstance() {
        if (instance == null) {
            instance = new StorageSubsystem();
        }

        return instance;
    }

    public enum Direction {
        IN,
        OUT,
        STOP
    }
}
