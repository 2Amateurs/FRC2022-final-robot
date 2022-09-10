package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.commands.PneumaticDefaultCommand;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class PneumaticSubsystem extends SubsystemBase {
    public static PneumaticSubsystem instance = null;

    public static Mode mode = Mode.RETRACT;
    private final Compressor compressor = new Compressor(Constants.pneumaticControlCanID, PneumaticsModuleType.CTREPCM);
    private final DoubleSolenoid leftSolenoid = new DoubleSolenoid(Constants.pneumaticControlCanID,
            PneumaticsModuleType.CTREPCM, 0, 1);
    private final DoubleSolenoid rightSolenoid = new DoubleSolenoid(Constants.pneumaticControlCanID,
            PneumaticsModuleType.CTREPCM, 2, 3);

    private PneumaticSubsystem() {
        leftSolenoid.set(kOff);
        rightSolenoid.set(kOff);
    }

    public void toggle(Mode mode) {
        switch (mode) {
            case EXTEND:
                SmartDashboard.putBoolean("Intake Out", true);
                leftSolenoid.set(kForward);
                rightSolenoid.set(kForward);
                break;
            case RETRACT:
                SmartDashboard.putBoolean("Intake Out", false);
                leftSolenoid.set(kReverse);
                rightSolenoid.set(kReverse);
                break;
        }
        PneumaticSubsystem.mode = mode;
    }

    public static PneumaticSubsystem getInstance() {
        if (instance == null) {
            instance = new PneumaticSubsystem();
            instance.setDefaultCommand(new PneumaticDefaultCommand());
        }

        return instance;
    }

    public enum Mode {
        EXTEND,
        RETRACT,
        TOGGLE
    }
}
