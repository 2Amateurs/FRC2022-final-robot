package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem instance = null;

    private final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.intakeMotorPort);

    private IntakeSubsystem() {
//        SmartDashboard.putNumber("IntakeSpeed", Constants.intakeSpeed);
    }

    public void intake(Mode mode) {
//        Constants.intakeSpeed = SmartDashboard.getNumber("IntakeSpeed", Constants.intakeSpeed);
        if (PneumaticSubsystem.mode == PneumaticSubsystem.Mode.RETRACT) {
            mode = Mode.OFF;
        }
        switch (mode) {
            case INTAKE:
                intakeMotor.set(ControlMode.PercentOutput, Constants.intakeSpeed);
                break;
            case EXPEL:
                intakeMotor.set(ControlMode.PercentOutput, Constants.expelSpeed);
                break;
            case OFF:
                intakeMotor.set(ControlMode.PercentOutput, 0);
                break;
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (PneumaticSubsystem.mode == PneumaticSubsystem.Mode.RETRACT) {
            intake(Mode.OFF);
        }
    }

    public static IntakeSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakeSubsystem();
        }

        return instance;
    }

    public enum Mode {
        INTAKE,
        EXPEL,
        OFF,
        TOGGLE
    }
}
