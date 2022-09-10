package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ClimberCommand extends CommandBase {
    private final ClimberSubsystem subsystem = ClimberSubsystem.getInstance();
    private final ClimberSubsystem.PIDMode pidMode;

    public ClimberCommand(ClimberSubsystem.PIDMode pidMode) {
        this.pidMode = pidMode;
    }

    @Override
    public void initialize() {
        switch (pidMode) {
            case EXTEND:
                subsystem.extend();
                break;
            case RETRACT:
                TurretSubsystem.getInstance().horBack();
                subsystem.retract();
                break;
            case STOP_PID:
                subsystem.stopPID();
                break;
            case START_PID:
                subsystem.startPID();
                break;
            case FREEZE_PID:
                subsystem.freezePID();
                break;
            case RESET_POS:
                subsystem.resetPos();
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
