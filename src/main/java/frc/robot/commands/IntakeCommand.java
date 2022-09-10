package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem subsystem = IntakeSubsystem.getInstance();
    private final IntakeSubsystem.Mode mode;
    private boolean intake = true;

    public IntakeCommand(IntakeSubsystem.Mode mode) {
        this.mode = mode;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        if (mode == IntakeSubsystem.Mode.TOGGLE) {
            if (intake) {
                subsystem.intake(IntakeSubsystem.Mode.INTAKE);
            } else {
                subsystem.intake(IntakeSubsystem.Mode.OFF);
            }
            intake = !intake;
        } else {
            subsystem.intake(mode);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
