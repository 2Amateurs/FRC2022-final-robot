package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretAutoToggleCommand extends CommandBase {
    private final TurretSubsystem subsystem = TurretSubsystem.getInstance();

    public TurretAutoToggleCommand() {
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.toggleGuidance();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
