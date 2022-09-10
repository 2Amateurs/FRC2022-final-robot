package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretDistanceCommand extends CommandBase {
    private final TurretSubsystem subsystem = TurretSubsystem.getInstance();
    private final boolean increase;

    public TurretDistanceCommand(boolean increase) {
        this.increase = increase;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.distanceChange(increase);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}