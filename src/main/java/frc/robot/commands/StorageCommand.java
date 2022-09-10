package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StorageSubsystem;

public class StorageCommand extends CommandBase {
    private final StorageSubsystem subsystem = StorageSubsystem.getInstance();
    private final StorageSubsystem.Direction direction;

    public StorageCommand(StorageSubsystem.Direction direction) {
        this.direction = direction;
        addRequirements(subsystem);
    }
    @Override
    public void initialize() {
        subsystem.feed(direction);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
