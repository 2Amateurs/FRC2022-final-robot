package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class AutomaticShootCommand extends CommandBase {
    private final double delay;
    private double startTimestamp;
    private boolean isFinished;
    private final ShooterSubsystem shooterSubsystem;
    private final StorageSubsystem storageSubsystem;

    public AutomaticShootCommand(double delay) {
        shooterSubsystem = ShooterSubsystem.getInstance();
        storageSubsystem = StorageSubsystem.getInstance();
        this.delay = delay;
    }
    @Override
    public void initialize() {
        isFinished = false;
        startTimestamp = Robot.timer.get();
    }
    @Override
    public void execute() {
        storageSubsystem.feed(StorageSubsystem.Direction.IN);
        if (Robot.timer.get() - startTimestamp < delay) {
            shooterSubsystem.shoot(true);
        } else {
            isFinished = true;
        }
    }
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
