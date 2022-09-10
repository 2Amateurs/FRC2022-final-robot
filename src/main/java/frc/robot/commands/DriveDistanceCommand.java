package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistanceCommand extends CommandBase {
    private final DriveSubsystem subsystem = DriveSubsystem.getInstance();
    private final double distance;
    private final double speed;

    public DriveDistanceCommand(double distance, double speed) {
        this.distance = distance;
        this.speed = speed;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.resetEncoders();
        subsystem.setCurrentDriveMode(Constants.FastMode);
    }

    @Override
    public void execute() {
        subsystem.drive(-speed, -speed, 0);
    }

    @Override
    public boolean isFinished() {
        return subsystem.getEncoderAveragePos() >= distance;
    }
}
