package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStopCommand extends CommandBase {
    private final DriveSubsystem subsystem = DriveSubsystem.getInstance();

    public DriveStopCommand() {
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setCurrentDriveMode(Constants.RawInput);
        subsystem.drive(0, 0, 0);
        subsystem.setCurrentDriveMode();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
