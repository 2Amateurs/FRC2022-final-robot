package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveReverseModeCommand extends CommandBase {
    private final DriveSubsystem subsystem = DriveSubsystem.getInstance();
    private boolean reverse = true;

    public DriveReverseModeCommand() {
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        if (reverse) {
            subsystem.setCurrentDriveMode(Constants.Reversed);
        } else {
            subsystem.setCurrentDriveMode();
        }
        reverse = !reverse;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
