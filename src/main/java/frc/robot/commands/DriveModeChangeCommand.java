package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveModeChangeCommand extends CommandBase {
    private final DriveSubsystem subsystem;
    private final boolean increase;

    public DriveModeChangeCommand(boolean increase) {
        this.subsystem = DriveSubsystem.getInstance();
        this.increase = increase;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        if(!Constants.SAFE_MODE) {
            subsystem.changeMode(increase);
        }

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
