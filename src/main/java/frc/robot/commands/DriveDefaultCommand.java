package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDefaultCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystem subsystem;
    private final OI oi = OI.getInstance();

    public DriveDefaultCommand() {
        this.subsystem = DriveSubsystem.getInstance();
        addRequirements(subsystem);
    }


    @Override
    public void execute() {
        subsystem.drive(oi.leftDriveAxis.getValue(), oi.rightDriveAxis.getValue(), oi.leftTurnAxis.getValue());
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
