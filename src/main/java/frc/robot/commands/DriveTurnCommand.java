package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class DriveTurnCommand extends CommandBase {
    private final DriveSubsystem subsystem = DriveSubsystem.getInstance();
    private final DoubleSupplier turn;

    public DriveTurnCommand(DoubleSupplier turn) {
        this.turn = turn;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.resetEncoders();
        subsystem.setCurrentDriveMode(Constants.FastMode);
    }

    @Override
    public void execute() {
        double turn = this.turn.getAsDouble();
        subsystem.drive(-turn, turn, 0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
