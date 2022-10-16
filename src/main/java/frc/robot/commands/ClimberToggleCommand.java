package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ClimberToggleCommand extends CommandBase {
    private final ClimberSubsystem subsystem = ClimberSubsystem.getInstance();

    public ClimberToggleCommand(){}

    @Override
    public void initialize() {
        GlobalVariables.climberEnabled = ! GlobalVariables.climberEnabled;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
