package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {
    private final ShooterSubsystem subsystem = ShooterSubsystem.getInstance();
    private final boolean shoot;

    public ShooterCommand(boolean shoot) {
        this.shoot = shoot;
        addRequirements(this.subsystem);
    }

    @Override
    public void initialize() {
        subsystem.shoot(shoot);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
