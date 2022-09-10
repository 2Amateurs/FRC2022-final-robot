package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.TurretSubsystem;

public class TurretDefaultCommand extends CommandBase {
    private final TurretSubsystem subsystem = TurretSubsystem.getInstance();
    private final OI oi = OI.getInstance();

    public TurretDefaultCommand() {
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.horAim(oi.turretHorAim.getValue());
        subsystem.vertAim(oi.turretVertAim.getValue(), false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}