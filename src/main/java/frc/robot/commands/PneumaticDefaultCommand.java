package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.PneumaticSubsystem;

public class PneumaticDefaultCommand extends CommandBase {
    private final PneumaticSubsystem subsystem = PneumaticSubsystem.getInstance();

    public PneumaticDefaultCommand() {
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if (OI.getInstance().pneumaticAxis.getValue() > 0.9) {
            subsystem.toggle(PneumaticSubsystem.Mode.RETRACT);
        } else if (OI.getInstance().pneumaticAxis.getValue() < -0.9) {
            subsystem.toggle(PneumaticSubsystem.Mode.EXTEND);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
