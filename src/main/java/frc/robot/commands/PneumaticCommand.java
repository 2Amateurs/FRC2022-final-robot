package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticSubsystem;

public class PneumaticCommand extends CommandBase {
    private final PneumaticSubsystem subsystem = PneumaticSubsystem.getInstance();
    private final PneumaticSubsystem.Mode mode;
    private boolean toggle = true;

    public PneumaticCommand(PneumaticSubsystem.Mode mode) {
        addRequirements(subsystem);
        this.mode = mode;
    }

    @Override
    public void initialize() {
        if (mode == PneumaticSubsystem.Mode.TOGGLE) {
            if (toggle) {
                subsystem.toggle(PneumaticSubsystem.Mode.EXTEND);
            } else {
                subsystem.toggle(PneumaticSubsystem.Mode.RETRACT);
            }
            toggle = !toggle;
        } else {
            subsystem.toggle(mode);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
