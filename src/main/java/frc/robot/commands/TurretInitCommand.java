package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretInitCommand extends CommandBase {
    private final TurretSubsystem subsystem = TurretSubsystem.getInstance();
    private final Timer timer = new Timer();

    public TurretInitCommand() {
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        subsystem.vertAim(-1, true);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.vertAim(0, false);
        subsystem.resetVertEncoder();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= 10 || subsystem.getVertLimitSwitch();
    }
}
