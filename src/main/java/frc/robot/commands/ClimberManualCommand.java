package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ClimberManualCommand extends CommandBase {
    private final ClimberSubsystem subsystem = ClimberSubsystem.getInstance();
    private final ClimberSubsystem.Side side;
    private final double speed;

    public ClimberManualCommand(ClimberSubsystem.Side side, double speed) {
        this.side = side;
        this.speed = speed;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        if (!Constants.OUTREACH_MODE) {
            TurretSubsystem.getInstance().horBack();
            switch (side) {
                case LEFT:
                    subsystem.leftMotor(speed);
                    break;
                case RIGHT:
                    subsystem.rightMotor(speed);
                    break;
                case BOTH:
                    subsystem.leftMotor(speed);
                    subsystem.rightMotor(speed);
                    break;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
