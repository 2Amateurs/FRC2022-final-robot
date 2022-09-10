package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.OI;

public class ClimberSubsystem extends SubsystemBase {
    private static ClimberSubsystem instance = null;

    //motor types not final
    private boolean enabled = true;
    private final CANSparkMax leftClimber = new CANSparkMax(Constants.leftClimberMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightClimber = new CANSparkMax(Constants.rightClimberMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder leftEncoder = leftClimber.getEncoder();
    private final RelativeEncoder rightEncoder = rightClimber.getEncoder();
    private final SparkMaxPIDController leftController = leftClimber.getPIDController();
    private final SparkMaxPIDController rightController = rightClimber.getPIDController();
    private final double kp = 0.3, ki = 0.01, kd = 0.0;
    private boolean pid = false;
    private double leftSetpoint = 0;
    private double rightSetpoint = 0;
    private double targetLeftSetpoint = 0;
    private double targetRightSetpoint = 0;
    private boolean reset = false;
    private int sign = 1;
    private double positiveLeftLimit = 0;
    private double positiveRightLimit = 0;
    private double negativeLeftLimit = 0;
    private double negativeRightLimit = 0;
    private double negativeStartTime = 0;

    private ClimberSubsystem() {
        if (Constants.DEBUG_MODE) {
            SmartDashboard.putNumber("Climber P Value", kp);
            SmartDashboard.putNumber("Climber I Value", ki);
            SmartDashboard.putNumber("Climber Speed Clamp Value", Constants.ClimberSpeed);
        }
        leftClimber.setSmartCurrentLimit(40, 80, 100);
        rightClimber.setSmartCurrentLimit(40, 80, 100);
        leftController.setP(kp);
        leftController.setI(ki);
        leftController.setD(kd);
        rightController.setP(kp);
        rightController.setI(ki);
        rightController.setD(kd);
        leftClimber.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightClimber.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void startPID() {
        pid = true;
    }

    public void stopPID() {
        pid = false;
    }

    public void rightMotor(double speed) {
        pid = false;
        resetPos();
        rightClimber.set(speed);
    }

    public void leftMotor(double speed) {
        pid = false;
        resetPos();
        leftClimber.set(speed);
    }

    public void bothMotors(double speed) {
        leftMotor(speed);
        rightMotor(speed);
    }

    public boolean findZero(double timeSinceStart) {
        startPID();
        reset = true;
        double leftVelocity = Math.abs(leftEncoder.getVelocity());
        double rightVelocity = Math.abs(rightEncoder.getVelocity());
        double leftPos = leftEncoder.getPosition();
        double rightPos = rightEncoder.getPosition();
        double leftErr = Math.abs(leftSetpoint - leftPos);
        double rightErr = Math.abs(rightSetpoint - rightPos);
        if (sign == 1) {
            if ((leftVelocity > 0.01 && leftErr < 0.5) || timeSinceStart < 1) {
                leftSetpoint += 0.1;
            } else {
                leftSetpoint = leftPos;
                positiveLeftLimit = leftPos;
            }
            if ((rightVelocity > 0.01 && rightErr < 0.5) || timeSinceStart < 1) {
                rightSetpoint += 0.1;
            } else {
                rightSetpoint = rightPos;
                positiveRightLimit = rightPos;
            }
            if (leftErr < 0.01 && rightErr < 0.01) {
                sign = -1;
                negativeStartTime = timeSinceStart;
            }
        } else {
            double negativeTime = timeSinceStart - negativeStartTime;
            if ((leftVelocity > 0.01 && leftErr > -0.5) || negativeTime < 1) {
                leftSetpoint -= 0.1;
            } else {
                leftSetpoint = leftPos;
                negativeLeftLimit = leftPos;
            }
            if ((rightVelocity > 0.01 && rightErr > -0.5) || negativeTime < 1) {
                rightSetpoint -= 0.1;
            } else {
                rightSetpoint = rightPos;
                negativeRightLimit = rightPos;
            }
            if (leftErr < 0.01 && rightErr < 0.01) {
                leftSetpoint = (positiveLeftLimit + negativeLeftLimit) / 2;
                rightSetpoint = (positiveRightLimit + negativeRightLimit) / 2;
                reset = false;
                return true;
            }
        }
        return false;
    }

    public void extend() {
        if (enabled) {
            startPID();
            leftSetpoint = 0;
            rightSetpoint = 0;
        }
    }

    public void retract() {
        if (enabled) {
            startPID();
            rightSetpoint = Constants.ClimberEncoderStepsDist * Constants.ClimberSetpointMult;
            leftSetpoint = Constants.ClimberEncoderStepsDist * Constants.ClimberSetpointMult;
        }
    }

    public void resetPos() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        leftSetpoint = 0;
        rightSetpoint = 0;
        targetLeftSetpoint = 0;
        targetRightSetpoint = 0;
    }

    public void relax() {
        if (enabled) {
            leftClimber.set(0);
            rightClimber.set(0);
            stopPID();
        }
    }

    public void freezePID() {
        leftClimber.set(0);
        rightClimber.set(0);
        leftSetpoint = leftEncoder.getPosition();
        rightSetpoint = rightEncoder.getPosition();
        targetLeftSetpoint = leftSetpoint;
        targetRightSetpoint = rightSetpoint;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Encoder Position", leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Encoder Position", rightEncoder.getPosition());

        enabled = (OI.driveSystem != OI.DriveSystem.TWO_SWITCH) && (OI.getInstance().getLayout(GlobalVariables.driveSystem).climberSubsystemToggle.get());
        double setpointChange = 0.2;
        if (reset) {
            setpointChange = 0.5;
        }
        if (pid) {
            targetLeftSetpoint = MathUtil.clamp(leftSetpoint * 3, targetLeftSetpoint - setpointChange, targetLeftSetpoint + setpointChange);
            targetRightSetpoint = MathUtil.clamp(rightSetpoint * 3, targetRightSetpoint - setpointChange, targetRightSetpoint + setpointChange);
            // This method will be called once per scheduler run
            SmartDashboard.putNumber("Left Climber Setpoint", targetLeftSetpoint);
            SmartDashboard.putNumber("Right Climber Setpoint", targetRightSetpoint);
            if (Constants.DEBUG_MODE) {
                double kp = SmartDashboard.getNumber("Climber P Value", this.kp);
                double ki = SmartDashboard.getNumber("Climber I Value", this.ki);
                leftController.setP(kp);
                rightController.setP(kp);
                leftController.setI(ki);
                rightController.setI(ki);
            }
            leftController.setReference(targetLeftSetpoint, CANSparkMax.ControlType.kPosition);
            rightController.setReference(targetRightSetpoint, CANSparkMax.ControlType.kPosition);
        }
    }

    public static ClimberSubsystem getInstance() {
        if (instance == null) {
            instance = new ClimberSubsystem();
        }

        return instance;
    }

    public enum Side {
        LEFT,
        RIGHT,
        BOTH
    }

    public enum PIDMode {
        EXTEND,
        RETRACT,
        START_PID,
        STOP_PID,
        RESET_POS,
        FREEZE_PID
    }
}
