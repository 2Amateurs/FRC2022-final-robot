package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.Robot;
import frc.robot.commands.DriveDefaultCommand;

import java.util.ArrayList;
import java.util.List;

public class DriveSubsystem extends SubsystemBase {
    private static DriveSubsystem instance = null;

    private final CANSparkMax leftFront = new CANSparkMax(Constants.leftFrontDriveMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightFront = new CANSparkMax(Constants.rightFrontDriveMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax leftBack = new CANSparkMax(Constants.leftBackDriveMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightBack = new CANSparkMax(Constants.rightBackDriveMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder leftFrontEncoder = leftFront.getEncoder();
    private final RelativeEncoder rightFrontEncoder = rightFront.getEncoder();
    private final RelativeEncoder leftBackEncoder = leftBack.getEncoder();
    private final RelativeEncoder rightBackEncoder = rightBack.getEncoder();

    //the groups make controlling the drive easier
    private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftFront, leftBack);
    private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightFront, rightBack);
    private final DifferentialDrive driveBase = new DifferentialDrive(leftMotors, rightMotors);
    private final DifferentialDriveOdometry driveOdometry = new DifferentialDriveOdometry(Robot.getRawGyro().getRotation2d());

    //variables for the previous speeds during the execution loop
    private double prevLeftSpeed = 0;
    private double prevRightSpeed = 0;
    private double prevTurnSpeed = 0;
    private DriveMode currentDriveMode = Constants.DefaultDriveMode;
    private DriveMode lastDriveSpeed = Constants.DefaultDriveMode;

    /**
     * Creates a new ExampleSubsystem.
     */
    private DriveSubsystem() {
        leftFront.setSmartCurrentLimit(40, 80, 100);
        leftBack.setSmartCurrentLimit(40, 80, 100);
        rightFront.setSmartCurrentLimit(40, 80, 100);
        rightBack.setSmartCurrentLimit(40, 80, 100);
//        final double meterPerRev =  Units.inchesToMeters(2 * Math.PI * 3) / 7.31;
        final double meterPerRev = 0.0548386;  // experimentally derived.
        setEncoderConversions(rightBackEncoder, meterPerRev);
        setEncoderConversions(leftBackEncoder, meterPerRev);
        setEncoderConversions(rightFrontEncoder, meterPerRev);
        setEncoderConversions(leftFrontEncoder, meterPerRev);
        leftMotors.setInverted(true);
        SmartDashboard.putString("Drive Mode", currentDriveMode.getName());
        resetEncoders();
        resetGyro();
    }

    private void setEncoderConversions(RelativeEncoder encoder, double meterPerRev)
    {
        encoder.setPositionConversionFactor(meterPerRev);
        encoder.setVelocityConversionFactor(meterPerRev * 0.020);
    }

    public void drive(double leftDriveValue, double rightDriveValue, double leftTurnValue) {
        double leftSpeed = leftDriveValue;
        double rightSpeed = rightDriveValue;
        double turnSpeed = leftTurnValue;
        if (currentDriveMode.isInvertLin()) {
            leftSpeed *= -1;
            rightSpeed *= -1;
            if (GlobalVariables.TankDrive) {
                double temp = leftSpeed;
                leftSpeed = rightSpeed;
                rightSpeed = temp;
            }
        }
        if (currentDriveMode.isInvertRot()) {
            turnSpeed *= -1;
        }

        if (currentDriveMode.isRawMode()) {
            prevLeftSpeed = leftSpeed;
            prevRightSpeed = rightSpeed;
        } else if (GlobalVariables.TankDrive) {
            double newLeftSpeed = leftSpeed * currentDriveMode.getLinearMult();
            double newRightSpeed = rightSpeed * currentDriveMode.getLinearMult();
            double maxAccel = currentDriveMode.getMaxLinAccel();

            prevLeftSpeed = MathUtil.clamp(newLeftSpeed, prevLeftSpeed - maxAccel, prevLeftSpeed + maxAccel);
            prevRightSpeed = MathUtil.clamp(newRightSpeed, prevRightSpeed - maxAccel, prevRightSpeed + maxAccel);
            driveBase.tankDrive(prevLeftSpeed, prevRightSpeed, false);
        } else {
            double newLeftSpeed = leftSpeed * currentDriveMode.getLinearMult();
            double newTurnSpeed = turnSpeed * currentDriveMode.getRotMult();
            double maxLinAccel = currentDriveMode.getMaxLinAccel();
            double maxRotAccel = currentDriveMode.getMaxRotAccel();

            prevLeftSpeed = MathUtil.clamp(newLeftSpeed, prevLeftSpeed - maxLinAccel, prevLeftSpeed + maxLinAccel);
            prevTurnSpeed = MathUtil.clamp(newTurnSpeed, prevTurnSpeed - maxRotAccel, prevTurnSpeed + maxRotAccel);
            driveBase.curvatureDrive(prevLeftSpeed, prevTurnSpeed, true);
        }
        driveBase.feed();
    }

    public void tankDriveSetVolts(double leftVolt, double rightVolt) {
        // TODO : I /absolutely/ hate this fix.  But without it, the robot run-away accelerates opposite its target location.
        rightMotors.setVoltage(-rightVolt);
        leftMotors.setVoltage(-leftVolt);
        driveBase.feed();
    }

    public double getHeading() {
        return Robot.getRawGyro().getRotation2d().getDegrees();
    }

    public void changeMode(final boolean increase) {
        if (currentDriveMode.index == -1) {
            currentDriveMode = DriveMode.driveModes.get(0);
        }
        if (increase) {
            currentDriveMode = DriveMode.driveModes.get((currentDriveMode.index + 1) % DriveMode.driveModes.size());
        } else {
            if(currentDriveMode.index - 1 == -1) {
                currentDriveMode = DriveMode.driveModes.get(DriveMode.driveModes.size() - 1);
            } else {
                currentDriveMode = DriveMode.driveModes.get(currentDriveMode.index - 1);
            }
        }
        SmartDashboard.putString("Drive Mode", currentDriveMode.getName());
//        SmartDashboard.putNumber("Drive Mode", (100.0 / DriveMode.driveModes.size()) * currentDriveMode.index);
    }

    public void setCurrentDriveMode(DriveMode newDriveMode) {
        lastDriveSpeed = this.currentDriveMode;
        this.currentDriveMode = newDriveMode;
        SmartDashboard.putString("Drive Mode", this.currentDriveMode.getName());
    }

    public void setCurrentDriveMode() {
        this.currentDriveMode = lastDriveSpeed;
        SmartDashboard.putString("Drive Mode", this.currentDriveMode.getName());
    }

    public double getEncoderAveragePos() {
        //double gearRatio = 1 / 7.31;
        //double wheelCircumference = 2 * Math.PI * 3;
        // CAN SPARK MAXes don't support inverting an encoder's values, so this jank is necessary.
        double leftFrontPos = leftFrontEncoder.getPosition();//Units.inchesToMeters(leftFrontEncoder.getPosition() * gearRatio * wheelCircumference) * -1;
        double rightFrontPos = -1. * rightFrontEncoder.getPosition();//Units.inchesToMeters(rightFrontEncoder.getPosition() * gearRatio * wheelCircumference) * -1;
        double leftBackPos = leftBackEncoder.getPosition();//Units.inchesToMeters(leftBackEncoder.getPosition() * gearRatio * wheelCircumference) * -1;
        double rightBackPos = -1. * rightBackEncoder.getPosition();//Units.inchesToMeters(rightBackEncoder.getPosition() * gearRatio * wheelCircumference) * -1;
        SmartDashboard.putNumber("Left Front Drive Encoder", leftFrontPos);
        SmartDashboard.putNumber("Right Front Drive Encoder", rightFrontPos);
        SmartDashboard.putNumber("Left Back Drive Encoder", leftBackPos);
        SmartDashboard.putNumber("Right Back Drive Encoder", rightBackPos);

        double averagePos = (leftFrontPos + rightFrontPos + leftBackPos + rightBackPos) / 4;
        SmartDashboard.putNumber("Average Drive Encoder", averagePos);
        return averagePos;
    }

    public double getLeftEncoder() {
        return (leftFrontEncoder.getPosition() + leftBackEncoder.getPosition()) / 2;
    }

    public double getRightEncoder() {
        return -1. * (rightFrontEncoder.getPosition() + rightBackEncoder.getPosition()) / 2;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds()
    {
        // Again, SPARK MAXes don't support inverting encoder values.
        return new DifferentialDriveWheelSpeeds(leftBackEncoder.getVelocity(), -1. * rightBackEncoder.getVelocity());
    }

    public void resetEncoders() {
        leftFrontEncoder.setPosition(0);
        rightFrontEncoder.setPosition(0);
        leftBackEncoder.setPosition(0);
        rightBackEncoder.setPosition(0);
        getEncoderAveragePos();
    }

    public void resetGyro() {
        Robot.getRawGyro().reset();
    }

    public void resetOdometry(Pose2d startingPose, Rotation2d rotation)
    {
        resetGyro();
        resetEncoders();
        this.driveOdometry.resetPosition(startingPose, rotation);
    }

    public void resetOdometry()
    {
        resetOdometry(new Pose2d(0,0, new Rotation2d(0)), new Rotation2d(0));
    }

    public Pose2d getPose() {
        return driveOdometry.getPoseMeters();
    }

    @Override
    public void periodic() {
        if(Constants.RAMSETE_AUTO) {
            Pose2d position = driveOdometry.getPoseMeters();
            double lastX = position.getX();
            double lastY = position.getY();
            driveOdometry.update(Robot.getRawGyro().getRotation2d(), getLeftEncoder(), getRightEncoder());
            position = driveOdometry.getPoseMeters();
            SmartDashboard.putNumber("PoseX", position.getX());
            SmartDashboard.putNumber("PoseY", position.getY());
            SmartDashboard.putNumber("PoseRot", position.getRotation().getDegrees());
            SmartDashboard.putNumber("PositionX", position.getTranslation().getX());
            SmartDashboard.putNumber("PositionY", position.getTranslation().getY());
        }

        getEncoderAveragePos();
    }

    public static DriveSubsystem getInstance() {
        if (instance == null) {
            instance = new DriveSubsystem();
            instance.setDefaultCommand(new DriveDefaultCommand());
        }

        return instance;
    }

    public static class DriveMode {
        private final String name;
        private final boolean rawMode;
        private final double maxLinAccel;
        private final double maxRotAccel;
        private final double linearMult;
        private final double rotMult;
        private final boolean invertLin;
        public int index = -1;
        private final boolean invertRot;

        public static final List<DriveMode> driveModes = new ArrayList<>();

        public DriveMode(DriveMode.Builder builder) {
            this.name = builder.name;
            this.rawMode = builder.rawMode;
            this.maxLinAccel = builder.maxLinAccel;
            this.maxRotAccel = builder.maxRotAccel;
            this.linearMult = builder.linearMult;
            this.rotMult = builder.rotMult;
            this.invertLin = builder.invertLin;
            this.invertRot = builder.invertRot;
            if (builder.addToList) {
                index = driveModes.size();
                driveModes.add(this);
            }
        }

        public String getName() {
            return name;
        }

        public boolean isRawMode() {
            return rawMode;
        }

        public double getMaxLinAccel() {
            return maxLinAccel;
        }

        public double getMaxRotAccel() {
            return maxRotAccel;
        }

        public double getLinearMult() {
            return linearMult;
        }

        public double getRotMult() {
            return rotMult;
        }

        public boolean isInvertLin() {
            return invertLin;
        }

        public boolean isInvertRot() {
            return invertRot;
        }

        public static class Builder {
            private final String name;
            private boolean rawMode = false;
            private double maxLinAccel = 1;
            private double maxRotAccel = 1;
            private double linearMult = 1;
            private double rotMult = 1;
            private boolean invertLin = false;
            private boolean invertRot = false;
            private boolean addToList = true;

            public Builder(String name) {
                this.name = name;
            }

            public DriveMode.Builder rawMode(boolean rawMode) {
                this.rawMode = rawMode;
                return this;
            }

            public DriveMode.Builder maxLinAccel(double maxLinAccel) {
                this.maxLinAccel = maxLinAccel;
                return this;
            }

            public DriveMode.Builder maxRotAccel(double maxRotAccel) {
                this.maxRotAccel = maxRotAccel;
                return this;
            }

            public DriveMode.Builder linearMult(double linearMult) {
                this.linearMult = linearMult;
                return this;
            }

            public DriveMode.Builder rotMult(double rotMult) {
                this.rotMult = rotMult;
                return this;
            }

            public DriveMode.Builder invertLin() {
                this.invertLin = true;
                return this;
            }

            public DriveMode.Builder invertRot() {
                this.invertRot = true;
                return this;
            }

            public DriveMode.Builder addToList(boolean addToList) {
                this.addToList = addToList;
                return this;
            }

            public DriveMode build() {
                return new DriveMode(this);
            }
        }
    }

}

