// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autonomous.RamseteControl;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.List;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private OI.Layout oi = OI.getInstance().getLayout(GlobalVariables.driveSystem);

    //region Drive
    private final DriveSubsystem driveSubsystem = DriveSubsystem.getInstance();
    private final DriveModeChangeCommand driveModeIncreaseCommand = new DriveModeChangeCommand(true);
    private final DriveModeChangeCommand driveModeDecreaseCommand = new DriveModeChangeCommand(false);
    //endregion

    //region Intake
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private final IntakeCommand intakeInCommand = new IntakeCommand(IntakeSubsystem.Mode.INTAKE);
    private final IntakeCommand intakeOutCommand = new IntakeCommand(IntakeSubsystem.Mode.EXPEL);
    private final IntakeCommand intakeStopCommand = new IntakeCommand(IntakeSubsystem.Mode.OFF);
    private final IntakeCommand intakeToggleCommand = new IntakeCommand(IntakeSubsystem.Mode.TOGGLE);
    //endregion

    //region Pneumatics
    private final PneumaticSubsystem pneumaticSubsystem = PneumaticSubsystem.getInstance();
    private final PneumaticCommand pneumaticCommand = new PneumaticCommand(PneumaticSubsystem.Mode.TOGGLE);
    //endregion

    //region Storage
    private final StorageSubsystem storageSubsystem = StorageSubsystem.getInstance();
    private final StorageCommand storageInCommand = new StorageCommand(StorageSubsystem.Direction.IN);
    private final StorageCommand storageOutCommand = new StorageCommand(StorageSubsystem.Direction.OUT);
    private final StorageCommand storageStopCommand = new StorageCommand(StorageSubsystem.Direction.STOP);
    private final SequentialCommandGroup storageReleaseCommand = new SequentialCommandGroup(new StorageCommand(StorageSubsystem.Direction.OUT), new WaitCommand(0.2), new StorageCommand(StorageSubsystem.Direction.STOP));
    //endregion

    //region ColorSensor
    private final ColorSensorSubsystem colorSensorSubsystem = new ColorSensorSubsystem();
    //endregion

    //region Turret
    private final TurretSubsystem turretSubsystem = TurretSubsystem.getInstance();
    private final TurretDistanceCommand turretModeIncreaseCommand = new TurretDistanceCommand(true);
    private final TurretDistanceCommand turretModeDecreaseCommand = new TurretDistanceCommand(false);
    private final TurretAutoToggleCommand turretAutoToggleCommand = new TurretAutoToggleCommand();
    private final TurretInitCommand turretInitCommand = new TurretInitCommand();
    //endregion

    //region Shooter
    private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
    private final ShooterCommand shootStartCommand = new ShooterCommand(true);
    private final ShooterCommand shootStopCommand = new ShooterCommand(false);
    //endregion

    //region Climber
    private final ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();
    private final ClimberManualCommand climberLeftDown = new ClimberManualCommand(ClimberSubsystem.Side.LEFT, 0.4);
    private final ClimberManualCommand climberRightDown = new ClimberManualCommand(ClimberSubsystem.Side.RIGHT, 0.4);
    private final ClimberManualCommand climberLeftUp = new ClimberManualCommand(ClimberSubsystem.Side.LEFT, -0.4);
    private final ClimberManualCommand climberRightUp = new ClimberManualCommand(ClimberSubsystem.Side.RIGHT, -0.4);
    private final ClimberManualCommand climberRightStop = new ClimberManualCommand(ClimberSubsystem.Side.RIGHT, 0);
    private final ClimberManualCommand climberLeftStop = new ClimberManualCommand(ClimberSubsystem.Side.LEFT, 0);
    private final ClimberManualCommand climberBothDown = new ClimberManualCommand(ClimberSubsystem.Side.BOTH, 0.7);
    private final ClimberManualCommand climberBothUp = new ClimberManualCommand(ClimberSubsystem.Side.BOTH, -1);
    private final ClimberCommand climberFreezePIDCommand = new ClimberCommand(ClimberSubsystem.PIDMode.FREEZE_PID);
    private final SequentialCommandGroup climberManualStopCommand = new SequentialCommandGroup(
            new ClimberCommand(ClimberSubsystem.PIDMode.RESET_POS),
            new ClimberCommand(ClimberSubsystem.PIDMode.START_PID));
    //endregion

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Add button to command mappings here.
        // See https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html
        // Example code for running a command with a button
        // oi.exampleButton.whenPressed(new ExampleCommand(exampleSubsystem));
        if (!Constants.OUTREACH_MODE) {
            oi.driveModeDownBtn.whenActive(driveModeDecreaseCommand);
            oi.driveModeUpBtn.whenActive(driveModeIncreaseCommand);
        }
        oi.driveToggleArms
                .whenActive(pneumaticCommand)
                .whenActive(intakeToggleCommand);
        oi.driveToggleIntake
                .whenActive(intakeInCommand)
                .whenInactive(intakeStopCommand);
        oi.driveFastModeBtn
                .whenActive(driveModeIncreaseCommand)
                .whenInactive(driveModeDecreaseCommand);

        oi.storageInBtn.and(oi.climberSubsystemToggle.negate())
                .whenActive(storageInCommand)
                .whenInactive(storageReleaseCommand);
        oi.storageOutBtn.and(oi.climberSubsystemToggle.negate())
                .whenActive(storageOutCommand)
                .whenInactive(storageStopCommand);
        oi.storageOutBtn.and(oi.climberSubsystemToggle)
                .whenActive(climberLeftUp)
                .whenInactive(climberManualStopCommand);

        oi.shooterBtn.and(oi.climberSubsystemToggle.negate())
                .whenActive(shootStartCommand)
                .whenInactive(shootStopCommand);
        oi.shooterBtn.and(oi.climberSubsystemToggle)
                .whenActive(climberLeftDown)
                .whenInactive(climberManualStopCommand);

        oi.turretAutoToggleBtn.whenActive(turretAutoToggleCommand);
        oi.turretModeDownBtn.whenActive(turretModeDecreaseCommand);
        oi.turretModeUpBtn.whenActive(turretModeIncreaseCommand);

        oi.intakeInBtn.and(oi.climberSubsystemToggle.negate())
                .whenActive(intakeInCommand)
                .whenInactive(intakeStopCommand);
        oi.intakeInBtn.and(oi.climberSubsystemToggle)
                .whenActive(climberRightDown)
                .whenInactive(climberManualStopCommand);
        oi.intakeOutBtn.and(oi.climberSubsystemToggle.negate())
                .whenActive(intakeOutCommand)
                .whenInactive(intakeStopCommand);
        oi.intakeOutBtn.and(oi.climberSubsystemToggle)
                .whenActive(climberBothDown)
                .whenInactive(climberManualStopCommand);

        oi.gyroResetBtn
                .whenActive(TurretSubsystem.EncoderWrapper::reset)
                .whenActive(Robot::resetGyro)
                .whenActive(() -> TurretSubsystem.targetYaw = 0.0);

        oi.rightClimberUp.whenActive(climberRightUp);
        oi.rightClimberDown.whenActive(climberRightDown);
        oi.rightClimberStop.whenActive(climberRightStop);
        oi.leftClimberUp.whenActive(climberLeftUp);
        oi.leftClimberDown.whenActive(climberLeftDown);
        oi.leftClimberStop.whenActive(climberLeftStop);
        oi.climberSubsystemToggle.whenActive(() -> GlobalVariables.climberEnabled = ! GlobalVariables.climberEnabled);
        oi.rightClimberDown
                .whenActive(climberRightDown)
                .whenInactive(climberManualStopCommand);
        oi.rightClimberUp
                .whenActive(climberRightUp)
                .whenInactive(climberManualStopCommand);
        oi.leftClimberDown
                .whenActive(climberLeftDown)
                .whenInactive(climberManualStopCommand);
        oi.leftClimberUp
                .whenActive(climberLeftUp)
                .whenInactive(climberManualStopCommand);
        oi.rightClimberUp2.and(oi.climberSubsystemToggle)
                .whenActive(climberRightUp)
                .whenInactive(climberManualStopCommand);
        oi.climberHighSpeedUp.and(oi.climberSubsystemToggle)
                .whenActive(climberBothUp)
                .whenInactive(climberManualStopCommand);

        oi.distanceOffsetUp.whenActive(() -> {
            GlobalVariables.DistanceOffset += 0.5;
            SmartDashboard.putNumber("Distance Offset", GlobalVariables.DistanceOffset);
        });
        oi.distanceOffsetDown.whenActive(() -> {
            GlobalVariables.DistanceOffset -= 0.5;
            SmartDashboard.putNumber("Distance Offset", GlobalVariables.DistanceOffset);
        });
        oi.turretAutoToggleBtn.whenActive(turretAutoToggleCommand);
    }

    public void changeControlLayout(OI.DriveSystem driveSystem) {
        oi = OI.getInstance().getLayout(driveSystem);
        CommandScheduler.getInstance().clearButtons();
        configureButtonBindings();
    }

    public void robotInitCommands() {
        CommandScheduler.getInstance().schedule(turretInitCommand, climberFreezePIDCommand);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // Resets the odometry, encoders, and gyro.
//            Command ramseteBackup = RamseteControl.generateRamseteBackupCommand(new Pose2d(4, 1, new Rotation2d(0)),
//                    List.of(new Translation2d(-4, -1)),
//                    new Pose2d(0, 0, new Rotation2d(0)));
        //  RamseteGenerators do not automatically stop at their end point, to make chaining easier.
        //  Adding a separate command to do so after completion avoids rolling into autonomous.
        //SequentialCommandGroup autonomousCommandGroup = ramseteTest.andThen(ramseteBackup).andThen(() -> driveSubsystem.tankDriveSetVolts(0., 0.));
        if (false) {
            return new SequentialCommandGroup(
                    new TurretInitCommand(),
                    new PneumaticCommand(PneumaticSubsystem.Mode.EXTEND),
                    new WaitCommand(0.5),
                    new IntakeCommand(IntakeSubsystem.Mode.INTAKE),
                    new StorageCommand(StorageSubsystem.Direction.IN),
                    new WaitCommand(0.5),
                    selectAutoDrive(0),
                    new DriveStopCommand(),
                    new WaitCommand(.75),
                    new SequentialCommandGroup(new StorageCommand(StorageSubsystem.Direction.OUT), new WaitCommand(0.2), new StorageCommand(StorageSubsystem.Direction.STOP)),
                    new TurretAutoToggleCommand(),
                    new ShooterCommand(true),
                    new WaitCommand(.75),
                    new StorageCommand(StorageSubsystem.Direction.IN),
                    new WaitCommand(2),
                    new ShooterCommand(false),
                    selectAutoDrive(1),
                    new DriveStopCommand(),
                    new WaitCommand(.75),
                    new SequentialCommandGroup(new StorageCommand(StorageSubsystem.Direction.OUT), new WaitCommand(0.2), new StorageCommand(StorageSubsystem.Direction.STOP)),
                    new ShooterCommand(true),
                    new WaitCommand(.75),
                    new StorageCommand(StorageSubsystem.Direction.IN),
                    new WaitCommand(2),
                    new ShooterCommand(false),
                    new IntakeCommand(IntakeSubsystem.Mode.OFF),
                    new StorageCommand(StorageSubsystem.Direction.STOP),
                    new TurretAutoToggleCommand()
            );
        } else {
            return new SequentialCommandGroup(
                    new TurretInitCommand(),
                    new PneumaticCommand(PneumaticSubsystem.Mode.EXTEND),
                    new WaitCommand(0.5),
                    new IntakeCommand(IntakeSubsystem.Mode.INTAKE),
                    new StorageCommand(StorageSubsystem.Direction.IN),
                    new WaitCommand(0.5),
                    selectAutoDrive(0),
                    new DriveStopCommand(),
                    new WaitCommand(1),
                    new IntakeCommand(IntakeSubsystem.Mode.OFF),
                    new PneumaticCommand(PneumaticSubsystem.Mode.RETRACT),
                    new SequentialCommandGroup(new StorageCommand(StorageSubsystem.Direction.OUT), new WaitCommand(0.2), new StorageCommand(StorageSubsystem.Direction.STOP)),
                    new TurretAutoToggleCommand(),
                    new TurretDistanceCommand(true),
                    new ShooterCommand(true),
                    new WaitCommand(1),
                    new StorageCommand(StorageSubsystem.Direction.IN),
                    new WaitCommand(0.5),
                    new SequentialCommandGroup(new StorageCommand(StorageSubsystem.Direction.OUT), new WaitCommand(0.2), new StorageCommand(StorageSubsystem.Direction.STOP)),
                    new StorageCommand(StorageSubsystem.Direction.IN),
                    new WaitCommand(5),
                    new StorageCommand(StorageSubsystem.Direction.STOP),
                    new ShooterCommand(false),
                    new TurretAutoToggleCommand()
            );
        }
    }

    private Command selectAutoDrive(int num) {
        if (Constants.RAMSETE_AUTO) {
            DriveSubsystem.getInstance().resetOdometry();
            switch (num) {
                case 0:
                    return RamseteControl.generateRamseteStopCommand(new Pose2d(0, 0, new Rotation2d(0)),
                            List.of(new Translation2d(1.7, 0)),
                            new Pose2d(1.7, 0, new Rotation2d(0)));
                case 1:
                    return RamseteControl.generateRamseteStopCommand(new Pose2d(1.7, 0, new Rotation2d(0)),
                            List.of(new Translation2d(3.6, 0)),
                            new Pose2d(5.3, 0, new Rotation2d(0)));
                case 2:
                    return RamseteControl.generateRamseteStopCommand(new Pose2d(0, 0, new Rotation2d(0)),
                            List.of(new Translation2d(1, 0)),
                            new Pose2d(1, 0, new Rotation2d(0)));
                default:
                    return null;
            }

        } else {
            return new DriveDistanceCommand(1.5, 0.2);
        }
    }
}
