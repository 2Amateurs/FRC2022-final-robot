// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveSubsystem.DriveMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean DEBUG_MODE = false;
    public static final boolean CALIBRATION_MODE = false;
    public static final boolean RAMSETE_AUTO = true;
    public static final boolean OUTREACH_MODE = true;
    public static final boolean SAFE_MODE = true;

    //region DRIVE VALUES
    public static final int leftFrontDriveMotorPort = 23; //old: 11
    public static final int leftBackDriveMotorPort = 24; //old: 12
    public static final int rightFrontDriveMotorPort = 21; //old: 13
    public static final int rightBackDriveMotorPort = 22; //old: 14
    public static final double DeadzoneDefault = 0.03;
    //endregion

    //region DRIVE MODES
    //Slow Mode: reduced speed
    public static final DriveMode SlowMode = new DriveMode.Builder("SlowMode")
            .maxLinAccel(0.025)
            .maxRotAccel(0.025)
            .linearMult(0.2)
            .rotMult(0.2)
            .build();
    //Standard Mode copy: public static DriveMode StdMode = new DriveMode("Standard", false, 0.025, 0.025, 0.8, 0.28, false, true);
    public static final DriveMode StdMode = new DriveMode.Builder("StandardMode")
            .maxLinAccel(0.025)
            .maxRotAccel(0.025)
            .linearMult(0.8)
            .rotMult(0.28)
            .addToList(false)
            .build();
    //Fast Mode: faster driving
    public static final DriveMode FastMode = new DriveMode.Builder("FastMode")
            .maxLinAccel(0.06)
            .maxRotAccel(0.04)
            .rotMult(0.35)
            .build();
    //Reversed: same as Standard, but the back of the robot is now the front
    public static final DriveMode Reversed = new DriveMode.Builder("Reversed")
            .maxLinAccel(StdMode.getMaxLinAccel())
            .maxRotAccel(StdMode.getMaxRotAccel())
            .linearMult(StdMode.getLinearMult())
            .rotMult(StdMode.getRotMult())
            .invertLin()
            .addToList(false)
            .build();
    public static DriveMode ExtraMode1 = new DriveMode.Builder("ExtraMode1")
            .maxLinAccel(0.04)
            .maxRotAccel(0.03)
            .linearMult(0.6)
            .rotMult(0.3)
            .addToList(false)
            .build();
    //Raw input means no acceleration processing is done, this would be for autonomous routines that handle their own acceleration processing, etc.
    public static final DriveMode RawInput = new DriveMode.Builder("RawMode").addToList(false).build();
    public static final DriveMode DefaultDriveMode = SlowMode;
    //endregion

    //region INTAKE/STORAGE VALUES
    public static final int pneumaticControlCanID = 0;
    public static final double intakeSpeed = 0.65;
    public static final double expelSpeed = -0.3;
    public static final int feederMotorPort = 17;
    public static final double feederInSpeed = 0.5;
    public static final double feederOutSpeed = -0.4;
    public static final int intakeMotorPort = 19;
    //endregion

    //region TURRET VALUES
    public static final int turretVertMotorPort = 15; //brushed motor
    public static final double MinTurretSpeed = 0.1;
    public static final double turretMaxHorSpeed = 0.5;
    public static final double turretMaxHorDiff = 15;
    public static final double turretMaxVertSpeed = 0.5;
    public static final int turretMinVertEncoderValue = 8000;
    public static final double turretVertSetpointChange = 200;
    public static final int minYawValue = -110;
    public static final int maxYawValue = 200;
    public static final double WrongColorOffset = 45;
    public static final double MinStorageFeedDelay = 2.5; // seconds since last cargo detect before acting as no cargo is on robot.
    public static final double CameraAngle = 45;
    public static final int turretHorMotorPort = 18;
    //endregion

    //region SHOOTER VALUES
    public static final int frontShooterCanID = 14;
    public static final int backShooterCanID = 20;
    public static final double maxRPM = 6500;
    public static final double launchSpeedAccuracyRequirement = 1000;
    //endregion

    //region CLIMBER VALUES
    public static final int leftClimberMotorPort = 16;
    public static final int rightClimberMotorPort = 25;
    public static final double ClimberSpeed = 1;
    public static final double ClimberEncoderError = 1;
    public static final double ClimberEncoderStepsDist = 55;
    public static final double ClimberSetpointMult = 1.10;
    //endregion
}