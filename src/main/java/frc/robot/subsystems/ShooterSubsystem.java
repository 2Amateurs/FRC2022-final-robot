package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

public class ShooterSubsystem extends SubsystemBase {
    public static ShooterSubsystem instance = null;

    private final CANSparkMax frontShooter = new CANSparkMax(Constants.frontShooterCanID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder frontEncoder = frontShooter.getEncoder();
    private final SparkMaxPIDController frontController = frontShooter.getPIDController();

    private final CANSparkMax backShooter = new CANSparkMax(Constants.backShooterCanID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder backEncoder = backShooter.getEncoder();
    private final SparkMaxPIDController backController = backShooter.getPIDController();

    private boolean shoot = false;

    private ShooterSubsystem() {
        frontShooter.setSmartCurrentLimit(40, 80, 100);
        backShooter.setSmartCurrentLimit(40, 80, 100);
        frontController.setP(0.3 / Constants.maxRPM);
        frontController.setI(0.0);
        frontController.setD(0.0);
        frontController.setFF(1.2 / Constants.maxRPM);
        backController.setP(0.3 / Constants.maxRPM);
        backController.setI(0.0);
        backController.setD(0.0);
        backController.setFF(1.65 / Constants.maxRPM);
        if (Constants.DEBUG_MODE || Constants.CALIBRATION_MODE) {
            SmartDashboard.putNumber("Front DEBUG Speed", 0.0);
            SmartDashboard.putNumber("Back DEBUG Speed", 0.0);
            SmartDashboard.putNumber("Front P value", frontController.getP());
            SmartDashboard.putNumber("Front I value", frontController.getI());
            SmartDashboard.putNumber("Front D value", frontController.getD());
            SmartDashboard.putNumber("Front FF value", frontController.getFF());
            SmartDashboard.putNumber("Back P value", backController.getP());
            SmartDashboard.putNumber("Back I value", backController.getI());
            SmartDashboard.putNumber("Back D value", backController.getD());
            SmartDashboard.putNumber("Back FF value", backController.getFF());
            SmartDashboard.putNumber("Front Speed Feed Forward", frontController.getFF());
            SmartDashboard.putNumber("Back Speed Feed Forward", backController.getFF());
        }
        SmartDashboard.putBoolean("Ready Override", GlobalVariables.readyOverride);
    }

    public void readyToShoot() {
        if (!shoot) {
            GlobalVariables.isReadyToFire = true;
            return;
        }
        if (Constants.CALIBRATION_MODE) {
            GlobalVariables.isReadyToFire = true;
            return;
        }
        if (!GlobalVariables.readyOverride) {
            GlobalVariables.isReadyToFire = true;
        } else {
            GlobalVariables.isReadyToFire = (Math.abs(GlobalVariables.getFrontShooterSpeed() - frontEncoder.getVelocity()) < Constants.launchSpeedAccuracyRequirement)
                    && (Math.abs(GlobalVariables.getBackShooterSpeed() - backEncoder.getVelocity()) < Constants.launchSpeedAccuracyRequirement);
        }
        SmartDashboard.putBoolean("ReadyToFire", GlobalVariables.isReadyToFire);
    }

    public void shoot(boolean shoot) {
        this.shoot = shoot;
    }

    @Override
    public void periodic() {
        readyToShoot();
        double frontSetpoint;
        double backSetpoint;

        GlobalVariables.readyOverride = SmartDashboard.getBoolean("Ready Override", true);

        if (Constants.CALIBRATION_MODE) {
            frontSetpoint = SmartDashboard.getNumber("Front DEBUG Speed", 0.0);
            backSetpoint = SmartDashboard.getNumber("Back DEBUG Speed", 0.0);

            SmartDashboard.putNumber("Front Encoder", frontEncoder.getVelocity() / Constants.maxRPM);
            SmartDashboard.putNumber("Back Encoder", backEncoder.getVelocity() / Constants.maxRPM);

            double newFrontP = SmartDashboard.getNumber("Front P value", frontController.getP());
            double newFrontI = SmartDashboard.getNumber("Front I value", frontController.getI());
            double newFrontD = SmartDashboard.getNumber("Front D value", frontController.getD());
            double newFrontFF = SmartDashboard.getNumber("Front FF value", frontController.getFF());
            double newBackP = SmartDashboard.getNumber("Back P value", backController.getP());
            double newBackI = SmartDashboard.getNumber("Back I value", backController.getI());
            double newBackD = SmartDashboard.getNumber("Back D value", backController.getD());
            double newBackFF = SmartDashboard.getNumber("Back FF value", backController.getFF());
            frontController.setP(newFrontP);
            frontController.setI(newFrontI);
            frontController.setD(newFrontD);
            frontController.setFF(newFrontFF);
            backController.setP(newBackP);
            backController.setI(newBackI);
            backController.setD(newBackD);
            backController.setFF(newBackFF);
        } else {
            frontSetpoint = GlobalVariables.getFrontShooterSpeed();
            backSetpoint = GlobalVariables.getBackShooterSpeed();
        }
        if (!shoot) {
            frontSetpoint = -300;
            backSetpoint = 300;
        }

        SmartDashboard.putNumber("Shooter Front Setpoint", frontSetpoint);
        SmartDashboard.putNumber("Shooter Back Setpoint", backSetpoint);

        frontController.setReference(frontSetpoint, CANSparkMax.ControlType.kVelocity);
        backController.setReference(backSetpoint, CANSparkMax.ControlType.kVelocity);
    }

    public static ShooterSubsystem getInstance() {
        if (instance == null) {
            instance = new ShooterSubsystem();
        }

        return instance;
    }
}

