package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.Robot;
import frc.robot.commands.TurretDefaultCommand;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import static frc.robot.GlobalVariables.*;
import static org.photonvision.PhotonUtils.calculateDistanceToTargetMeters;

public class TurretSubsystem extends SubsystemBase {
    public static TurretSubsystem instance = null;

    private final VictorSPX horMotor = new VictorSPX(Constants.turretHorMotorPort);
    private final PIDController horController = new PIDController(0.013, 0.0, 0.0);
    private final PIDController wheelAimController = new PIDController(0.013, 0.0, 0.0);
    private double horEncoderPos = 0;

    private final TalonSRX vertMotor = new TalonSRX(Constants.turretVertMotorPort);
    private final DigitalInput vertLimitSwitch = new DigitalInput(2);
    private final PIDController vertController = new PIDController(0.001, 0.0, 0.0);
    private double vertSetpoint = 0;

    private final PhotonCamera camera = new PhotonCamera("vault6936");

    public static double targetYaw = 0.0;
    public static double gyroYaw = 0.0;
    public static double offsetYaw = 0.0;
    private double tmpYaw = 0.0;
    static final double cameraHeight = Units.inchesToMeters(32);
    static final double targetHeight = Units.inchesToMeters(93); //real value: 104
    private boolean prevAutoTarget = autoTarget;

    private TurretSubsystem() {
        if (Constants.DEBUG_MODE) {
            SmartDashboard.putNumber("Camera Angle", Constants.CameraAngle);
            SmartDashboard.putNumber("Horiz P value", horController.getP());
            SmartDashboard.putNumber("Horiz I value", horController.getI());
            SmartDashboard.putNumber("Horiz D value", horController.getD());
        }
        if (Constants.CALIBRATION_MODE) {
            SmartDashboard.putNumber("Vertical Calibration Setpoint", 0);
        }
        camera.setLED(VisionLEDMode.kOn);
        camera.setDriverMode(true);
        camera.setDriverMode(false);
    }

    public void horAim(double horAim) {
        double horSpeedMult = 1.4;
        targetYaw -= horAim * horSpeedMult;
        if (Constants.DEBUG_MODE) {
            SmartDashboard.putNumber("Target Yaw", targetYaw);
        }
    }

    public void horBack() {
        targetYaw = 180;
    }

    public void vertAim(double vertAim, boolean init) {
        if (init) {
            vertSetpoint = -300000;
        } else {
            if (vertAim > 0.9) {
                vertSetpoint += Constants.turretVertSetpointChange;
            } else if (vertAim < -0.9) {
                vertSetpoint -= Constants.turretVertSetpointChange;
            }
            vertSetpoint = MathUtil.clamp(vertSetpoint, 0, Constants.turretMinVertEncoderValue);
        }
        //if(Constants.DEBUG_MODE) {
        SmartDashboard.putNumber("Vertical Setpoint", vertSetpoint);
        //}
    }

    private void setVertMotor(double speed) {
        if (!vertLimitSwitch.get() || speed > 0) {
            vertMotor.set(TalonSRXControlMode.PercentOutput, speed);
        } else {
            vertMotor.set(TalonSRXControlMode.PercentOutput, 0);
        }
    }

    public void distanceChange(final boolean increase) {
        double distance = Math.floor(TargetDistance);
        if (increase) {
            distance++;
        } else {
            distance--;
        }

        double maxDist = 10;
        if (Constants.OUTREACH_MODE) {
            maxDist = 5;
        }
        TargetDistance = MathUtil.clamp(distance, -1, maxDist);
        String presetName = distance < 0 ? "low" : String.format("%dm", (int) distance);
        SmartDashboard.putString("Shooter Preset", presetName);
    }

    public void toggleGuidance() {
        if (autoTarget) { // Switching autoTarget off
            camera.setLED(VisionLEDMode.kOff);
            camera.setDriverMode(true);
            autoTarget = false;
        } else { // Switching autoTarget on
            camera.setLED(VisionLEDMode.kOn);
            camera.setDriverMode(false);
            autoTarget = true;
        }
        SmartDashboard.putBoolean("Turret Auto On", autoTarget);
    }

    public boolean getVertLimitSwitch() {
        return vertLimitSwitch.get();
    }

    public void resetVertEncoder() {
        vertMotor.setSelectedSensorPosition(0);
        vertSetpoint = 0;
    }

    private PhotonTrackedTarget camera() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (Constants.DEBUG_MODE) {
            SmartDashboard.putNumber("Target Count", result.targets.size());
        }
        PhotonTrackedTarget bestTarget = null;
        if (result.hasTargets()) {
            bestTarget = getBestTarget(result);
            double cameraPitch = SmartDashboard.getNumber("Camera Angle", Constants.CameraAngle);
            double camDist = calculateDistanceToTargetMeters(cameraHeight, targetHeight,
                    Units.degreesToRadians(cameraPitch),
                    Units.degreesToRadians(bestTarget.getPitch()));
            GlobalVariables.setDistance(camDist);
            SmartDashboard.putNumber("Distance to Target", TargetDistance);
            SmartDashboard.putNumber("Camera Distance", camDist);
        }
        return bestTarget;
    }

    private PhotonTrackedTarget getBestTarget(PhotonPipelineResult result) {
        PhotonTrackedTarget bestTarget = null;
        if (result.hasTargets()) {
            for (PhotonTrackedTarget target : result.targets) {
                if (bestTarget == null) {
                    bestTarget = target;
                }
                if (target.getPitch() >= bestTarget.getPitch()) {
                    bestTarget = target;
                }
            }
        }
        return bestTarget;
    }

    private double autoTarget(PhotonTrackedTarget target) {
        double goalYaw;
        if (autoTarget && target != null) {
            goalYaw = (-target.getYaw() * 0.6) + horEncoderPos + offsetYaw;
            SmartDashboard.putNumber("Target Yaw", goalYaw);
            targetYaw = goalYaw;
        } else if (autoTarget) {
            goalYaw = targetYaw;
        } else if (prevAutoTarget) {
            goalYaw = 0;
            targetYaw = 0;
        } else {
            goalYaw = targetYaw;
        }
//        goalYaw = MathUtil.clamp(goalYaw, Constants.minYawValue, Constants.maxYawValue);
        SmartDashboard.putNumber("Result Yaw", goalYaw);
        return goalYaw;
    }

    private void horMotorControl(double goalYaw) {
        if (!autoTarget) {
            goalYaw = loopHorMotor(goalYaw);
        }
        if (Constants.DEBUG_MODE) {
            double newP = SmartDashboard.getNumber("Horiz P value", horController.getP());
            double newI = SmartDashboard.getNumber("Horiz I value", horController.getI());
            double newD = SmartDashboard.getNumber("Horiz D value", horController.getD());
            horController.setP(newP);
            horController.setI(newI);
            horController.setD(newD);
        }
        double clampedHorGoalDiff = MathUtil.clamp(goalYaw - horEncoderPos, -Constants.turretMaxHorDiff, Constants.turretMaxHorDiff);
        double horSpeed = -horController.calculate(horEncoderPos, horEncoderPos + clampedHorGoalDiff);
        horSpeed = MathUtil.clamp(horSpeed, -Constants.turretMaxHorSpeed, Constants.turretMaxHorSpeed);
        if (Constants.DEBUG_MODE) {
            SmartDashboard.putNumber("Hor Target Speed", horSpeed);
        }
        if (Constants.WHEEL_AIM) {
            GlobalVariables.autoWheelTurn = MathUtil.clamp(wheelAimController.calculate(goalYaw, 0), -0.3, 0.3);
            if (Math.abs(goalYaw) < 1) {
                GlobalVariables.wheelAiming = false;
            }
        } else {
            if (Math.abs(horSpeed) < Constants.MinTurretSpeed) {
                horMotor.set(ControlMode.PercentOutput, 0);
            } else {
                horMotor.set(ControlMode.PercentOutput, horSpeed);
            }
        }
    }

    private double loopHorMotor(double goalYaw) {
        tmpYaw = goalYaw - Math.copySign(360, tmpYaw);
        if (tmpYaw == MathUtil.clamp(tmpYaw, Constants.minYawValue, Constants.maxYawValue)) {
            targetYaw = tmpYaw;
            return tmpYaw;
        } else {
            return goalYaw;
        }
    }

    private void vertMotorControl(PhotonTrackedTarget target) {
        SmartDashboard.putBoolean("Vertical Limit Switch", getVertLimitSwitch());
        if(autoTarget) {
            if (vertSetpoint >= 0) {
                if (Constants.CALIBRATION_MODE) {
                    vertSetpoint = SmartDashboard.getNumber("Vertical Calibration Setpoint", 0);
                } else {
                    vertSetpoint = GlobalVariables.getVertSetpoint();
                }
                vertSetpoint -= goodBall ? 0 : 500;
                vertSetpoint = MathUtil.clamp(vertSetpoint, 0, 8000);
            }
        }
        double vertSpeed = vertController.calculate(vertMotor.getSelectedSensorPosition(), vertSetpoint);
        vertSpeed = MathUtil.clamp(vertSpeed, -Constants.turretMaxVertSpeed, Constants.turretMaxVertSpeed);
        SmartDashboard.putNumber("Vertical Setpoint", vertSetpoint);
        if (Constants.DEBUG_MODE) {
            SmartDashboard.putNumber("Vertical Speed", vertSpeed);
        }
        if (Math.abs(vertSpeed) > Constants.MinTurretSpeed) {
            setVertMotor(vertSpeed);
        } else {
            setVertMotor(0);
        }
    }

    @Override
    public void periodic() {
        if (autoTarget) {
            distanceChange(true);
        }
        PhotonTrackedTarget target = camera();
        horEncoderPos = EncoderWrapper.getRawYaw();
        SmartDashboard.putNumber("Vertical Encoder", vertMotor.getSelectedSensorPosition());
        if (Constants.DEBUG_MODE) {
            SmartDashboard.putNumber("Encoder Position", horEncoderPos);
            SmartDashboard.putNumber("Encoder Winds", EncoderWrapper.getWinds());
        }
        gyroYaw = Robot.getGyro();
        double goalYaw = autoTarget(target);
        horMotorControl(goalYaw);
        vertMotorControl(target);
        prevAutoTarget = autoTarget;
    }

    public static TurretSubsystem getInstance() {
        if (instance == null) {
            instance = new TurretSubsystem();
            instance.setDefaultCommand(new TurretDefaultCommand());
        }

        return instance;
    }

    public static class EncoderWrapper {
        private static final Encoder encoder = new Encoder(0, 1);
        private static final double stepsPerDegree = 25000.0 / 360.0; //very estimate much wow

        public static Encoder getEncoder() {
            return encoder;
        }

        public static double getYaw() { // positive is counter-clockwise
            return (encoder.get() / stepsPerDegree) % 360;
        }

        public static double getRawYaw() {
            return encoder.get() / stepsPerDegree;
        }

        public static int getWinds() {
            return (int) ((encoder.get() / stepsPerDegree) / 360);
        }

        public static void reset() {
            encoder.reset();
        }
    }

    public enum TurretMode {
        //region Speed-based Presets
        OFF(0.0, 0.0, 0, 0, "OFF"),
        low(1500, -1500, 3000, 1, "low"),
        m1(2374.587, -2594.667, 502.01, 1, "1m"),
        m2(2582.784, -2751.224, 728.141, 2, "2m"),
        m3(2790.981, -2907.781, 954.272, 3, "3m"),
        m4(2999.178, -3064.338, 1180.403, 4, "4m"),
        m5(3207.375, -3220.895, 1406.534, 5, "5m"),
        m6(3415.572, -3377.452, 1632.665, 6, "6m"),
        m7(3623.769, -3534.009, 1858.796, 7, "7m"),
        m8(3831.966, -3690.566, 2084.927, 8, "8m"),
        m9(4040.163, -3847.123, 2311.058, 9, "9m"),
        m10(4248.36, -4003.68, 2537.189, 10, "10m");
        //endregion


        private final double frontSpeed;
        private final double backSpeed;
        private final double vertPosition;
        private final double distance;
        public final String name;

        TurretMode(double frontSpeed, double backSpeed, double vertPosition, double distance, String name) {
            this.frontSpeed = frontSpeed;
            this.backSpeed = backSpeed;
            this.vertPosition = vertPosition;
            this.distance = distance;
            this.name = name;
        }

        public double getFrontSpeed() {
            return frontSpeed;
        }

        public double getBackSpeed() {
            return backSpeed;
        }

        public double getVertPosition() {
            return vertPosition;
        }

        public double getDistance() {
            return distance;
        }

        public String getName() {
            return name;
        }
    }
}