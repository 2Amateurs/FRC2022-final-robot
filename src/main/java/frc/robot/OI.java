package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.AxisMapping;
import frc.robot.utils.POVMapping;
import frc.robot.utils.buttons.LogitechAttack;
import frc.robot.utils.buttons.SwitchPro;
import frc.robot.utils.buttons.ToggleBoard;

import java.util.ArrayList;
import java.util.List;

public class OI {
    private static OI instance = null;

    private final List<Layout> layouts = new ArrayList<>();
    public static DriveSystem driveSystem;

    public AxisMapping leftDriveAxis;
    public AxisMapping leftTurnAxis;
    public AxisMapping rightDriveAxis;

    public AxisMapping turretHorAim;
    public AxisMapping turretVertAim;
    public AxisMapping pneumaticAxis;


    private OI() {
        layouts.add(new TwoGamepad());
        layouts.add(new InnovatorStation());
    }

    public Layout getLayout(DriveSystem driveSystem) {
        OI.driveSystem = driveSystem;
        Layout layout = layouts.get(driveSystem.ordinal());

        leftDriveAxis = layout.leftDriveAxis;
        leftTurnAxis = layout.leftTurnAxis;
        rightDriveAxis = layout.rightDriveAxis;

        turretHorAim = layout.turretHorAim;
        turretVertAim = layout.turretVertAim;
        pneumaticAxis = layout.pneumaticAxis;

        return layout;
    }

    public class TwoGamepad extends Layout {
        @Override
        void assignButtons() {
            gyroResetBtn = driveGamepad.btnSCREENSHOT;
            driveModeDownBtn = driveGamepad.btnL;
            driveModeUpBtn = driveGamepad.btnR;
            driveFastModeBtn = driveGamepad.btnHOME;

            shooterBtn = new Trigger(); //was previously mapped to btnZL, but I unmapped it for outreach events
            storageInBtn = payloadGamepad.btnZR;
            storageOutBtn = payloadGamepad.btnL;
            turretModeUpBtn = payloadGamepad.btnA;
            turretModeDownBtn = payloadGamepad.btnB;
            intakeInBtn = payloadGamepad.btnZR;
            intakeOutBtn = payloadGamepad.btnY;
            turretAutoToggleBtn = payloadGamepad.btnHOME;
            rightClimberUp2 = payloadGamepad.btnR;
            distanceOffsetUp = payloadGamepad.btnPLUS;
            distanceOffsetDown = payloadGamepad.btnMINUS;
            climberHighSpeedUp = payloadGamepad.btnX;

            leftDriveAxis = new AxisMapping.Builder(driveGamepadJoystick, SwitchPro.axsLY).build();
            leftTurnAxis = new AxisMapping.Builder(driveGamepadJoystick, SwitchPro.axsLX).inverted(true).build();
            rightDriveAxis = new AxisMapping.Builder(driveGamepadJoystick, SwitchPro.axsRY).build();

            turretHorAim = new AxisMapping.Builder(payloadGamepadJoystick, SwitchPro.axsLX).build();
            turretVertAim = new AxisMapping.Builder(payloadGamepadJoystick, SwitchPro.axsLY).build();
            pneumaticAxis = new AxisMapping.Builder(payloadGamepadJoystick, SwitchPro.axsRY).build();

            automaticShoot = payloadGamepad.btnZL;
        }
    }

    public class InnovatorStation extends Layout {
        @Override
        void assignButtons() {
            driveModeDownBtn = leftFlightStick.btn2;
            driveModeUpBtn = rightFlightStick.btn2;
            gyroResetBtn = leftFlightStick.btn7;

            shooterBtn = payloadGamepad.btnZL;
            storageInBtn = payloadGamepad.btnZR;
            storageOutBtn = payloadGamepad.btnL;
            turretModeUpBtn = payloadGamepad.btnA;
            turretModeDownBtn = payloadGamepad.btnB;
            intakeInBtn = payloadGamepad.btnZR;
            intakeOutBtn = payloadGamepad.btnY;
            turretAutoToggleBtn = payloadGamepad.btnHOME;
            rightClimberUp2 = payloadGamepad.btnR;
            distanceOffsetUp = payloadGamepad.btnPLUS;
            distanceOffsetDown = payloadGamepad.btnMINUS;
            climberHighSpeedUp = payloadGamepad.btnX;

            climberSubsystemToggle = payloadGamepad.btnSCREENSHOT;

            leftDriveAxis = new AxisMapping.Builder(leftFlightStickJoystick, LogitechAttack.axsY).deadzoneValue(0.1).build();
            leftTurnAxis = new AxisMapping.Builder(leftFlightStickJoystick, LogitechAttack.axsX).build();
            rightDriveAxis = new AxisMapping.Builder(rightFlightStickJoystick, LogitechAttack.axsY).deadzoneValue(0.1).build();

            turretHorAim = new AxisMapping.Builder(payloadGamepadJoystick, SwitchPro.axsLX).build();
            turretVertAim = new AxisMapping.Builder(payloadGamepadJoystick, SwitchPro.axsLY).build();
            pneumaticAxis = new AxisMapping.Builder(payloadGamepadJoystick, SwitchPro.axsRY).build();

            driveToggleIntake = rightFlightStick.btnTRIGGER;
            driveToggleArms = leftFlightStick.btnTRIGGER;

            rightClimberDown = rightFlightStick.btn3;
            rightClimberUp = rightFlightStick.btn5;
            leftClimberDown = leftFlightStick.btn4;
            leftClimberUp = leftFlightStick.btn6;
            rightStickPOV = POVMapping.getBuilder().setJoystick(rightFlightStickJoystick).build();
            leftStickPOV = POVMapping.getBuilder().setJoystick(leftFlightStickJoystick).build();

            rightClimberUp = new Trigger(() -> rightStickPOV.isFront());
            rightClimberDown = new Trigger(()-> rightStickPOV.isBack());
            rightClimberStop = new Trigger(()-> (rightStickPOV.isFront() && rightStickPOV.isBack()) == false);
            leftClimberUp = new Trigger(() -> leftStickPOV.isFront());
            leftClimberDown = new Trigger(()-> leftStickPOV.isBack());
            leftClimberStop = new Trigger(()-> (leftStickPOV.isFront() && leftStickPOV.isBack()) == false);
        }
    }

    public abstract class Layout {
        Joystick leftFlightStickJoystick = new Joystick(0);
        Joystick rightFlightStickJoystick = new Joystick(1);
        Joystick payloadGamepadJoystick = new Joystick(2);
        Joystick driveGamepadJoystick = new Joystick(3);
        //Joystick toggleBoardJoystick = new Joystick(4);

        LogitechAttack leftFlightStick = new LogitechAttack(leftFlightStickJoystick);
        LogitechAttack rightFlightStick = new LogitechAttack(rightFlightStickJoystick);
        public POVMapping rightStickPOV;
        public POVMapping leftStickPOV;
        public Trigger leftClimberStop = new Trigger();
        public Trigger rightClimberStop = new Trigger();
        SwitchPro payloadGamepad = new SwitchPro(payloadGamepadJoystick);
        SwitchPro driveGamepad = new SwitchPro(driveGamepadJoystick);
        //ToggleBoard toggleBoard = new ToggleBoard(toggleBoardJoystick);

        public Trigger gyroResetBtn = new Trigger();
        public Trigger driveModeDownBtn = new Trigger();
        public Trigger driveModeUpBtn = new Trigger();
        public Trigger driveFastModeBtn = new Trigger();
        public Trigger driveToggleIntake = new Trigger();
        public Trigger driveToggleArms = new Trigger();

        public Trigger rightClimberDown = new Trigger();
        public Trigger rightClimberUp = new Trigger();
        public Trigger rightClimberUp2 = new Trigger();
        public Trigger leftClimberDown = new Trigger();
        public Trigger leftClimberUp = new Trigger();
        public Trigger climberHighSpeedUp = new Trigger();

        public Trigger intakeInBtn = new Trigger();
        public Trigger intakeOutBtn = new Trigger();
        public Trigger turretModeDownBtn = new Trigger();
        public Trigger turretModeUpBtn = new Trigger();
        public Trigger storageInBtn = new Trigger();
        public Trigger storageOutBtn = new Trigger();
        public Trigger turretAutoToggleBtn = new Trigger();
        public Trigger shooterBtn = new Trigger();
        public Trigger distanceOffsetUp = new Trigger();
        public Trigger distanceOffsetDown = new Trigger();

        public Trigger climberSubsystemToggle = new Trigger();

        public AxisMapping leftDriveAxis;
        public AxisMapping leftTurnAxis;
        public AxisMapping rightDriveAxis;

        public AxisMapping turretHorAim;
        public AxisMapping turretVertAim;
        public AxisMapping pneumaticAxis;

        Trigger automaticShoot;

        Layout() {
            assignButtons();
        }

        abstract void assignButtons();
    }

    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }

        return instance;
    }

    public enum DriveSystem {
        TWO_SWITCH,
        INNOVATOR_STATION
    }
}
