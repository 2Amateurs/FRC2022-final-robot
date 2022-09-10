package frc.robot.utils.buttons;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class SwitchPro {

    public final JoystickButton btnY;
    public final JoystickButton btnB;
    public final JoystickButton btnA;
    public final JoystickButton btnX;
    public final JoystickButton btnL;
    public final JoystickButton btnR;
    public final JoystickButton btnZL;
    public final JoystickButton btnZR;
    public final JoystickButton btnMINUS;
    public final JoystickButton btnPLUS;
    public final JoystickButton btnLSTICK;
    public final JoystickButton btnRSTICK;
    public final JoystickButton btnHOME;
    public final JoystickButton btnSCREENSHOT;

    public static final int axsLX = 0;
    public static final int axsLY = 1;
    public static final int axsRX = 2;
    public static final int axsRY = 3;

    public SwitchPro(Joystick joystick) {
        btnY = new JoystickButton(joystick, 1);
        btnB = new JoystickButton(joystick, 2);
        btnA = new JoystickButton(joystick, 3);
        btnX = new JoystickButton(joystick, 4);
        btnL = new JoystickButton(joystick, 5);
        btnR = new JoystickButton(joystick, 6);
        btnZL = new JoystickButton(joystick, 7);
        btnZR = new JoystickButton(joystick, 8);
        btnMINUS = new JoystickButton(joystick, 9);
        btnPLUS = new JoystickButton(joystick, 10);
        btnLSTICK = new JoystickButton(joystick, 11);
        btnRSTICK = new JoystickButton(joystick, 12);
        btnHOME = new JoystickButton(joystick, 13);
        btnSCREENSHOT = new JoystickButton(joystick, 14);
    }
}
