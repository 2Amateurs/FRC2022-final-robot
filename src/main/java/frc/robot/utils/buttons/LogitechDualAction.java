package frc.robot.utils.buttons;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class LogitechDualAction {
    public final JoystickButton btnX;
    public final JoystickButton btnA;
    public final JoystickButton btnB;
    public final JoystickButton btnY;
    public final JoystickButton btnLB;
    public final JoystickButton btnRB;
    public final JoystickButton btnLT;
    public final JoystickButton btnRT;
    public final JoystickButton btnBACK;
    public final JoystickButton btnSTART;
    public final JoystickButton btnLSTICK;
    public final JoystickButton btnRSTICK;

    public static final int axsLX = 0;
    public static final int axsLY = 1;
    public static final int axsRX = 2;
    public static final int axsRY = 3;

    public LogitechDualAction(Joystick joystick) {
        btnX = new JoystickButton(joystick, 1);
        btnA = new JoystickButton(joystick, 2);
        btnB = new JoystickButton(joystick, 3);
        btnY = new JoystickButton(joystick, 4);
        btnLB = new JoystickButton(joystick, 5);
        btnRB = new JoystickButton(joystick, 6);
        btnLT = new JoystickButton(joystick, 7);
        btnRT = new JoystickButton(joystick, 8);
        btnBACK = new JoystickButton(joystick, 9);
        btnSTART = new JoystickButton(joystick, 10);
        btnLSTICK = new JoystickButton(joystick, 11);
        btnRSTICK = new JoystickButton(joystick, 12);
    }
}

