package frc.robot.utils.buttons;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ToggleBoard {
    public final JoystickButton btn1;
    public final JoystickButton btn2;
    public final JoystickButton btn3;
    public final JoystickButton btn4;
    public final JoystickButton btn5;
    public final JoystickButton btn6;
    public final JoystickButton btn7;
    public final JoystickButton btn8;
    public final JoystickButton btn9;
    public final JoystickButton btn10;
    public final JoystickButton btn11;
    public final JoystickButton btn12;
    public final JoystickButton btn13;
    public final JoystickButton btn14;
    public final JoystickButton btn15;
    public final JoystickButton btn16;

    public ToggleBoard(Joystick joystick) {
        btn1 = new JoystickButton(joystick, 1);
        btn2 = new JoystickButton(joystick, 2);
        btn3 = new JoystickButton(joystick, 3);
        btn4 = new JoystickButton(joystick, 4);
        btn5 = new JoystickButton(joystick, 5);
        btn6 = new JoystickButton(joystick, 6);
        btn7 = new JoystickButton(joystick, 7);
        btn8 = new JoystickButton(joystick, 8);
        btn9 = new JoystickButton(joystick, 9);
        btn10 = new JoystickButton(joystick, 10);
        btn11 = new JoystickButton(joystick, 11);
        btn12 = new JoystickButton(joystick, 12);
        btn13 = new JoystickButton(joystick, 13);
        btn14 = new JoystickButton(joystick, 14);
        btn15 = new JoystickButton(joystick, 15);
        btn16 = new JoystickButton(joystick, 16);
    }
}

