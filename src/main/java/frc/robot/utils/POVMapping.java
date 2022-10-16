package frc.robot.utils;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.GlobalVariables;

public class POVMapping {
    Joystick joystick;
    private POVMapping(Builder builder) {
        this.joystick = builder.joystick;
    }
    public boolean isFront() {
        int POV = joystick.getPOV();
        return ((POV == 315 || POV == 0 || POV == 45) && GlobalVariables.climberEnabled);
    }
    public boolean isBack() {
        int POV = joystick.getPOV();
        return ((POV == 135 || POV == 180 || POV == 225) && GlobalVariables.climberEnabled);
    }
    public boolean isActive() {
        int POV = joystick.getPOV();
        return !(POV == -1);
    }
    public static Builder getBuilder() {
        return new Builder();
    }
    public static class Builder {
        private Builder() {

        }
        Joystick joystick;
        public Builder setJoystick(Joystick joystick) {
            this.joystick = joystick;
            return this;
        }
        public POVMapping build() {
            return new POVMapping(this);
        }
    }
}
