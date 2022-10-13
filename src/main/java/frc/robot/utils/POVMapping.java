package frc.robot.utils;

import edu.wpi.first.wpilibj.Joystick;

public class POVMapping {
    Joystick joystick;
    private POVMapping(Builder builder) {
        this.joystick = builder.joystick;
    }
    public boolean isFront() {
        return (joystick.getPOV() == 0);
    }
    public boolean isBack() {
        return (joystick.getPOV() == 180);
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
