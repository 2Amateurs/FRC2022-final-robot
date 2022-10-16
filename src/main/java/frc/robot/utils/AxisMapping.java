package frc.robot.utils;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

/**
 * A wrapper class for the axis of a {@link Joystick Joystick} designed to have the ability to invert, override, or
 * apply a deadzone to the value before passing it on
 */
public class AxisMapping {
    private final Joystick joystick;
    private final int axisID;
    private boolean inverted;
    private boolean overrideControl = false;
    private double overrideValue = 0.0;
    private boolean deadzone = true;
    private double deadzoneValue = Constants.DeadzoneDefault;
    private boolean driveControl = false;

    private AxisMapping(Builder builder) {
        this.joystick = builder.joystick;
        this.axisID = builder.axisID;
        this.inverted = builder.inverted;
        this.overrideControl = builder.overrideControl;
        this.overrideValue = builder.overrideValue;
        this.deadzone = builder.deadzone;
        this.deadzoneValue = builder.deadzoneValue;
        this.driveControl = builder.driveControl;
    }

    /**
     * A double with the value inverted if needed, deadzone applied if needed, and overrided if needed.
     */
    public double getValue() {
        double value = joystick.getRawAxis(axisID);
        if (overrideControl) {
            return overrideValue;
        }
        if (inverted) {
            value *= -1;
        }
        if (driveControl) {

        }
        if (this.deadzone && Math.abs(value) <= deadzoneValue) {
            value = 0.0;
        }
        return value;
    }

    /**
     * A double of the direct value from the axis without any modifications
     */
    public double getRawValue() {
        return joystick.getRawAxis(axisID);
    }

    /**
     * Whether the axis is inverted
     */
    public boolean isInverted() {
        return inverted;
    }

    /**
     * Whether the axis is inverted
     */
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    /**
     * Whether the value is overridden
     */
    public boolean isOverrideControl() {
        return overrideControl;
    }

    /**
     * Whether the value is overridden
     */
    public void setOverrideControl(boolean overrideControl) {
        this.overrideControl = overrideControl;
    }

    /**
     * The value to be used if overridden
     */
    public double getOverrideValue() {
        return overrideValue;
    }

    /**
     * The value to be used if overridden
     */
    public void setOverrideValue(double overrideValue) {
        this.overrideValue = overrideValue;
    }

    /**
     * @param override      Whether the value is overridden
     * @param overrideValue The value to be used if overridden
     */
    public void setOverride(boolean override, double overrideValue) {
        this.overrideControl = override;
        this.overrideValue = overrideValue;
    }

    /**
     * Set the override to false and the value to 0
     */
    public void releaseOverride() {
        setOverride(false, 0.0);
    }

    /**
     * Whether the deadzone is applied
     */
    public boolean isDeadzone() {
        return deadzone;
    }

    /**
     * Whether the deadzone is applied
     */
    public void setDeadzone(boolean deadzone) {
        this.deadzone = deadzone;
    }

    /**
     * The value of the deadzone to be applied
     */
    public double getDeadzoneValue() {
        return deadzoneValue;
    }

    /**
     * The value of the deadzone to be applied
     */
    public void setDeadzoneValue(double deadzoneValue) {
        this.deadzoneValue = deadzoneValue;
    }

    public static class Builder {
        private final Joystick joystick;
        private final int axisID;
        private boolean inverted = false;
        private boolean overrideControl = false;
        private double overrideValue = 0.0;
        private boolean deadzone = true;
        private double deadzoneValue = Constants.DeadzoneDefault;
        private boolean driveControl = false;

        public Builder(Joystick joystick, int axisID) {
            this.joystick = joystick;
            this.axisID = axisID;
        }

        public Builder inverted(boolean inverted) {
            this.inverted = inverted;
            return this;
        }
        public Builder setAsDriveControl() {
            this.driveControl = true;
            return this;
        }

        public Builder overrideControl(boolean overrideControl) {
            this.overrideControl = overrideControl;
            return this;
        }

        public Builder overrideValue(double overrideValue) {
            this.overrideValue = overrideValue;
            return this;
        }

        public Builder deadzone(boolean deadzone) {
            this.deadzone = deadzone;
            return this;
        }

        public Builder deadzoneValue(double deadzoneValue) {
            this.deadzoneValue = deadzoneValue;
            return this;
        }

        public AxisMapping build() {
            return new AxisMapping(this);
        }
    }
}
