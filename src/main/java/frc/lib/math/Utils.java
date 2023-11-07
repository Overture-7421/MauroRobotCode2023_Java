package frc.lib.math;

public class Utils {
    public static <T extends Number> int sgn(T vol) {
        return Double.compare(vol.doubleValue(), 0);
    }

    /**
     * 
     * Apply a filter to an Axis, so that when the driver is not using it, the
     * robot doesn't move randomly. It applies an Exponential curve for finer
     * control at smaller inputs.
     * 
     * @param axisValue       The raw axis value given by the Joystick
     * @param deadzone        The threshold for when the axis is considered as valid
     * @param exponentialGain How much of an Exponential curve we want
     * @return The filtered axis value
     */
    public static double applyAxisFilter(double axisValue, double deadzone, double exponentialGain) {
        double axisMag = Math.abs(axisValue);
        if (axisMag < deadzone)
            return 0.0;

        double res = exponentialGain * Math.pow((axisMag - deadzone) / (1 - deadzone), 3)
                + (1 - exponentialGain) * (axisMag - deadzone) / (1 - deadzone);

        return res * sgn(axisValue);
    }

    public static double applyAxisFilter(double axisValue) {
        double deadzone = 0.05;
        double exponentialGain = 0.5;

        double axisMag = Math.abs(axisValue);
        if (axisMag < deadzone)
            return 0.0;

        double res = exponentialGain * Math.pow((axisMag - deadzone) / (1 - deadzone), 3)
                + (1 - exponentialGain) * (axisMag - deadzone) / (1 - deadzone);

        return res * sgn(axisValue);
    }
}
