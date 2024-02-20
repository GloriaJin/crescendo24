package frc.robot.utility.conversion;

public class AngleUtil {
        /**
     * Transform an angle measurement in degrees to radians
     * @param degrees The angle in degrees
     * @return The angle in radians
     */
    public static double degToRad(final double degrees) {
        return 2.0 * Math.PI * degrees / 360;
    }

    /**
     * Transform an angle measurement in radians to degrees
     * @param radians The angle in radians
     * @return The angle in degrees
     */
    public static double radToDeg(final double radians) {
        return 360 * radians / (2 * Math.PI);
    }

    /**
     * Transforms a given angle to a 0 to 360 degree range
     * @param angle The angle to be transformed
     * @return The unsigned range of the given angle, from 0 (inclusive) to 360 (exclusive)
     */
    public static double unsignedRangeDegrees(double angle) {
       angle = angle % 360;
       if (angle < 0) angle += 360;
       return angle;
    }

    /**
     * Transforms a given angle to a -180 to 180 degree range
     * @param angle The angle to be transformed
     * @return The signed range of the given angle, from -180 (exclusive) to 180 (inclusive)
     */
    public static double signedRangeDegrees(double angle) {
        angle = angle % 360;
        if (angle <= -180) angle += 360;
        if (angle > 180) angle -= 360;
        return angle;
    }

    /**
     * Transforms a given angle to a 0 to 1 rotation range
     * @param angle The angle to be transformed
     * @return The unsigned range of the given angle, from 0 (inclusive) to 1 (exclusive)
     */
    public static double unsignedRangeRotations(double angle) {
        angle = angle % 1;
        if (angle < 0) angle += 1;
        return angle;
    }

    /**
     * Returns the signed difference between two angles
     * @param reference The angle in degrees for the difference to be relative to
     * @param angle The angle in degrees to check for difference relative to the reference angle
     * @return The signed error in degrees from -180 (exclusive) to 180 (inclusive) between the two angles
     */
    public static double signedRangeDifferenceDegrees(final double reference, final double angle) {
        return unsignedRangeDegrees(angle - reference);
    }

    public static double getUnwrappedSetpoint(double current, double setpoint) {
        current = unsignedRangeDegrees(current);
        setpoint = unsignedRangeDegrees(setpoint);
        if (current > 180 && setpoint < 180 && (setpoint + 360) - current < current - setpoint)
            return setpoint + 360;
        else if (current < 180 && setpoint > 180 && current - (setpoint - 360) < setpoint - current)
            return setpoint - 360;

        return setpoint;
    }
}
