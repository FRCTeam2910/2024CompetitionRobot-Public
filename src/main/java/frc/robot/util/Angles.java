package frc.robot.util;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Util class for normalizing and finding angles
 */
public class Angles {
    private Angles() {}

    /**
     * Normalizes the angle to be 0 to 2*π.
     *
     * @param angleInRadians The angle to normalize in radians.
     * @return A normalized angle between 0 and 2 * pi
     */
    public static double normalizeAnglePositive(final double angleInRadians) {
        return (angleInRadians % (2.0 * Math.PI) + 2.0 * Math.PI) % (2.0 * Math.PI);
    }

    /**
     * Normalizes the angle to be -pi <= result < pi.
     *
     * @param angleInRadians The angle to normalize in radians.
     * @return A normalized angle between -pi and pi
     */
    public static double normalizeAngle(final double angleInRadians) {
        var angle = normalizeAnglePositive(angleInRadians);
        if (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }

        return angle;
    }

    /**
     * Given 2 angles, return the shortest angular difference.
     *
     * @param fromAngleInRadians From angle in radians
     * @param toAngleInRadians To angle in radians
     * @return The result will always be -pi <= result < pi. Adding the result to "from" will
     *     always get you an equivalent angle to "to".
     */
    public static double shortestAngularDistance(final double fromAngleInRadians, final double toAngleInRadians) {
        return normalizeAngle(toAngleInRadians - fromAngleInRadians);
    }

    /**
     * Gets the angle between two 2d points (x,y) and (x1,y1)
     *
     * @param p1 start point [x, y]
     * @param p2 finish point [x1, y1]
     * @return The angle between p1 -> p2 in radians
     */
    public static double getLineAngle(final List<Double> p1, final List<Double> p2) {
        final double angle = Math.atan2(p2.get(1) - p1.get(1), p2.get(0) - p1.get(0));
        return normalizeAnglePositive(angle);
    }

    /**
     * Gets the angle between two 2d points (x,y) and (x1,y1)
     *
     * @param p1 start point
     * @param p2 finish point
     * @return The angle between p1 -> p2 in radians
     */
    public static double getLineAngle(final Translation2d p1, final Translation2d p2) {
        final double angle = Math.atan2(p2.getY() - p1.getY(), p2.getX() - p1.getX());
        return normalizeAnglePositive(angle);
    }
}
