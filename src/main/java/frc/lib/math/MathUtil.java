package frc.lib.math;

public class MathUtil {
    private MathUtil() {}

    /**
     * @return The magnitude of the vector &lt;x, y&gt;
     */
    public static double magnitude(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * The cosine function in degrees mode
     */
    public static double cos(double degrees) {
        return Math.cos(Math.toRadians(degrees));
    }

    /**
     * The sin function in degrees mode
     */
    public static double sin(double degrees) {
        return Math.sin(Math.toRadians(degrees));
    }

    /**
     * The tan function in degrees mode
     */
    public static double tan(double degrees) {
        return sin(degrees) / cos(degrees);
    }


    /**
     * The atan2 function in degrees mode
     *
     * @return the angle in degrees that the vector &lt;x, y&gt; makes with the
     * positive x-axis, measured counterclockwise
     */
    public static double atan(double y, double x) {
        return Math.toDegrees(Math.atan2(y, x));
    }

    /**
     * @return the angle in degrees that the line y = slope * x makes with the positive
     * x-axis, measured counterclockwise
     */
    public static double atan(double slope) {
        return Math.toDegrees(Math.atan(slope));
    }

    /**
     * The modulo function.
     *
     * @return a value v such that v is in [0, m) and v = x - N * m where N is an integer
     */
    public static double mod(double x, double m) {
        return (x % m + m) % m;
    }

    /**
     * Constrains x to the range [min, max). values outside the range will wrap around.
     * For min = 0, this behaves the same as mod
     *
     * @return a value v such that v is in [min, max) and v = x - N * (max - min) where N is an integer
     */
    public static double mod(double x, double min, double max) {
        return mod(x - min, max - min) + min;
    }

    /**
     * Rounds num to the nearest multiple of precision
     */
    public static double round(double num, double precision) {
        return Math.round(num / precision) * precision;
    }

    public static double max(double... arr) {
        double max = Double.NEGATIVE_INFINITY;
        for (double i : arr) if (i > max) max = i;
        return max;
    }

    public static double signedPow(double a, double p) {
        return Math.signum(a) * Math.pow(Math.abs(a), p);
    }

    public static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    public static double clamp(double t, double a, double b) {
        return t < a ? a : t > b ? b : t;
    }
}