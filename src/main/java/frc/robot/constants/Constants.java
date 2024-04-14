package frc.robot.constants;

public final class Constants {
    public static final double DEADZONE_VALUE = 0.08;
    public static final int POSE_WINDOW_LENGTH = 1;
    public static final double INTAKE_PIVOT_UP_MULTIPLIER = 2;


    public enum CurrentRobot {
        ZEUS, SIREN
    }
    public static final CurrentRobot currentRobot = CurrentRobot.SIREN;

    private static final double[] oldOffsets = {
            -54.98, -122.4, 74.44, 121.92
    };
    private static final double[] newOffsets = {
            135.36, 155.23, -94.98, -88.23
    };
    static final double[] swerveOffsets = switch (currentRobot) {
        case ZEUS -> oldOffsets;
        case SIREN -> newOffsets;
    };
}
