package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
public final class Constants {
    public static final double DEADZONE_VALUE = 0.08;
    public static final int POSE_WINDOW_LENGTH = 1;
    public static final double INTAKE_PIVOT_UP_MULTIPLIER = 2;


    public enum CurrentRobot {
        ZEUS, CADENZA, SIM
    }
    public static final CurrentRobot currentRobot = CurrentRobot.CADENZA;

    private static final double[] oldOffsets = {
            -54.98, -122.4, 74.44, 121.92
    };
    private static final double[] newOffsets = {
            Rotation2d.fromRotations(-0.119).getDegrees(),
            Rotation2d.fromRotations(-0.07).getDegrees(),
            Rotation2d.fromRotations(-0.265 + 0.5).getDegrees(),
            Rotation2d.fromRotations(0.474).getDegrees(),
    };
    static final double[] swerveOffsets = switch (currentRobot) {
        case ZEUS -> oldOffsets;
        case CADENZA, SIM -> newOffsets;
    };
}
