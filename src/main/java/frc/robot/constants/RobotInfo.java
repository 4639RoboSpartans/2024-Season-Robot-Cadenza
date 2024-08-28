package frc.robot.constants;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class RobotInfo {
  public static final class SwerveInfo {
    public static final double centerToWheelMeters = 0.245;

    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(centerToWheelMeters, centerToWheelMeters),
            new Translation2d(centerToWheelMeters, -centerToWheelMeters),
            new Translation2d(-centerToWheelMeters, centerToWheelMeters),
            new Translation2d(-centerToWheelMeters, -centerToWheelMeters));

    public static final double CURRENT_MAX_ROBOT_MPS = 9;
    public static final double TELOP_ROTATION_SPEED = 6;
    public static final double TELEOP_AIM_SPEED = 8; // TODO: tune this
    public static final PIDTemplate TeleopRotationPID = new PIDTemplate(0.008, 0, 0);
    public static double TeleopTranslationScalar = 5;
    public static double AimTranslationDeadzone = 0.01;
    public static double AimTranslationScalar = 0.5;

    public static final PIDConstants TranslationPID = new PIDConstants(2, 0, 0.0);
    public static final PIDConstants RotationPID = new PIDConstants(1, 0, 0.0);
  }

  public static final class IntakeInfo {

    public static final double INTAKE_SPEED = -0.85;
    public static final double INTAKE_PIVOT_DEFAULT_SETPOINT = 0.63;
    public static final double INTAKE_PIVOT_EXTENDED_SETPOINT = 0.87;
    public static final PIDTemplate INTAKE_PIVOT_PID_CONSTANTS = new PIDTemplate(.6, 0, 0.02);
  }

  public static final class HopperInfo {

    public static final double HOPPER_SPEED = 0.8; // was 0.7
  }

  public static final class ClimberInfo {
    public static final double CLIMBER_SPEED = 0.9;
  }

  public static final class AimInfo {
    public static final double AIM_TIME = 0.5;
  }

  public static final class VisionInfo {
    public static final double visionScalar = 0.5;
  }

  public static class ShooterInfo {

    public enum ShootingMode {
      AUTO_SPEAKER,
      SPEAKER,
      AMP,
      TRAP,
      IDLE,
      LAUNCH,
      INTAKE
    }

    public static final ShooterSetpointMeasurement[] measurements = {
      new ShooterSetpointMeasurement(1.4, new ShooterSetpoint(24.75, .482)),
      new ShooterSetpointMeasurement(1.5, new ShooterSetpoint(24.75, .4845)),
      new ShooterSetpointMeasurement(1.6, new ShooterSetpoint(24.75, .4895)),
      new ShooterSetpointMeasurement(1.7, new ShooterSetpoint(24.75, .4925)),
      new ShooterSetpointMeasurement(1.8, new ShooterSetpoint(24.75, .495)),
      new ShooterSetpointMeasurement(1.9, new ShooterSetpoint(25, .4999)),
      new ShooterSetpointMeasurement(2.0, new ShooterSetpoint(25, .505)),
      new ShooterSetpointMeasurement(2.1, new ShooterSetpoint(25, .509)),
      new ShooterSetpointMeasurement(2.2, new ShooterSetpoint(25.5, .5115)),
      new ShooterSetpointMeasurement(2.3, new ShooterSetpoint(26, .5145)),
      new ShooterSetpointMeasurement(2.4, new ShooterSetpoint(26.25, .517)),
      new ShooterSetpointMeasurement(2.5, new ShooterSetpoint(26.5, .519)),
      new ShooterSetpointMeasurement(2.6, new ShooterSetpoint(26.5, .5235)),
      new ShooterSetpointMeasurement(2.7, new ShooterSetpoint(26.75, .5305)),
      new ShooterSetpointMeasurement(2.8, new ShooterSetpoint(27, .531)),
      new ShooterSetpointMeasurement(2.85, new ShooterSetpoint(27.5, .531)),
      new ShooterSetpointMeasurement(2.9, new ShooterSetpoint(27.75, .532)),
      new ShooterSetpointMeasurement(2.95, new ShooterSetpoint(28, .533)),
      new ShooterSetpointMeasurement(3.0, new ShooterSetpoint(28.5, .53625)),
      new ShooterSetpointMeasurement(3.1, new ShooterSetpoint(29.25, .536)),
      new ShooterSetpointMeasurement(3.2, new ShooterSetpoint(29.5, .5415)),
      new ShooterSetpointMeasurement(3.3, new ShooterSetpoint(30.6, .5445)),
      new ShooterSetpointMeasurement(3.4, new ShooterSetpoint(33, .545)),
      new ShooterSetpointMeasurement(3.5, new ShooterSetpoint(35, .547)),
      new ShooterSetpointMeasurement(3.6, new ShooterSetpoint(37, .5535)),
      new ShooterSetpointMeasurement(3.7, new ShooterSetpoint(39.5, .5575)),
      new ShooterSetpointMeasurement(3.8, new ShooterSetpoint(42, .559)),
      new ShooterSetpointMeasurement(3.9, new ShooterSetpoint(45, .5592)),
      new ShooterSetpointMeasurement(4.0, new ShooterSetpoint(47.5, .5632)),
      new ShooterSetpointMeasurement(4.1, new ShooterSetpoint(50, .5648)),
      new ShooterSetpointMeasurement(4.2, new ShooterSetpoint(52.5, .5655)),
      new ShooterSetpointMeasurement(4.3, new ShooterSetpoint(54.5, .5665)),
      new ShooterSetpointMeasurement(4.4, new ShooterSetpoint(55.75, .568)),
      new ShooterSetpointMeasurement(4.5, new ShooterSetpoint(56.5, .569)),
      new ShooterSetpointMeasurement(4.6, new ShooterSetpoint(57.25, .5705)),
      new ShooterSetpointMeasurement(4.7, new ShooterSetpoint(58.5, .571)),
      new ShooterSetpointMeasurement(4.9, new ShooterSetpoint(58.75, .571)),
      // Max speed is 58.5
      // Min angle is .9

    };

    public static final double SHOOTER_IDLE_SPEED = 0.2;
    public static final double SHOOTER_AUTON_IDLE_SPEED = 0.35;
    public static final double SHOOTER_INTAKE_SPEED = -0.35;
    public static final double SHOOTER_PIVOT_BOTTOM_SETPOINT = .579;
    public static final double AngleOffset = -0.005;

    public static final double SHOOTER_PIVOT_ERROR = 0.01;

    public static final ShooterSetpoint SHOOTER_AMP_SETPOINT = new ShooterSetpoint(6.8, .49);
    public static final ShooterSetpoint SHOOTER_SPEAKER_SETPOINT = measurements[1].setpoint();
    public static final ShooterSetpoint SHOOTER_TRAP_SETPOINT = new ShooterSetpoint(20, .499);
    public static final ShooterSetpoint SHOOTER_LAUNCH_SETPOINT = new ShooterSetpoint(40, .529);

    public static final ShooterSetpoint SHOOTER_INTAKE_SETPOINT = new ShooterSetpoint(-15, .529);

    public static final double SHOOTER_VOLTAGE = 10;

    public static final PIDTemplate SHOOTER_AIM_PID_CONSTANTS = new PIDTemplate(4, 0, 0);
  }
}
