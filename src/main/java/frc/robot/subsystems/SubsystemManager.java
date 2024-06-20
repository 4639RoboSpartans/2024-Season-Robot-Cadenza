package frc.robot.subsystems;

import static frc.robot.constants.Constants.currentRobot;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.IDs;
import frc.robot.led.DummyLEDStrip;
import frc.robot.led.LEDStrip;
import frc.robot.led.PhysicalLEDStrip;
import frc.robot.oi.OI;
import frc.robot.subsystems.aim.AimSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.DummyClimberSubsystem;
import frc.robot.subsystems.climber.IClimberSubsystem;
import frc.robot.subsystems.hopper.DummyHopperSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.intake.DummyIntakeSubsystem;
import frc.robot.subsystems.intake.IIntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.sensors.IRSensor;
import frc.robot.subsystems.shooter.DummyShooterSubsystem;
import frc.robot.subsystems.shooter.FalconShooterSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;
import frc.robot.subsystems.shooter.pivot.DummyShooterPivotSubsystem;
import frc.robot.subsystems.shooter.pivot.IShooterPivotSubsystem;
import frc.robot.subsystems.shooter.pivot.NeoShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveConstantsCreator;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveTrainConstants;
import frc.robot.subsystems.swerve.SwerveModuleConstants;

public class SubsystemManager {
  private static NavX navX;
  private static IRSensor irSensor;

  private static OI oi;

  private static LEDStrip ledStrip;

  private static SwerveDriveSubsystem swerveDrive;

  private static IShooterSubsystem shooter;
  private static IShooterPivotSubsystem shooterPivot;
  private static AimSubsystem aimSubsystem;
  private static IIntakeSubsystem intake;
  private static IHopperSubsystem hopper;
  private static IClimberSubsystem climber;

  public static NavX getNavX() {
    if (navX == null) {
      navX = new NavX();
    }
    return navX;
  }

  public static IRSensor getIRSensor() {
    if (irSensor == null) {
      irSensor = new IRSensor();
    }
    return irSensor;
  }

  public static LEDStrip getLedStrip() {
    if (ledStrip == null) {
      ledStrip =
          switch (currentRobot) {
            case ZEUS -> new DummyLEDStrip();
            case SIREN -> new PhysicalLEDStrip(0, 64);
          };
    }
    return ledStrip;
  }

  public static SwerveDriveSubsystem getSwerveDrive() {
    if (swerveDrive == null) {
      SwerveDriveTrainConstants drivetrain =
          new SwerveDriveTrainConstants().withPigeon2Id(1).withCANbusName("Fred").withTurnKp(5);

      Slot0Configs steerGains = new Slot0Configs();
      Slot0Configs driveGains = new Slot0Configs();

      {
        steerGains.kP = 30;
        steerGains.kD = 0.2;
        driveGains.kP = 1;
      }

      SwerveDriveConstantsCreator m_constantsCreator =
          new SwerveDriveConstantsCreator(
              6.55, // 10:1 ratio for the drive motor
              12.8, // 12.8:1 ratio for the steer motor
              2, // 3 inch radius for the wheels
              17, // Only apply 17 stator amps to prevent slip
              steerGains, // Use the specified steer gains
              driveGains, // Use the specified drive gains
              true // CANcoder not reversed from the steer motor. For WCP Swerve X this should be
              // true.
              );

      SwerveModuleConstants frontRight =
          m_constantsCreator.createModuleConstants(
              1,
              2,
              9,
              -0.538818,
              Units.inchesToMeters(27.5 / 2.0),
              Units.inchesToMeters(-22.0 / 2.0));

      SwerveModuleConstants frontLeft =
          m_constantsCreator.createModuleConstants(
              3,
              4,
              10,
              -0.474609,
              Units.inchesToMeters(27.5 / 2.0),
              Units.inchesToMeters(22.0 / 2.0));
      SwerveModuleConstants backRight =
          m_constantsCreator.createModuleConstants(
              5,
              6,
              11,
              -0.928467,
              Units.inchesToMeters(-27.5 / 2.0),
              Units.inchesToMeters(-22.0 / 2.0));
      SwerveModuleConstants backLeft =
          m_constantsCreator.createModuleConstants(
              7,
              8,
              12,
              -0.756348,
              Units.inchesToMeters(-27.5 / 2.0),
              Units.inchesToMeters(22.0 / 2.0));

      swerveDrive =
          new SwerveDriveSubsystem(drivetrain, frontLeft, frontRight, backLeft, backRight);
    }
    return swerveDrive;
  }

  public static IShooterPivotSubsystem getShooterPivot(IShooterSubsystem shooter) {
    if (shooterPivot == null) {
      shooterPivot =
          switch (currentRobot) {
            case ZEUS -> new DummyShooterPivotSubsystem();
            case SIREN -> new NeoShooterPivotSubsystem(
                IDs.SHOOTER_PIVOT_MOTOR_LEFT, IDs.SHOOTER_PIVOT_MOTOR_RIGHT, shooter);
          };
    }
    return shooterPivot;
  }

  public static IShooterSubsystem getShooter() {
    if (shooter == null) {
      shooter =
          switch (currentRobot) {
            case ZEUS -> new DummyShooterSubsystem();
            case SIREN -> new FalconShooterSubsystem(
                IDs.SHOOTER_SHOOTER_LEFT_MOTOR, IDs.SHOOTER_SHOOTER_RIGHT_MOTOR);
          };
    }
    return shooter;
  }

  public static AimSubsystem getAimSubsystem() {
    if (aimSubsystem == null) {
      aimSubsystem = new AimSubsystem(swerveDrive);
    }
    return aimSubsystem;
  }

  public static IIntakeSubsystem getIntake() {
    if (intake == null) {
      intake =
          switch (currentRobot) {
            case ZEUS -> new DummyIntakeSubsystem();
            case SIREN -> new IntakeSubsystem(
                IDs.INTAKE_PIVOT_MOTOR_LEFT,
                IDs.INTAKE_PIVOT_MOTOR_RIGHT,
                IDs.INTAKE_MOTOR,
                IDs.INTAKE_ENCODER_DIO_PORT);
              // case SIREN -> new DummyIntakeSubsystem();
          };
    }
    ;
    return intake;
  }

  public static IHopperSubsystem getHopper() {
    if (hopper == null) {
      hopper =
          switch (currentRobot) {
            case ZEUS -> new DummyHopperSubsystem();
            case SIREN -> new HopperSubsystem(IDs.HOPPER_MOTOR);
          };
    }
    return hopper;
  }

  public static IClimberSubsystem getClimber() {
    if (climber == null) {
      climber =
          switch (currentRobot) {
            case ZEUS -> new DummyClimberSubsystem();
            case SIREN -> new ClimberSubsystem(IDs.CLIMBER_LEFT, IDs.CLIMBER_RIGHT);
          };
    }
    return climber;
  }

  public static OI getOI() {
    if (oi == null) {
      oi = new OI();
    }
    return oi;
  }
}
