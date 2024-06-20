package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.oi.OI;
import frc.robot.subsystems.SubsystemManager;

public class MovementUtil {
  private static Translation2d desiredPose, targetLock;
  private static Rotation2d desiredRotation;
  private static boolean toDesired, locked, useRotation;
  public static boolean configured = false;

  private static final OI oi = SubsystemManager.getOI();

  private static final PIDController rotationPID = new PIDController(0.08, 0, 0);
  private static final PIDController movementXPID = new PIDController(0.5, 0, 0);
  private static final PIDController movementYPID = new PIDController(0.5, 0, 0);

  public static void configure(
      Translation2d desiredPose,
      Translation2d targetLock,
      Rotation2d desiredRotation,
      boolean toDesired,
      boolean locked,
      boolean useRotation) {
    MovementUtil.desiredPose = desiredPose;
    MovementUtil.targetLock = targetLock;
    MovementUtil.desiredRotation = desiredRotation;
    MovementUtil.toDesired = toDesired;
    MovementUtil.locked = locked;
    MovementUtil.useRotation = useRotation;
    configured = true;
  }

  public static void setLocked(Translation2d targetLock) {
    MovementUtil.locked = true;
    MovementUtil.useRotation = false;
    MovementUtil.targetLock = targetLock;
  }

  public static void setLocked(Rotation2d desiredRotation) {
    MovementUtil.locked = true;
    MovementUtil.useRotation = true;
    MovementUtil.desiredRotation = desiredRotation;
  }

  public static void setLocked(boolean locked) {
    MovementUtil.locked = locked;
  }

  public static void setToDesired(Translation2d desiredPose) {
    MovementUtil.toDesired = true;
    MovementUtil.desiredPose = desiredPose;
  }

  public static void setToDesired(boolean toDesired) {
    MovementUtil.toDesired = toDesired;
  }

  public static Rotation2d getDesiredRotation(Pose2d robotPose) {
    if (useRotation) {
      return desiredRotation;
    } else {
      Translation2d currTranslation = robotPose.getTranslation();
      Translation2d robotToPose = currTranslation.minus(targetLock);
      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        robotToPose = new Translation2d().minus(robotToPose);
      }
      double x = robotToPose.getX();
      double y = robotToPose.getY();
      return Rotation2d.fromRadians(Math.atan(y / x));
    }
  }

  public static Translation2d getPoseOffset(Pose2d robotPose) {
    Translation2d currTranslation = robotPose.getTranslation();
    double currX = currTranslation.getX();
    double currY = currTranslation.getY();
    double targetX = desiredPose.getX();
    double targetY = desiredPose.getY();
    double xDiff, yDiff;
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      xDiff = currX - targetX;
      yDiff = targetY - currY;
    } else {
      xDiff = targetX - currX;
      yDiff = currY - targetY;
    }
    return new Translation2d(xDiff, yDiff);
  }

  public static ChassisSpeeds getRobotRelative(Pose2d robotPose) {
    Rotation2d desiredRotation = getDesiredRotation(robotPose);
    Translation2d translationDiff = getPoseOffset(robotPose);
    OI.AxesInputs driveInputs = oi.getLeftStickInputs();
    OI.AxesInputs turnInputs = oi.getRightStickInputs();
    double rotationSpeed;
    if (locked) {
      rotationPID.setSetpoint(desiredRotation.getDegrees());
      rotationSpeed = rotationPID.calculate(desiredRotation.getDegrees());
    } else {
      rotationSpeed = -turnInputs.x;
    }
    double xSpeed, ySpeed;
    if (toDesired) {
      movementXPID.setSetpoint(0);
      movementYPID.setSetpoint(0);
      xSpeed = movementXPID.calculate(translationDiff.getX());
      ySpeed = movementYPID.calculate(translationDiff.getY());
    } else {
      xSpeed = driveInputs.x;
      ySpeed = driveInputs.y;
    }
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        ySpeed, xSpeed, rotationSpeed, robotPose.getRotation());
  }

  public static void reset() {
    locked = false;
    toDesired = false;
  }

  public static boolean atTranslation(Pose2d robotPose) {
    Translation2d currDiff = getPoseOffset(robotPose);
    double x = Math.abs(currDiff.getX());
    double y = Math.abs(currDiff.getY());
    return x < 0.02 && y < 0.02;
  }

  public static boolean atRotation(Pose2d robotPose) {
    double rotationDiffDegrees =
        Math.abs(desiredRotation.minus(robotPose.getRotation()).getDegrees());
    return rotationDiffDegrees < 5;
  }

  public static boolean atPose(Pose2d robotPose) {
    if (toDesired && locked) {
      return atTranslation(robotPose) && atRotation(robotPose);
    } else {
      return false;
    }
  }
}
