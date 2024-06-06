package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DummySwerveDriveSubsystem implements ISwerveDriveSubsystem {

  @Override
  public void setMovement(ChassisSpeeds chassisSpeeds) {}

  @Override
  public void stop() {}

  @Override
  public void periodic() {}

  public void resetPose(Pose2d pose) {}

  public Pose2d getPose() {
    return null;
  }

  @Override
  public void resetDesiredRotation() {}

  @Override
  public void setDesiredRotation(Rotation2d desiredRotation) {}

  @Override
  public double getRawRotationSpeed() {
    return 0;
  }

  @Override
  public void toggleFieldRelative() {}

  @Override
  public Rotation2d getRotation2d() {
    return null;
  }
}
