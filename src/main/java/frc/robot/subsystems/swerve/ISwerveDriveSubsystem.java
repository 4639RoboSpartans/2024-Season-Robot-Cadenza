package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ISwerveDriveSubsystem extends Subsystem {
  Rotation2d getRotation2d();

  void setMovement(ChassisSpeeds chassisSpeeds);

  void stop();

  void periodic();

  void resetPose(Pose2d pose);

  Pose2d getPose();

  void resetDesiredRotation();

  void setDesiredRotation(Rotation2d desiredRotation);

  Rotation2d getDesiredRotation();

  double getRawRotationSpeed();

    void toggleFieldRelative();
}
