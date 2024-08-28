package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.DoubleSupplier;

public interface ISwerveDriveSubsystem extends Subsystem {
  Rotation2d getRotation2d();

  void setMovement(ChassisSpeeds chassisSpeeds);

  void stop();

  void periodic();

  void resetPose(Pose2d pose);

  Pose2d getPose();

  void resetDesiredRotation();

  void setDesiredRotation(Rotation2d desiredRotation);

  double getRawRotationSpeed();

  double getAimRotationSpeed(double forwardsSpeed, double sidewaysSpeed);

  double getRawXSpeed();

  double getRawYSpeed();

  void setDesiredPose(Pose2d pose);

  void setDesiredTranslation(Translation2d translation);

  Command ampAimCommand();

  Command speakerAimCommand(DoubleSupplier forwardsSpeeds, DoubleSupplier sidewaysSpeeds);
}
