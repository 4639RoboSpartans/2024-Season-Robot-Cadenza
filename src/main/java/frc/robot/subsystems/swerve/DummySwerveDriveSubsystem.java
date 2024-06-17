package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

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
  public double getAimRotationSpeed(double forwardsSpeed, double sidewaysSpeed) {
    return 0;
  }

  @Override
  public double getRawXSpeed() {
    return 0;
  }

  @Override
  public double getRawYSpeed() {
    return 0;
  }

  @Override
  public void setDesiredPose(Pose2d pose) {

  }

  @Override
  public void setDesiredTranslation(Translation2d translation) {

  }

  @Override
  public Command ampAimCommand() {
    return null;
  }

  @Override
  public Command speakerAimCommand(DoubleSupplier forwardsSpeeds, DoubleSupplier sidewaysSpeeds) {
    return null;
  }


  @Override
  public Rotation2d getRotation2d() {
    return null;
  }
}
