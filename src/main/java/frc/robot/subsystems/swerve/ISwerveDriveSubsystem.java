package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ISwerveDriveSubsystem extends Subsystem {
    Rotation2d getRotation2d();
    void setFieldCentricMovement(ChassisSpeeds chassisSpeeds);
    void setRawMovement(ChassisSpeeds chassisSpeeds);
    void stop();

    void reset();
}