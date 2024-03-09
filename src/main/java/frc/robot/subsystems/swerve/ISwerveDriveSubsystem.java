package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ISwerveDriveSubsystem extends Subsystem {
    Rotation2d getRotation2d();
    void setMovement(ChassisSpeeds chassisSpeeds);
    void setRawMovement(ChassisSpeeds chassisSpeeds);
    void stop();
    void periodic();
    void setBrakeMode();
    void setCoastMode();
    SwerveModule getSwerveModule(String module);
    void resetOdometry(Pose2d pose);
    void resetPose(Pose2d pose);
    double getHeading();
}