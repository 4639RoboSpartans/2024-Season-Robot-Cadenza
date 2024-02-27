package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.NavX;

@SuppressWarnings("unused")
public interface ISwerveDriveSubsystem extends Subsystem {
    Rotation2d getRotation2d();
    void setMovement(ChassisSpeeds chassisSpeeds);
    void setRawMovement(ChassisSpeeds chassisSpeeds);
    void stop();
    void periodic();
    void setBrakeMode();
    void setCoastMode();
}
