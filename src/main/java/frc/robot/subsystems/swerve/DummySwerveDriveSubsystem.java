package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DummySwerveDriveSubsystem implements ISwerveDriveSubsystem {
    @Override
    public Rotation2d getRotation2d() {
        return new Rotation2d();
    }

    @Override
    public void setMovement(ChassisSpeeds chassisSpeeds) {

    }

    @Override
    public void setRawMovement(ChassisSpeeds chassisSpeeds) {

    }

    @Override
    public void stop() {

    }

    @Override
    public void periodic() {

    }

    @Override
    public void setBrakeMode() {

    }

    @Override
    public void setCoastMode() {

    }

    @Override
    public void resetOdometry(Pose2d pose) {

    }

    @Override
    public void resetPose(Pose2d pose) {

    }

    @Override
    public SwerveModule getSwerveModule(String module){
        return null;
    }
}
