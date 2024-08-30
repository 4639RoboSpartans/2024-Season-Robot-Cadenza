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
    public void setFieldCentricMovement(ChassisSpeeds chassisSpeeds) {

    }

    @Override
    public void setRawMovement(ChassisSpeeds chassisSpeeds) {

    }

    @Override
    public void stop() {

    }

    @Override
    public void reset() {

    }

    @Override
    public Pose2d getPose() {
        return null;
    }

    @Override
    public boolean aligned() {
        return false;
    }
}
