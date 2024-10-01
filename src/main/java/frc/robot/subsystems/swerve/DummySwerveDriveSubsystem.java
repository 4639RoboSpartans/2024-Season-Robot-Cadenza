package frc.robot.subsystems.swerve;

import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;

public class DummySwerveDriveSubsystem extends SwerveDriveSubsystem {
    public DummySwerveDriveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
    }

    @Override
    public Rotation2d getRotation2d() {
        return null;
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
    public Command pathfindCommand(Pose2d targetPose) {
        return null;
    }

    @Override
    public Command driveFieldCentricCommand() {
        return null;
    }

    @Override
    public Command SOTFCommand() {
        return null;
    }

    @Override
    public Command followChoreoPath(ChoreoTrajectory trajectory, boolean resetPosition) {
        return null;
    }

    @Override
    public Command followChoreoPath(String pathName, boolean resetPosition) {
        return null;
    }

    @Override
    protected boolean isAlignedSupplier() {
        return false;
    }

    @Override
    protected boolean inShootingRangeSupplier() {
        return false;
    }

    @Override
    protected boolean inShootingSectorSupplier() {
        return false;
    }

    @Override
    protected void buildSendable(SendableBuilder builder) {

    }
}
