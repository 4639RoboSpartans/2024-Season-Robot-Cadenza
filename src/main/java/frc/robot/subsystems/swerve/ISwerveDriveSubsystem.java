package frc.robot.subsystems.swerve;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ISwerveDriveSubsystem extends Subsystem, Sendable {
    enum DriveState {
        TELEOP,
        TRACK_TARGET,
        SOTF,
        PATHFIND
    }

    Rotation2d getRotation2d();
    void stop();

    void reset();

    Pose2d getPose();

    Command pathfindCommand(Pose2d targetPose);

    Command driveFieldCentricCommand();

    Command SOTFCommand();

    Trigger inLaunchRange();

    Command followChoreoPath(ChoreoTrajectory trajectory, boolean resetPosition);


    Command followChoreoPath(String pathName, boolean resetPosition);

    Trigger isAligned();

    Trigger inShootingRange();

    Trigger inShootingSector();

    Trigger inSpinupRange();
}