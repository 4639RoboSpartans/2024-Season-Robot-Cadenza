package frc.robot.commands.semiauto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.network.LimeLight;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import java.util.ArrayDeque;

record RobotPose(double xOffset, double zOffset, double yRotation) {}

public class CenterLimelight extends Command {
    private final SwerveDriveSubsystem swerveDrive;
    private final ArrayDeque<RobotPose> prevPoses = new ArrayDeque<>();
    
    public CenterLimelight(SwerveDriveSubsystem swerveDriveSubsystem) {
        this.swerveDrive = swerveDriveSubsystem;

        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        swerveDrive.stop();
        prevPoses.clear();
    }

    @Override
    public void execute() {
        acceptInput();

        if(prevPoses.isEmpty()) return;

        double xOffset = prevPoses.stream().mapToDouble(RobotPose::xOffset).sum() / prevPoses.size();
        double zOffset = prevPoses.stream().mapToDouble(RobotPose::zOffset).sum() / prevPoses.size();
        double yRotation = prevPoses.stream().mapToDouble(RobotPose::yRotation).sum() / prevPoses.size();

        double xSpeed = -xOffset * Constants.RobotInfo.MAX_ROBOT_SPEED;
        double zSpeed = -zOffset * Constants.RobotInfo.MAX_ROBOT_SPEED * 0.3;
        double yRotationSpeed = -yRotation * Constants.RobotInfo.MAX_ROBOT_SPEED * 0.02;

        swerveDrive.setRawMovement(new ChassisSpeeds(xSpeed, zSpeed, yRotationSpeed));
    }

    private void acceptInput() {
        double xOffset = LimeLight.getXDistance();
        double zOffset = LimeLight.getZDistance() - Constants.FieldDistances.ShooterApriltagZDistance;
        double yRotation = LimeLight.getYRotation();

        if(xOffset == 0) return;

        if(prevPoses.size() >= Constants.CENTER_LIMELIGHT_AVERAGING_WINDOW_LENGTH) {
            prevPoses.removeFirst();
        }
        prevPoses.addLast(new RobotPose(xOffset, zOffset, yRotation));
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
    }
}
