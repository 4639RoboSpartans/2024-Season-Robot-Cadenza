package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;
import frc.robot.util.AimUtil;

class _AimCommand extends Command {
    private final ISwerveDriveSubsystem swerveDriveSubsystem;

    public _AimCommand(ISwerveDriveSubsystem swerveDriveSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        addRequirements(swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        swerveDriveSubsystem.stop();
    }

    @Override
    public void execute() {
        Rotation2d rotateSpeed = AimUtil.getAmpRotation(0, 0);

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, rotateSpeed.getRadians() / Math.PI);
        swerveDriveSubsystem.setFieldCentricMovement(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDriveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
