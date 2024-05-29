package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.aim.AimSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

class _AimCommand extends Command {
  private final SwerveDriveSubsystem swerveDriveSubsystem;
  private final AimSubsystem aimSubsystem;

  public _AimCommand(SwerveDriveSubsystem swerveDriveSubsystem, AimSubsystem aimSubsystem) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.aimSubsystem = aimSubsystem;
    addRequirements(swerveDriveSubsystem, aimSubsystem);
  }

  @Override
  public void initialize() {
    swerveDriveSubsystem.driveStopMotion();
  }

  @Override
  public void execute() {
    double rotateSpeed = -aimSubsystem.getSwerveRotation();

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, rotateSpeed);
    swerveDriveSubsystem.driveFieldCentric(chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    swerveDriveSubsystem.driveStopMotion();
  }

  @Override
  public boolean isFinished() {
    return aimSubsystem.isAtSetpoint();
  }
}
