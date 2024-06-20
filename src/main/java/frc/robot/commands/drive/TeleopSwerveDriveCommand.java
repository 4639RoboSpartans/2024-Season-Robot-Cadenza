package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Controls.DriverControls;
import frc.robot.constants.FieldConstants;
import frc.robot.oi.OI;
import frc.robot.subsystems.aim.AimSubsystem;
import frc.robot.subsystems.swerve.MovementUtil;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem.DriveState;

public class TeleopSwerveDriveCommand extends Command {
  private final SwerveDriveSubsystem swerveDriveSubsystem;
  private final OI oi;

  public TeleopSwerveDriveCommand(
      SwerveDriveSubsystem swerveDriveSubsystem, OI oi) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.oi = oi;
    addRequirements(swerveDriveSubsystem);
  }

  @Override
  public void initialize() {
    swerveDriveSubsystem.driveStopMotion();
  }

  @Override
  public void execute() {
    if (oi.driverController().getButton(DriverControls.AimButton).getAsBoolean()) {
      swerveDriveSubsystem.setState(DriveState.SPEAKER_LOCK);
    } else {
      swerveDriveSubsystem.setState(DriveState.MANUAL);
    }
    swerveDriveSubsystem.driveFieldCentric(
        MovementUtil.getRobotRelative(swerveDriveSubsystem.getPoseMeters()));
  }

  @Override
  public void end(boolean interrupted) {
    swerveDriveSubsystem.driveStopMotion();
  }
}
