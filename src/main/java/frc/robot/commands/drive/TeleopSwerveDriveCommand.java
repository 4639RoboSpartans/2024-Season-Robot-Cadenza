package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Controls.DriverControls;
import frc.robot.constants.RobotInfo.SwerveInfo;
import frc.robot.oi.OI;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.aim.AimSubsystem;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;

public class TeleopSwerveDriveCommand extends Command {
  private final ISwerveDriveSubsystem swerveDriveSubsystem;
  private final AimSubsystem aimSubsystem;
  private final OI oi;

  public TeleopSwerveDriveCommand(
      ISwerveDriveSubsystem swerveDriveSubsystem, AimSubsystem aimSubsystem, OI oi) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.aimSubsystem = aimSubsystem;
    this.oi = oi;
    addRequirements(swerveDriveSubsystem, aimSubsystem);
  }

  @Override
  public void initialize() {
    swerveDriveSubsystem.stop();
  }

  @Override
  public void execute() {
    double forwardsSpeed =
        oi.driverController().getAxis(DriverControls.SwerveForwardAxis)
            * SwerveInfo.CURRENT_MAX_ROBOT_MPS;
    double sidewaysSpeed =
        -oi.driverController().getAxis(DriverControls.SwerveStrafeAxis)
            * SwerveInfo.CURRENT_MAX_ROBOT_MPS;
    double rotationMultiplier = Math.hypot(forwardsSpeed, sidewaysSpeed) / 2;
    double rotateSpeed = getRotationSpeed(rotationMultiplier);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forwardsSpeed, sidewaysSpeed, rotateSpeed);
    swerveDriveSubsystem.setMovement(chassisSpeeds);
    SmartDashboard.putNumber("navX heading", SubsystemManager.getNavX().getHeading());
  }

  private double getRotationSpeed(double rotationMultiplier) {
    double rawSpeed;
    if (oi.driverController().getButton(DriverControls.AimButton).getAsBoolean()) {

      rawSpeed =
          (-aimSubsystem.getSwerveRotation()
                  - oi.driverController().getAxis(DriverControls.SwerveRotationAxis)
                      * SwerveInfo.TELOP_ROTATION_SPEED)
              * (1 + rotationMultiplier * 15);
    } else {
      rawSpeed =
          -oi.driverController().getAxis(DriverControls.SwerveRotationAxis)
              * SwerveInfo.TELOP_ROTATION_SPEED;
    }
    return rawSpeed * (1 + rotationMultiplier);
  }

  @Override
  public void end(boolean interrupted) {
    swerveDriveSubsystem.stop();
  }
}
