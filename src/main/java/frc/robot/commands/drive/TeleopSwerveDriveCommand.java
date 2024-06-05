package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Controls.DriverControls;
import frc.robot.constants.RobotInfo.SwerveInfo;
import frc.robot.oi.OI;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;
import frc.robot.util.AimUtil;

public class TeleopSwerveDriveCommand extends Command {
  private final ISwerveDriveSubsystem swerveDriveSubsystem;
  private final OI oi;
  private final PIDController rotationController;
  private boolean wasTurning = false;

  public TeleopSwerveDriveCommand(ISwerveDriveSubsystem swerveDriveSubsystem, OI oi) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.oi = oi;
    rotationController = SwerveInfo.TeleopRotationPID.create();
    addRequirements(swerveDriveSubsystem);
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

  private void updateDesiredRotation(double rawInput) {
    if (oi.driverController().getButton(DriverControls.AimButton).getAsBoolean()) {
      if (wasTurning) wasTurning = false;
      swerveDriveSubsystem.setDesiredRotation(AimUtil.getRotation());
    } else {
      if (rawInput != 0) {
        wasTurning = true;
      } else {
        if (wasTurning) {
          wasTurning = false;
          swerveDriveSubsystem.setDesiredRotation(swerveDriveSubsystem.getRotation2d());
        }
      }
    }
  }

  private double getRotationSpeed(double rotationMultiplier) {
    double rawInput = -oi.driverController().getAxis(DriverControls.SwerveRotationAxis);
    updateDesiredRotation(rawInput);
    if (rawInput != 0) {
      return rawInput * (1 + rotationMultiplier) * SwerveInfo.CURRENT_MAX_ROBOT_MPS;
    } else {
      return swerveDriveSubsystem.getRawRotationSpeed() * (1 + rotationMultiplier);
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerveDriveSubsystem.stop();
  }
}
