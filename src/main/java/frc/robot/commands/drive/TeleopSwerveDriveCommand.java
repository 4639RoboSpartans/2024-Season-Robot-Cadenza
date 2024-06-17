package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Controls.DriverControls;
import frc.robot.constants.RobotInfo;
import frc.robot.constants.RobotInfo.SwerveInfo;
import frc.robot.oi.OI;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;
import frc.robot.util.AimUtil;

public class TeleopSwerveDriveCommand extends Command {
  private final ISwerveDriveSubsystem swerveDriveSubsystem;
  private final OI oi;
  private boolean wasTurning = false;

  public TeleopSwerveDriveCommand(ISwerveDriveSubsystem swerveDriveSubsystem, OI oi) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.oi = oi;
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
    double rawTurnSpeed =
            oi.driverController().getAxis(DriverControls.SwerveRotationAxis)
            * SwerveInfo.TELOP_ROTATION_SPEED;
    double turnSpeed;
    if (Math.abs(rawTurnSpeed) > 0) {
      wasTurning = true;
      turnSpeed = rawTurnSpeed;
    } else if (wasTurning){
      swerveDriveSubsystem.resetDesiredRotation();
      wasTurning = false;
      turnSpeed = swerveDriveSubsystem.getRawRotationSpeed();
    } else {
      turnSpeed = swerveDriveSubsystem.getRawRotationSpeed();
    }
    swerveDriveSubsystem.setMovement(new ChassisSpeeds(
            forwardsSpeed,
            sidewaysSpeed,
            turnSpeed
    ));
  }

  @Override
  public void end(boolean interrupted) {
    swerveDriveSubsystem.stop();
  }
}
