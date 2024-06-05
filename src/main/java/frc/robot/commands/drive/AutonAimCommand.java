package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;

public class AutonAimCommand extends ParallelRaceGroup {
  public AutonAimCommand(ISwerveDriveSubsystem swerveDriveSubsystem, double timeSeconds) {
    super(new WaitCommand(timeSeconds), new _AimCommand(swerveDriveSubsystem));
  }
}
