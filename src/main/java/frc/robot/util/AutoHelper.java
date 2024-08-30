package frc.robot.util;

import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.intake.IIntakeSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;

public class AutoHelper {
    private static ISwerveDriveSubsystem swerve = SubsystemManager.getSwerveDrive();
    private static IShooterSubsystem shooter = SubsystemManager.getShooter();
    private static IIntakeSubsystem intake = SubsystemManager.getIntake();

}
