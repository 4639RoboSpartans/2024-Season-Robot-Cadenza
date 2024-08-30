package frc.robot.util;

import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.intake.IIntakeSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;

public class AutoHelper {
    private final ISwerveDriveSubsystem swerve;
    private final IIntakeSubsystem intake;
    private final IShooterSubsystem shooter;

    public AutoHelper() {
        swerve = SubsystemManager.getSwerveDrive();
        intake = SubsystemManager.getIntake();
        shooter = SubsystemManager.getShooter();
    }
}
