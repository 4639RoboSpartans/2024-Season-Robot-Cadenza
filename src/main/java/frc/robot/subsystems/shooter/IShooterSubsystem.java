package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IShooterSubsystem extends Subsystem {
    boolean isUpToSpeed();

    void runShooter();

    void stopShooter();
}