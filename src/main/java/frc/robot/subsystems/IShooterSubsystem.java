package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IShooterSubsystem extends Subsystem {
    void shoot(double speed);
    void stop();
}
