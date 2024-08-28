package frc.robot.subsystems.shooter.pivot;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IShooterPivotSubsystem extends Subsystem {
    void stop();

    boolean isAtSetPoint();

    double getCurrentAngle();
}
