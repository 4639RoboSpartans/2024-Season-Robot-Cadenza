package frc.robot.subsystems.shooter.pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DummyShooterPivotSubsystem extends SubsystemBase implements IShooterPivotSubsystem {
    public DummyShooterPivotSubsystem() {}

    public void stop() {}

    public boolean isAtSetPoint() { return false; }

}
