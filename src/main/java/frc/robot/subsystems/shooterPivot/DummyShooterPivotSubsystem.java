package frc.robot.subsystems.shooterPivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("unused")
public class DummyShooterPivotSubsystem extends SubsystemBase implements IShooterPivotSubsystem {
    public DummyShooterPivotSubsystem() {}

    @Override
    public void setAngleDegrees(double degrees) {}

    public void stop() {}
}
