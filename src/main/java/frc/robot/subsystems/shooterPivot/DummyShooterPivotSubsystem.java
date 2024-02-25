package frc.robot.subsystems.shooterPivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("unused")
public class DummyShooterPivotSubsystem extends SubsystemBase implements IShooterPivotSubsystem {
    public DummyShooterPivotSubsystem() {}

    @Override
    public void setAngleDegrees(double degrees) {}

    public void stop() {}

    public void manualSet(double degrees) {}

    public boolean isAtSetPoint() { return false; }

    public void setShooting(boolean shooting) {}

    public void setManual(boolean manual) {}
}
