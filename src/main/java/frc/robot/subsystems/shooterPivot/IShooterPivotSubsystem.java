package frc.robot.subsystems.shooterPivot;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IShooterPivotSubsystem extends Subsystem {
    void setAngleDegrees(double degrees);

    void stop();

    void manualSet(double degrees);

    boolean isAtSetPoint();

    void setShooting(boolean shooting);

    void setManual(boolean manual);
}
