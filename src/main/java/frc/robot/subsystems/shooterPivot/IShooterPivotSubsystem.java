package frc.robot.subsystems.shooterPivot;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.RobotInfo.ShooterInfo.SHOOTING_MODE;

public interface IShooterPivotSubsystem extends Subsystem {
    void setAngleDegrees(double degrees);

    void stop();

    void manualSet(double degrees);

    boolean isAtSetPoint();

    void setShooting(SHOOTING_MODE shooting);

    void setManual(boolean manual);
}
