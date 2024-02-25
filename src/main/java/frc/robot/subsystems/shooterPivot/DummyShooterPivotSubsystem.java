package frc.robot.subsystems.shooterPivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotInfo.AimInfo.LIMELIGHT_STATUS;
import frc.robot.Constants.RobotInfo.ShooterInfo.SHOOTING_MODE;

@SuppressWarnings("unused")
public class DummyShooterPivotSubsystem extends SubsystemBase implements IShooterPivotSubsystem {
    public DummyShooterPivotSubsystem() {}

    @Override
    public void setAngleDegrees(double degrees) {}

    public void stop() {}

    public void manualSet(double degrees) {}

    public boolean isAtSetPoint() { return false; }

    public void setShooting(SHOOTING_MODE shooting) {}

    public void setManual(LIMELIGHT_STATUS manual) {}
}
