package frc.robot.subsystems.shooter.pivot;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.RobotInfo.AimInfo.LIMELIGHT_STATUS;
import frc.robot.Constants.RobotInfo.ShooterInfo.ShootingMode;

public interface IShooterPivotSubsystem extends Subsystem {
    void stop();

    boolean isAtSetPoint();
}
