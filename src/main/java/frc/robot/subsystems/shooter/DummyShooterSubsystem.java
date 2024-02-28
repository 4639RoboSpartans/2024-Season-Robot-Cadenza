package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotInfo.ShooterInfo.SHOOTING_MODE;
import frc.robot.subsystems.shooter.pivot.DummyShooterPivotSubsystem;
import frc.robot.subsystems.shooter.pivot.IShooterPivotSubsystem;

@SuppressWarnings("unused")
public class DummyShooterSubsystem extends SubsystemBase implements IShooterSubsystem {
    @Override
    public boolean isUpToSpeed() {
        return false;
    }

    @Override
    public void runShooter() {
    }

    @Override
    public void stopShooter() {
    }

    @Override
    public void setShooting(SHOOTING_MODE shooting) {
        
    }

    @Override
    public IShooterPivotSubsystem getPivot() {
        return new DummyShooterPivotSubsystem();
    }
}
