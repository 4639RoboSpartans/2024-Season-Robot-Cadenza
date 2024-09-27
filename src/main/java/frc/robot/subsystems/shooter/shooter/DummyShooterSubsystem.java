package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotInfo.ShooterInfo.ShootingMode;

public class DummyShooterSubsystem extends SubsystemBase implements IShooterSubsystem {
    @Override
    public boolean isReady() {
        return true;
    }

    @Override
    public void setShootingMode(ShootingMode shooting) {
        
    }

    @Override
    public ShootingMode getShootingMode() {
        return ShootingMode.IDLE;
    }
}
