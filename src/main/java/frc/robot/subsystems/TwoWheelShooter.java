package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TwoWheelShooter extends SubsystemBase implements IShooterSubsystem {
    private final TalonFX motor1, motor2;

    public TwoWheelShooter(int motor1ID, int motor2ID) {
        motor1 = new TalonFX(motor1ID);
        motor2 = new TalonFX(motor2ID);
        motor1.setInverted(true);
        motor2.setInverted(false);
    }

    public void shoot(double speed) {
        motor1.set(speed);
        motor2.set(-speed);
    }
    public void stop() {
        motor1.stopMotor();
        motor2.stopMotor();
    }
}