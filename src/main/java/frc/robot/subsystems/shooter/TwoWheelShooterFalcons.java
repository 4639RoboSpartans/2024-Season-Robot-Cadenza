package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TwoWheelShooterFalcons extends SubsystemBase implements IShooterSubsystem {
    private final TalonFX motor1, motor2;

    public TwoWheelShooterFalcons(int motor1ID, int motor2ID) {
        motor1 = new TalonFX(motor1ID);
        motor2 = new TalonFX(motor2ID);
        motor1.setInverted(true);
        motor2.setInverted(false);
        motor1.setNeutralMode(NeutralModeValue.Coast);
        motor2.setNeutralMode(NeutralModeValue.Coast);
    }

    public void shoot(double speed) {
        motor1.set(speed);
        motor2.set(speed);
    }
    public void stop() {
        motor1.stopMotor();
        motor2.stopMotor();
    }
}