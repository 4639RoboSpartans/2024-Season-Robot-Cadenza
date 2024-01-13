package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TwoWheelShooterNeos extends SubsystemBase implements IShooterSubsystem {
    private final CANSparkMax motor1, motor2;

    public TwoWheelShooterNeos(int motor1ID, int motor2ID) {
        motor1 = new CANSparkMax(motor1ID, CANSparkLowLevel.MotorType.kBrushless);
        motor2 = new CANSparkMax(motor2ID, CANSparkLowLevel.MotorType.kBrushless);
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