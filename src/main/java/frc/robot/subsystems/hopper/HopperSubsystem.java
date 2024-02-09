package frc.robot.subsystems.hopper;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase implements IHopperSubsystem {
    private final CANSparkMax motor;

    public HopperSubsystem(int motorID) {
        motor = new CANSparkMax(motorID, CANSparkMax.MotorType.kBrushless);
    }

    @Override
    public void run() {
        motor.set(Constants.RobotInfo.HOPPER_SPEED);
    }

    @Override
    public void runBackwards() {
        motor.set(-Constants.RobotInfo.HOPPER_SPEED);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
