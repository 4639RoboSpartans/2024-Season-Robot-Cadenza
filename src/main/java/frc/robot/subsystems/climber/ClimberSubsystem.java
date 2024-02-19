package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.RobotInfo.*;

@SuppressWarnings("unused")
public class ClimberSubsystem extends SubsystemBase implements IClimberSubsystem {
    private final CANSparkMax leftMotor, rightMotor;

    public ClimberSubsystem(int leftMotorID, int rightMotorID) {
        leftMotor = new CANSparkMax(leftMotorID, CANSparkMax.MotorType.kBrushless);
        rightMotor = new CANSparkMax(rightMotorID, CANSparkMax.MotorType.kBrushless);

        leftMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        rightMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        rightMotor.setInverted(true);
    }

    public void setLeftSpeed(double speed) {
        leftMotor.set(speed * ClimberInfo.CLIMBER_SPEED);
    }

    public void setRightSpeed(double speed) {
        rightMotor.set(speed * ClimberInfo.CLIMBER_SPEED);
    }

    @Override
    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }
}
