package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class ClimberSubsystem extends SubsystemBase implements IClimberSubsystem {
    private final TalonFX leftMotor, rightMotor;

    public ClimberSubsystem(int leftMotorID, int rightMotorID) {
        leftMotor = new TalonFX(leftMotorID);
        rightMotor = new TalonFX(rightMotorID);

        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);

        leftMotor.setInverted(true);
    }

    public void setLeftSpeed(double speed) {
        leftMotor.set(Constants.RobotInfo.CLIMBER_SPEED);
    }

    public void setRightSpeed(double speed) {
        rightMotor.set(Constants.RobotInfo.CLIMBER_SPEED);
    }

    public void setSpeed(double speed) {
        setLeftSpeed(speed);
        setRightSpeed(speed);
    }

    @Override
    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }
}
