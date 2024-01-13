package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class ClimberSubsystem implements IClimberSubsystem {
    private final TalonFX leftMotor, rightMotor;

    public ClimberSubsystem(int leftMotorID, int rightMotorID){
        leftMotor = new TalonFX(leftMotorID);
        rightMotor = new TalonFX(rightMotorID);

        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setLeftSpeed(double speed) {
        leftMotor.set(Constants.RobotInfo.CLIMBER_LEFT_SPEED * speed);
    }
    public void setRightSpeed(double speed) {
        rightMotor.set(Constants.RobotInfo.CLIMBER_RIGHT_SPEED * speed);
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
