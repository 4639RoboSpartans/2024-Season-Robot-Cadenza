package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class ClimberSubsystem implements IClimberSubsystem{
    private final TalonFX leftMotor, rightMotor;
    private final boolean leftInvert, rightInvert;

    public ClimberSubsystem(){
        leftMotor = new TalonFX(Constants.LEFT_CLIMBER_MOTOR);
        rightMotor = new TalonFX(Constants.RIGHT_CLIMBER_MOTOR);

        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);

        leftInvert = Constants.LEFT_CLIMBER_DIRECTION == Constants.MotorDirection.Normal;
        rightInvert = Constants.RIGHT_CLIMBER_DIRECTION == Constants.MotorDirection.Normal;

        leftMotor.setInverted(leftInvert);
        rightMotor.setInverted(rightInvert);
    }

    public void stop(){
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public void moveLeftUp(double speed){
        leftMotor.set(speed);
    }

    public void moveRightUp(double speed){
        rightMotor.set(speed);
    }

    public void moveLeftDown(double speed){
        leftMotor.set(-speed);
    }

    public void moveRightDown(double speed){
        rightMotor.set(-speed);
    }

    public void moveUp(double speed){
        moveRightUp(speed);
        moveLeftUp(speed);
    }

    public void moveDown(double speed){
        moveRightDown(speed);
        moveLeftDown(speed);
    }
}
