package frc.robot.subsystems.tramp;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class RollerTrampSubsystem implements ITrampSubsystem{
    private final TalonFX leftRotator, rightRotator, rollerMotor;

    public RollerTrampSubsystem(){
        leftRotator = new TalonFX(Constants.IDs.LEFT_ROTATOR_MOTOR);
        rightRotator = new TalonFX(Constants.IDs.RIGHT_ROTATOR_MOTOR);
        rollerMotor = new TalonFX(Constants.IDs.ROLLER_MOTOR);

        leftRotator.setNeutralMode(NeutralModeValue.Brake);
        rightRotator.setNeutralMode(NeutralModeValue.Brake);
        rollerMotor.setNeutralMode(NeutralModeValue.Brake);
        
        leftRotator.setInverted(true);
    }

    public void rotateArm(){
        leftRotator.set(Constants.RobotInfo.TRAMP_ROTATOR_SPEED);
        rightRotator.set(Constants.RobotInfo.TRAMP_ROTATOR_SPEED);
    }

    public void release(){
        rollerMotor.set(Constants.RobotInfo.TRAMP_ROLLER_RELEASE_SPEED);
    }
    
}
