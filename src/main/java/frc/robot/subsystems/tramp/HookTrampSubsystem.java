package frc.robot.subsystems.tramp;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class HookTrampSubsystem implements ITrampSubsystem{
    private final TalonFX leftRotator, rightRotator, hookMotor;

    public HookTrampSubsystem(){
        leftRotator = new TalonFX(Constants.IDs.LEFT_ROTATOR_MOTOR);
        rightRotator = new TalonFX(Constants.IDs.RIGHT_ROTATOR_MOTOR);
        hookMotor = new TalonFX(Constants.IDs.HOOK_MOTOR);

        leftRotator.setNeutralMode(NeutralModeValue.Brake);
        rightRotator.setNeutralMode(NeutralModeValue.Brake);
        hookMotor.setNeutralMode(NeutralModeValue.Brake);
        
        leftRotator.setInverted(true);
    }

    public void rotateArm(){
        leftRotator.set(Constants.RobotInfo.TRAMP_ROTATOR_SPEED);
        rightRotator.set(Constants.RobotInfo.TRAMP_ROTATOR_SPEED);
    }

    public void release(){
        hookMotor.set(Constants.RobotInfo.TRAMP_HOOK_RELEASE_SPEED);
    }
    
}
