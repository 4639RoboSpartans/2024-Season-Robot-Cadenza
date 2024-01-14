package frc.robot.subsystems.tramp;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class HookTrampSubsystem implements ITrampSubsystem{
    private final TalonFX leftRotator, rightRotator, hookMotor;
    private final PIDController rotatorPID, hookPID;

    public HookTrampSubsystem(){
        leftRotator = new TalonFX(Constants.IDs.LEFT_ROTATOR_MOTOR);
        rightRotator = new TalonFX(Constants.IDs.RIGHT_ROTATOR_MOTOR);
        hookMotor = new TalonFX(Constants.IDs.HOOK_MOTOR);

        leftRotator.setNeutralMode(NeutralModeValue.Brake);
        rightRotator.setNeutralMode(NeutralModeValue.Brake);
        hookMotor.setNeutralMode(NeutralModeValue.Brake);
        
        leftRotator.setInverted(true);
        rotatorPID = Constants.RobotInfo.TRAMP_ROTATOR_PID.create();
        hookPID = Constants.RobotInfo.HOOK_ROTATOR_PID.create();
    }

    public void rotateArm(double degrees){
        leftRotator.set(Constants.RobotInfo.TRAMP_ROTATOR_SPEED);
        rightRotator.set(Constants.RobotInfo.TRAMP_ROTATOR_SPEED);
    }

    public void release(){
        hookMotor.set(Constants.RobotInfo.TRAMP_HOOK_RELEASE_SPEED);
    }

    public void setAngleDegrees(double degrees) {
        rotatorPID.setSetpoint(degrees);
    }

    public void setHookAngleDegrees(double degrees){
        hookPID.setSetpoint(degrees);
    }

    @Override
    public void periodic() {
        double currentLeftMotorDegrees = leftRotator.getPosition().getValue();
        leftRotator.set(rotatorPID.calculate(currentLeftMotorDegrees));
        double currentRightMotorDegrees = leftRotator.getPosition().getValue();
        leftRotator.set(rotatorPID.calculate(currentRightMotorDegrees));
        double currentHookMotorDegrees = leftRotator.getPosition().getValue();
        leftRotator.set(rotatorPID.calculate(currentHookMotorDegrees));
    }

    public void stop(){
        leftRotator.stopMotor();
        rightRotator.stopMotor();
        hookMotor.stopMotor();
    }
    
}
