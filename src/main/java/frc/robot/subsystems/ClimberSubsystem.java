package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class ClimberSubsystem implements IClimberSubsystem{
    private final TalonFX leftMotor, rightMotor;
    private final int leftUp, rightUp;

    public ClimberSubsystem(){
        leftMotor = new TalonFX(Constants.LEFT_CLIMBER_MOTOR);
        rightMotor = new TalonFX(Constants.RIGHT_CLIMBER_MOTOR);
        if (Constants.LEFT_UP){
            leftUp = 1;
        }
        else{
            leftUp = -1;
        }
        if (Constants.RIGHT_UP){
            rightUp = 1;
        }
        else{
            rightUp = -1;
        }
    }

    public void stop(){
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public void moveLeftUp(double speed){
        double motorSpeed = leftUp * speed;
        leftMotor.set(motorSpeed);
    }

    public void moveRightUp(double speed){
        double motorSpeed = rightUp * speed;
        rightMotor.set(motorSpeed);
    }

    public void moveLeftDown(double speed){
        double motorSpeed = -leftUp * speed;
        leftMotor.set(motorSpeed);
    }

    public void moveRightDown(double speed){
        double motorSpeed = -rightUp * speed;
        rightMotor.set(motorSpeed);
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
