package frc.robot.subsystems.tramp;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class RollerTrampSubsystem implements ITrampSubsystem {
    private final TalonFX leftRotator, rightRotator, rollerMotor;
    private final PIDController rotatorPID;

    public RollerTrampSubsystem() {
        leftRotator = new TalonFX(Constants.IDs.LEFT_ROTATOR_MOTOR);
        rightRotator = new TalonFX(Constants.IDs.RIGHT_ROTATOR_MOTOR);
        rollerMotor = new TalonFX(Constants.IDs.ROLLER_MOTOR);

        leftRotator.setNeutralMode(NeutralModeValue.Brake);
        rightRotator.setNeutralMode(NeutralModeValue.Brake);
        rollerMotor.setNeutralMode(NeutralModeValue.Brake);

        leftRotator.setInverted(true);
        rotatorPID = Constants.RobotInfo.TRAMP_ROTATOR_PID.create();
    }

    public void rotateArm(double degrees) {
        leftRotator.set(Constants.RobotInfo.TRAMP_ROTATOR_SPEED);
        rightRotator.set(Constants.RobotInfo.TRAMP_ROTATOR_SPEED);
    }

    public void release() {
        rollerMotor.set(Constants.RobotInfo.TRAMP_ROLLER_RELEASE_SPEED);
    }

    public void setAngleDegrees(double degrees) {
        rotatorPID.setSetpoint(degrees);
    }

    @Override
    public void periodic() {
        double currentLeftMotorDegrees = leftRotator.getPosition().getValue();
        leftRotator.set(rotatorPID.calculate(currentLeftMotorDegrees));
        double currentRightMotorDegrees = leftRotator.getPosition().getValue();
        leftRotator.set(rotatorPID.calculate(currentRightMotorDegrees));
    }

    public void stop() {
        leftRotator.stopMotor();
        rightRotator.stopMotor();
        rollerMotor.stopMotor();
    }

    public void setHookAngleDegrees(double degrees) {

    }
}
