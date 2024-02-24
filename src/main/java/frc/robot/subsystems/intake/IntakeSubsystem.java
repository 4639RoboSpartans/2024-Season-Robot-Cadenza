package frc.robot.subsystems.intake;

import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.RobotInfo.*;

@SuppressWarnings("unused")
public class IntakeSubsystem extends SubsystemBase implements IIntakeSubsystem {
    private final CANSparkMax pivotMotorLeft;
    private final CANSparkMax pivotMotorRight;
    private final CANSparkMax intakeMotor;

    private final PIDController pivotPID;
    private final DutyCycleEncoder encoder;

    public IntakeSubsystem(int pivotMotorLeftID, int pivotMotorRightID, int intakeMotorID, int encoderID) {
        pivotMotorLeft = new CANSparkMax(pivotMotorLeftID, CANSparkMax.MotorType.kBrushless);
        pivotMotorRight = new CANSparkMax(pivotMotorRightID, CANSparkMax.MotorType.kBrushless);
        intakeMotor = new CANSparkMax(intakeMotorID, CANSparkMax.MotorType.kBrushless);

        pivotPID = IntakeInfo.INTAKE_PIVOT_PID.create();

        pivotMotorLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
        pivotMotorRight.setIdleMode(CANSparkBase.IdleMode.kBrake);

        pivotMotorLeft.setInverted(false);
        pivotMotorRight.follow(pivotMotorLeft, true);

        encoder = new DutyCycleEncoder(Constants.IDs.INTAKE_ENCODER_CHANNEL);
        setExtended(false);
    }

    public void setExtended(boolean extended) {
        if(extended) {
            pivotPID.setSetpoint(IntakeInfo.INTAKE_PIVOT_EXTENDED_SETPOINT);
        }
        else {
            pivotPID.setSetpoint(IntakeInfo.INTAKE_PIVOT_DEFAULT_SETPOINT);
        }
    }

    //Spins intake motor to intake notes
    public void intake() {
        intakeMotor.set(IntakeInfo.INTAKE_SPEED);
    }

    public void outtake() {
        intakeMotor.set(-IntakeInfo.INTAKE_SPEED);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    @Override
    public void periodic() {
        double pidOutput = -pivotPID.calculate(encoder.getAbsolutePosition());

        if(pidOutput > 0) pidOutput *= Constants.INTAKE_PIVOT_UP_MULTIPLIER;

        SmartDashboard.putNumber("target pivot pos", pivotPID.getSetpoint());
        SmartDashboard.putNumber("current pivot pos", getPosition());
        SmartDashboard.putNumber("pivot pid output", pidOutput);

        pivotMotorLeft.set(pidOutput);
    }

    public void stop() {
        pivotPID.setSetpoint(getPosition());

        pivotMotorLeft.stopMotor();
        pivotMotorRight.stopMotor();
        intakeMotor.stopMotor();
    }

    private double getPosition() {
        return encoder.getAbsolutePosition();
    }
}
