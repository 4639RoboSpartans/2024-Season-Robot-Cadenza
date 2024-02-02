package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class IntakeSubsystem extends SubsystemBase implements IIntakeSubsystem {
    private final TalonFX pivotMotorLeft;
    private final TalonFX pivotMotorRight;
    private final TalonFX intakeMotor;

    private final CANcoder encoder;

    private final PIDController pivotPID;

    public IntakeSubsystem(int pivotMotorLeftID, int pivotMotorRightID, int intakeMotorID, int encoderID) {
        pivotMotorLeft = new TalonFX(pivotMotorLeftID);
        pivotMotorRight = new TalonFX(pivotMotorRightID);
        intakeMotor = new TalonFX(intakeMotorID);

        encoder = new CANcoder(encoderID);

        pivotPID = Constants.RobotInfo.INTAKE_PIVOT_PID.create();

        pivotMotorLeft.setNeutralMode(NeutralModeValue.Brake);
        pivotMotorRight.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor.setNeutralMode(NeutralModeValue.Coast);

        pivotMotorRight.setInverted(true);
    }

    //Pivots the intake to "retract" and "extend" intake
    public void setIntakeAngle(double degrees) {
        pivotPID.setSetpoint(degrees);
    }

    //Spins intake motor to intake notes
    public void intake() {
        intakeMotor.set(Constants.RobotInfo.INTAKE_SPEED);
    }

    @Override
    public void periodic() {
        double pidOutput = pivotPID.calculate(encoder.getPosition().getValue());

        pivotMotorLeft.set(pidOutput);
        pivotMotorRight.set(pidOutput);
    }

    public void stop() {
        pivotPID.setSetpoint(encoder.getPosition().getValue());

        pivotMotorLeft.stopMotor();
        pivotMotorRight.stopMotor();
        intakeMotor.stopMotor();
    }
}
