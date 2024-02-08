package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class IntakeSubsystem extends SubsystemBase implements IIntakeSubsystem {
//    private final TalonFX pivotMotorLeft;
//    private final TalonFX pivotMotorRight;
    private final CANSparkMax intakeMotor;

//    private final CANcoder encoder;

    private final PIDController pivotPID;

    public IntakeSubsystem(int pivotMotorLeftID, int pivotMotorRightID, int intakeMotorID, int encoderID) {
//        pivotMotorLeft = new TalonFX(pivotMotorLeftID);
//        pivotMotorRight = new TalonFX(pivotMotorRightID);
        intakeMotor = new CANSparkMax(intakeMotorID, CANSparkMax.MotorType.kBrushless);

//        encoder = new CANcoder(encoderID);

        pivotPID = Constants.RobotInfo.INTAKE_PIVOT_PID.create();

//        pivotMotorLeft.setNeutralMode(NeutralModeValue.Brake);
//        pivotMotorRight.setNeutralMode(NeutralModeValue.Brake);

//        pivotMotorRight.setInverted(true);
    }

    //Pivots the intake to "retract" and "extend" intake
    public void setIntakeAngle(double degrees) {
        pivotPID.setSetpoint(degrees);
    }

    //Spins intake motor to intake notes
    public void intake() {
        intakeMotor.set(Constants.RobotInfo.INTAKE_SPEED);
    }

    public void outtake() {
        intakeMotor.set(-Constants.RobotInfo.INTAKE_SPEED);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    @Override
    public void periodic() {
//        double pidOutput = pivotPID.calculate(encoder.getPosition().getValue());

//        pivotMotorLeft.set(pidOutput);
//        pivotMotorRight.set(pidOutput);
    }

    public void stop() {
//        pivotPID.setSetpoint(encoder.getPosition().getValue());

//        pivotMotorLeft.stopMotor();
//        pivotMotorRight.stopMotor();
        intakeMotor.stopMotor();
    }
}
