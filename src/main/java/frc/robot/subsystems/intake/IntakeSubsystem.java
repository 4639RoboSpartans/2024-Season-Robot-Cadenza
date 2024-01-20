package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class IntakeSubsystem extends SubsystemBase implements IIntakeSubsystem {
    private final TalonFX intakeMotor;
    private final TalonFX pivotMotor;

    private final PIDController pivotPID;

    public IntakeSubsystem(int intakeMotorID, int pivotID) {
        intakeMotor = new TalonFX(intakeMotorID);
        pivotMotor = new TalonFX(pivotID);

        pivotPID = Constants.RobotInfo.SHOOTER_AIM_PID.create();

        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        pivotMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    //Pivots the intake to "retract" and "extend" intake
    public void setIntakeDegree(double degrees) {
        pivotPID.setSetpoint(degrees);
    }

    //Spins intake motor to intake notes
    public void intake() {
        intakeMotor.set(Constants.RobotInfo.INTAKE_SPEED);
    }

    public void stop() {
        intakeMotor.stopMotor();
        pivotMotor.stopMotor();
    }
}
