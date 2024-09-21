package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;
import frc.robot.oi.OI;
import frc.robot.subsystems.SubsystemManager;

import static frc.robot.constants.RobotInfo.IntakeInfo;

public class IntakeSubsystem extends SubsystemBase implements IIntakeSubsystem {
    private final CANSparkMax pivotMotorLeft;
    private final CANSparkMax pivotMotorRight;
    private final CANSparkMax intakeMotor;
    private final DutyCycleEncoder encoder;
    private ExtensionState state;

    private final ProfiledPIDController pivotPID;
//    private final PIDController pivotPID;
    public IntakeSubsystem() {
        pivotMotorLeft = new CANSparkMax(IDs.INTAKE_PIVOT_MOTOR_LEFT,
                CANSparkMax.MotorType.kBrushless);
        pivotMotorRight = new CANSparkMax(IDs.INTAKE_PIVOT_MOTOR_RIGHT,
                CANSparkMax.MotorType.kBrushless);
        intakeMotor = new CANSparkMax(IDs.INTAKE_MOTOR,
                CANSparkMax.MotorType.kBrushless);

        pivotPID = new ProfiledPIDController(
                IntakeInfo.INTAKE_PIVOT_PID_CONSTANTS.kp(),
                IntakeInfo.INTAKE_PIVOT_PID_CONSTANTS.ki(),
                IntakeInfo.INTAKE_PIVOT_PID_CONSTANTS.kd(),
                new TrapezoidProfile.Constraints(
                        2.5, 1.5
                )
        );

//        pivotPID = IntakeInfo.INTAKE_PIVOT_PID_CONSTANTS.create();

        pivotMotorRight.setIdleMode(CANSparkBase.IdleMode.kCoast);
        pivotMotorLeft.setIdleMode(CANSparkBase.IdleMode.kCoast);

        pivotMotorRight.setInverted(true);

        encoder = new DutyCycleEncoder(IDs.INTAKE_ENCODER_DIO_PORT);
        encoder.setDistancePerRotation(0.1);
        state = ExtensionState.RETRACTED;
    }

    public void setExtended(ExtensionState extended) {
        state = extended;
    }

    //Spins intake motor to intake notes
    public void intake() {
        intakeMotor.set(IntakeInfo.INTAKE_SPEED);
    }

    public void amp() {
        setExtended(IIntakeSubsystem.ExtensionState.AMP);
        intakeMotor.set(IntakeInfo.AMP_OUTTAKE_SPEED);
    }

    public void outtake() {
        intakeMotor.set(-IntakeInfo.INTAKE_SPEED);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    @Override
    public void periodic() {
        pivotPID.setGoal(switch (state) {
            case RETRACTED -> IntakeInfo.INTAKE_PIVOT_DEFAULT_SETPOINT;
            case EXTENDED -> IntakeInfo.INTAKE_PIVOT_EXTENDED_SETPOINT;
            case AMP -> IntakeInfo.INTAKE_PIVOT_AMP_SETPOINT;
        });
        double pidOutput = -pivotPID.calculate(getPosition());
        SmartDashboard.putString("Intake/Intake position", state.toString());
        SmartDashboard.putNumber("Intake/Intake angle", getPosition());
        SmartDashboard.putNumber("Intake/Intake PID output", pidOutput);
        SmartDashboard.putNumber("Intake/Intake applied voltage left", pivotMotorLeft.getAppliedOutput());
        SmartDashboard.putNumber("Intake/Intake applied voltage right", pivotMotorRight.getAppliedOutput());

        pivotMotorRight.set(pidOutput);
        pivotMotorLeft.set(pidOutput);
    }

    public void stop() {
        pivotPID.setGoal(getPosition());

        pivotMotorLeft.stopMotor();
        pivotMotorRight.stopMotor();
        intakeMotor.stopMotor();
    }

    private double getPosition() {
        return encoder.getAbsolutePosition();
    }
}