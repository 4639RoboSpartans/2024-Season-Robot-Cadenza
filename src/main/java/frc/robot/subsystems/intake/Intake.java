package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.constants.IntakeConstants;

import java.util.function.DoubleConsumer;

public class Intake extends SubsystemBase implements IIntakeSubsystem {
    private final CANSparkMax pivotLeft, pivotRight, intake;
    private final ProfiledPIDController pivotPID;
    private final DutyCycleEncoder encoder;

    private static Intake instance;

    private Intake() {
        pivotLeft = new CANSparkMax(IntakeConstants.IDs.INTAKE_PIVOT_MOTOR_LEFT,
                CANSparkMax.MotorType.kBrushless);
        pivotRight = new CANSparkMax(IntakeConstants.IDs.INTAKE_PIVOT_MOTOR_RIGHT,
                CANSparkMax.MotorType.kBrushless);
        pivotLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
        pivotRight.setIdleMode(CANSparkBase.IdleMode.kBrake);

        pivotRight.follow(pivotLeft, true);

        intake = new CANSparkMax(IntakeConstants.IDs.INTAKE_MOTOR,
                CANSparkMax.MotorType.kBrushless);
        intake.setIdleMode(CANSparkBase.IdleMode.kCoast);

        pivotPID = new ProfiledPIDController(
                IntakeConstants.INTAKE_PIVOT_kp,
                IntakeConstants.INTAKE_PIVOT_ki,
                IntakeConstants.INTAKE_PIVOT_kd,
                new TrapezoidProfile.Constraints(
                        IntakeConstants.INTAKE_PIVOT_VELOCITY,
                        IntakeConstants.INTAKE_PIVOT_ACCELERATION
                )
        );

        encoder = new DutyCycleEncoder(IntakeConstants.IDs.INTAKE_ENCODER_DIO_PORT);
    }

    @Override
    public Command intake() {
        return runOnce(() -> intake.set(IntakeConstants.INTAKE_SPEED));
    }

    @Override
    public Command outtake() {
        return runOnce(() -> intake.set(-IntakeConstants.INTAKE_SPEED));
    }

    @Override
    public Command pivotExtend() {
        return runOnce(() -> pivotPID.setGoal(IntakeConstants.INTAKE_PIVOT_EXTENDED_SETPOINT))
                .andThen(runPivot().alongWith(stopIntake()));
    }

    @Override
    public Command pivotRetract() {
        return runOnce(() -> pivotPID.setGoal(IntakeConstants.INTAKE_PIVOT_DEFAULT_SETPOINT))
                .andThen(runPivot().alongWith(stopIntake()));
    }

    @Override
    public Command pivotOuttake() {
        return pivotRetract().alongWith(outtake());
    }

    @Override
    public Command ampPrep() {
        return pivotExtend().alongWith(outtake());
    }

    @Override
    public Command pivotIntake() {
        return pivotExtend().alongWith(intake());
    }

    @Override
    public Command pivotAmp() {
        return runOnce(() -> pivotPID.setGoal(IntakeConstants.INTAKE_PIVOT_AMP_SETPOINT))
                .andThen(runPivot()
                        .alongWith(outtake()));
    }

    @Override
    public Command stopIntake() {
        return runOnce(() -> intake.set(0));
    }

    @Override
    public Command stop() {
        return runOnce(() -> {
            intake.set(0);
            pivotPID.setGoal(getIntakePosition());
        }).andThen(runPivot());
    }

    private Command runPivot() {
        return run(() -> {
            double PIDOutput = pivotPID.calculate(getIntakePosition());
            pivotLeft.set(PIDOutput);
        });
    }

    private double getIntakePosition() {
        return encoder.getAbsolutePosition();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Intake");
        builder.addDoubleProperty("Position",
                this::getIntakePosition,
                (DoubleConsumer) null);
        builder.addDoubleProperty("PID output",
                () -> pivotPID.calculate(getIntakePosition()),
                (DoubleConsumer) null);
        builder.addDoubleProperty("Intake speed",
                intake::getAppliedOutput,
                (DoubleConsumer) null);
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }
}
