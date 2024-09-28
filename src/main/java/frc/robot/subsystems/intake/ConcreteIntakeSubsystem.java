package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.lib.util.VelocityDutyCycleEncoder;

import static frc.robot.constants.RobotInfo.IntakeInfo;

public class ConcreteIntakeSubsystem extends IntakeSubsystem {
    private final CANSparkMax pivotMotorLeft;
    private final CANSparkMax pivotMotorRight;
    private final CANSparkMax intakeMotor;
    private final VelocityDutyCycleEncoder encoder;

    private final ProfiledPIDController pivotPID;

    private MechanismRoot2d pivotRoot;
    private MechanismLigament2d pivot;

    public ConcreteIntakeSubsystem() {
        pivotMotorLeft = new CANSparkMax(IntakeConstants.IDs.INTAKE_PIVOT_MOTOR_LEFT,
                CANSparkMax.MotorType.kBrushless);
        pivotMotorRight = new CANSparkMax(IntakeConstants.IDs.INTAKE_PIVOT_MOTOR_RIGHT,
                CANSparkMax.MotorType.kBrushless);
        intakeMotor = new CANSparkMax(IntakeConstants.IDs.INTAKE_MOTOR,
                CANSparkMax.MotorType.kBrushless);
        pivotMotorLeft.restoreFactoryDefaults();
        pivotMotorRight.restoreFactoryDefaults();

        pivotMotorLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
        pivotMotorRight.setIdleMode(CANSparkBase.IdleMode.kBrake);
        intakeMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);

        pivotMotorLeft.setSmartCurrentLimit(40);
        pivotMotorRight.setSmartCurrentLimit(40);
        intakeMotor.setSmartCurrentLimit(40);

        pivotMotorRight.follow(pivotMotorLeft, true);

        pivotPID = new ProfiledPIDController(
                IntakeConstants.INTAKE_PIVOT_kp,
                IntakeConstants.INTAKE_PIVOT_ki,
                IntakeConstants.INTAKE_PIVOT_kd,
                new TrapezoidProfile.Constraints(
                        IntakeConstants.INTAKE_PIVOT_VELOCITY,
                        IntakeConstants.INTAKE_PIVOT_ACCELERATION
                )
        );

        encoder = new VelocityDutyCycleEncoder(IntakeConstants.IDs.INTAKE_ENCODER_DIO_PORT);
        encoder.setDutyCycleRange(-2, 2);
    }

    @Override
    protected void setExtendedState(ExtensionState extended) {
        pivotPID.setGoal(switch (extended) {
            case EXTENDED -> IntakeInfo.INTAKE_PIVOT_EXTENDED_SETPOINT;
            case RETRACTED -> IntakeInfo.INTAKE_PIVOT_DEFAULT_SETPOINT;
            case AMP -> IntakeInfo.INTAKE_PIVOT_AMP_SETPOINT;
        });
    }

    @Override
    protected void outtakeRun() {
        intakeMotor.set(-IntakeInfo.INTAKE_SPEED);
    }

    @Override
    protected void ampRun() {
        intakeMotor.set(-IntakeInfo.INTAKE_SPEED);
    }

    @Override
    protected void stopIntakeRun() {
        intakeMotor.set(0);
    }

    @Override
    protected void stopRun() {
        intakeMotor.set(0);
        pivotPID.setGoal(getRotations());
    }

    @Override
    protected void intakeRun() {
        intakeMotor.set(IntakeInfo.INTAKE_SPEED);
    }

    @Override
    public void periodic() {
        double PIDOutput = pivotPID.calculate(getRotations());
        if (pivot != null) {
            pivotMotorLeft.set(PIDOutput);
            pivotMotorLeft.setIdleMode(DriverStation.isDisabled() ?
                    CANSparkBase.IdleMode.kCoast : CANSparkBase.IdleMode.kBrake);
            pivot.setAngle(measuredPivotRotationsToMechanismDegrees(getRotations()));
        }
    }

    private double measuredPivotRotationsToMechanismDegrees(double rotations) {
        double angle = Rotation2d.fromRotations(rotations).getDegrees() % 360;
        return angle + IntakeConstants.INTAKE_PIVOT_MECHANISM_OFFSET_DEGREES;
    }

    @Override
    public double getRotations() {
        return encoder.getAbsolutePosition();
    }

    @Override
    public void instantiateMech(Mechanism2d mech) {
        pivotRoot = mech.getRoot("Pivot", 3, 1);
        pivot = pivotRoot.append(
                new MechanismLigament2d(
                        "Arm",
                        3,
                        getRotations()
                )
        );
    }
}