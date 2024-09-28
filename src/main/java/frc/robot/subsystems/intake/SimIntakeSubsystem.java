package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class SimIntakeSubsystem extends IntakeSubsystem {
    private ExtensionState state;
    private final DCMotorSim simPivot;
    private final ProfiledPIDController pivotPID;
    private double intakeOutput;

    private final Mechanism2d pivotMech;
    private final MechanismRoot2d pivotRoot;
    private final MechanismLigament2d pivot;

    public SimIntakeSubsystem() {
        state = ExtensionState.RETRACTED;
        simPivot = new DCMotorSim(DCMotor.getNeo550(2),
                IntakeConstants.INTAKE_PIVOT_GEARING,
                IntakeConstants.INTAKE_PIVOT_MOI);
        simPivot.setState(
                measuredPivotRotationsToMechanismDegrees(IntakeConstants.INTAKE_PIVOT_DEFAULT_SETPOINT),
                0
        );
        pivotPID = new ProfiledPIDController(
                IntakeConstants.INTAKE_PIVOT_kp,
                IntakeConstants.INTAKE_PIVOT_ki,
                IntakeConstants.INTAKE_PIVOT_kd,
                new TrapezoidProfile.Constraints(
                        IntakeConstants.INTAKE_PIVOT_VELOCITY,
                        IntakeConstants.INTAKE_PIVOT_ACCELERATION
                )
        );
        setExtendedState(ExtensionState.RETRACTED);
        intakeOutput = 0;

        pivotMech = new Mechanism2d(3, 3);
        pivotRoot = pivotMech.getRoot("intake", 1, 1);
        pivot = pivotRoot.append(
                new MechanismLigament2d(
                        "pivot",
                        2,
                        measuredPivotRotationsToMechanismDegrees(getAngle())
                )
        );
    }

    @Override
    protected void setExtendedState(ExtensionState extended) {
        pivotPID.setGoal(switch (extended) {
            case RETRACTED -> IntakeConstants.INTAKE_PIVOT_DEFAULT_SETPOINT;
            case EXTENDED -> IntakeConstants.INTAKE_PIVOT_EXTENDED_SETPOINT;
            case AMP -> IntakeConstants.INTAKE_PIVOT_AMP_SETPOINT;
        });
    }

    @Override
    protected void outtakeRun() {
        intakeOutput = -IntakeConstants.INTAKE_SPEED;
    }

    @Override
    protected void ampRun() {
        intakeOutput = IntakeConstants.AMP_OUTTAKE_SPEED;
    }

    @Override
    protected void stopIntakeRun() {

    }

    @Override
    protected void stopRun() {

    }

    @Override
    protected void intakeRun() {
        intakeOutput = IntakeConstants.INTAKE_SPEED;
    }

    @Override
    public void periodic() {
        double PIDOutput = pivotPID.calculate(getAngle());
        simPivot.setInputVoltage(PIDOutput);
        pivot.setAngle(measuredPivotRotationsToMechanismDegrees(getAngle()));
    }

    private double getAngle() {
        return simPivot.getAngularPositionRotations();
    }

    private double measuredPivotRotationsToMechanismDegrees(double rotations) {
        double angle = rotations % 1;
        return angle + Rotation2d.fromDegrees(IntakeConstants.INTAKE_PIVOT_MECHANISM_OFFSET_DEGREES).getRotations();
    }
}
