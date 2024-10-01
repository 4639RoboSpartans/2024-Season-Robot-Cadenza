package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SimIntakeSubsystem extends IntakeSubsystem {
    private double rawTargetAngle;
    private double intakeOutput;

    private MechanismRoot2d pivotRoot;
    private MechanismLigament2d pivot;

    public SimIntakeSubsystem() {
        setExtendedState(ExtensionState.RETRACTED);
        intakeOutput = 0;
    }

    @Override
    public double getRotations() {
        return rawTargetAngle;
    }

    @Override
    public void initMech(Mechanism2d mech) {
        pivotRoot = mech.getRoot("intake", 1, 1);
        pivot = pivotRoot.append(
                new MechanismLigament2d(
                        "pivot",
                        1,
                        measuredPivotRotationsToMechanismDegrees(getRotations())
//                        90
                )
        );
    }

    @Override
    protected void setExtendedState(ExtensionState extended) {
        rawTargetAngle = switch (extended) {
            case RETRACTED -> IntakeConstants.INTAKE_PIVOT_RETRACTED_SETPOINT;
            case EXTENDED -> IntakeConstants.INTAKE_PIVOT_EXTENDED_SETPOINT;
            case AMP -> IntakeConstants.INTAKE_PIVOT_AMP_SETPOINT;
        };
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
        intakeOutput = 0;
    }

    @Override
    protected void stopRun() {
        intakeOutput = 0;
    }

    @Override
    protected void intakeRun() {
        intakeOutput = IntakeConstants.INTAKE_SPEED;
    }

    @Override
    protected Trigger atSetPoint() {
        return new Trigger(() -> true);
    }

    @Override
    public void periodic() {
        pivot.setAngle(measuredPivotRotationsToMechanismDegrees(getRotations()));
    }

    private double measuredPivotRotationsToMechanismDegrees(double rotations) {
        return -Rotation2d.fromRotations(rotations).getDegrees() + IntakeConstants.INTAKE_PIVOT_MECHANISM_OFFSET_DEGREES;
    }

    @Override
    protected void buildSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Intake");
        builder.addDoubleProperty("Rotations",
                this::getRotations,
                null);
        builder.addDoubleProperty("Intake output",
                () -> intakeOutput,
                null);
    }
}
