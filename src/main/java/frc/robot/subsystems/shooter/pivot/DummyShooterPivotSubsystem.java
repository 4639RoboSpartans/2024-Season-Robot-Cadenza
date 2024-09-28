package frc.robot.subsystems.shooter.pivot;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class DummyShooterPivotSubsystem extends PivotSubsystem {
    private double currentAngle;

    private MechanismRoot2d pivotRoot;
    private MechanismLigament2d pivot;

    public DummyShooterPivotSubsystem() {
        currentAngle = 0;
    }

    @Override
    protected void buildSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Pivot");
        builder.addDoubleProperty("Rotations",
                () -> currentAngle,
                null);
    }

    @Override
    public double getRotations() {
        return currentAngle;
    }

    @Override
    public void instantiateMech(Mechanism2d mech) {
        pivotRoot = mech.getRoot("Pivot", 3, 1);
        pivot = pivotRoot.append(
                new MechanismLigament2d("Shooter", 1, getRotations())
        );
    }

    @Override
    public void runShootingAngle(double angle) {
        currentAngle = angle;
    }

    @Override
    protected boolean atSetPointSupplier() {
        return true;
    }

    @Override
    public void periodic() {
        pivot.setAngle(currentAngle);
    }
}
