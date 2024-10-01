package frc.robot.subsystems.shooter.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.AimUtil;

public class SimPivotSubsystem extends PivotSubsystem {
    private double rawTargetAngle;

    private MechanismRoot2d pivotRoot;
    private MechanismLigament2d pivot;

    public SimPivotSubsystem() {
        rawTargetAngle = ShooterConstants.SHOOTER_LOWER_OFFSET;
    }

    @Override
    protected void buildSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Pivot");
        builder.addDoubleProperty("Rotations",
                () -> rawTargetAngle,
                null);
        builder.addDoubleProperty("Target",
                () -> AimUtil.getShooterSetpoint().angle(),
                null);
    }

    @Override
    public double getRotations() {
        return ShooterConstants.SHOOTER_LOWER_OFFSET - rawTargetAngle;
    }

    @Override
    public void initMech(Mechanism2d mech) {
        pivotRoot = mech.getRoot("Pivot", 3, 1);
        pivot = pivotRoot.append(
                new MechanismLigament2d("Shooter", 1, getRotations())
        );
    }

    @Override
    public void runShootingAngle(double angle) {
        rawTargetAngle = angle;
    }

    @Override
    protected boolean atSetPointSupplier() {
        return true;
    }

    @Override
    public void periodic() {
        runShootingAngle(getTargetAngle());
        if (pivot != null)
            pivot.setAngle(Rotation2d.fromRotations(ShooterConstants.SHOOTER_LOWER_OFFSET - rawTargetAngle + 1.0 / 12.0));
    }
}
