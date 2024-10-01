package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.shooter.ShooterSubsystem;

import java.util.Objects;

public class ShooterSuperstructure extends SubsystemBase {
    private static ShooterSuperstructure instance;

    public static ShooterSuperstructure getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, ShooterSuperstructure::new);
    }

    private final ShooterSubsystem shooterSubsystem;
    private final PivotSubsystem pivotSubsystem;

    private ShooterSuperstructure() {
        shooterSubsystem = ShooterSubsystem.getInstance();
        pivotSubsystem = PivotSubsystem.getInstance();
    }

    public Command runShootingMode(ShooterConstants.ShootingMode mode) {
        return shooterSubsystem.runShootingMode(mode).alongWith(pivotSubsystem.setShootingMode(mode));
    }

    public Trigger atSetPoint() {
        return shooterSubsystem.atSetPoint().and(pivotSubsystem.atSetPoint());
    }

    public void sendSubsystems() {
        SmartDashboard.putData(shooterSubsystem);
        SmartDashboard.putData(pivotSubsystem);
    }

    public Command shooterSysIDQuasistatic() {
        return shooterSubsystem.getSysIDQuasistaticCommand();
    }

    public Command shooterSysIDDynamic() {
        return shooterSubsystem.getSysIDDynamicCommand();
    }
}
