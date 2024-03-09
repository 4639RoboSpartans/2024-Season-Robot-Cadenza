package frc.robot.subsystems.aim;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.RobotInfo;

public interface AimInterface extends Subsystem, Sendable {
    double getSwerveRotation();

    boolean isAtSetpoint();

    RobotInfo.ShooterInfo.ShooterSetpoint getShooterSetpoint();

    @Override
    void periodic();

    void resetPID();

    double getTxDerivative();
}
