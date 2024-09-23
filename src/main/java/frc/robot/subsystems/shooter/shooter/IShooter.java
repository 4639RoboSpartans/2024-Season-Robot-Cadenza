package frc.robot.subsystems.shooter.shooter;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IShooter extends Subsystem, Sendable {
    Command autoShoot();

    Command manualShoot();

    Command launch();

    Command idle();

    Command stop();

    Trigger atSpeed();
}
