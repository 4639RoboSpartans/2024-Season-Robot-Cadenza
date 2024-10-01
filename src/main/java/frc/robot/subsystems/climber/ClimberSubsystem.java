package frc.robot.subsystems.climber;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import java.util.Objects;

public abstract class ClimberSubsystem extends SubsystemBase {
    private static ClimberSubsystem instance;

    public static ClimberSubsystem getInstance() {
        return instance = Objects.requireNonNullElseGet(instance,
                Robot.isReal()? ConcreteClimberSubsystem::new
                : SimClimberSubsystem::new);
    }

    public Command applyPIDControl() {
        return run(this::runPID);
    }

    public Command up() {
        return runOnce(this::runUp);
    }

    public Command down() {
        return runOnce(this::runDown);
    }

    public Command swapLeftUp() {
        return runOnce(this::runSwapLeftUp);
    }

    public Command swapRightUp() {
        return runOnce(this::runSwapRightUp);
    }

    protected abstract void runPID();

    protected abstract void runUp();

    protected abstract void runDown();

    protected abstract void runSwapLeftUp();

    protected abstract void runSwapRightUp();

    public abstract void transitionToPIDControl();

    @Override
    public void initSendable(SendableBuilder builder) {
        buildSendable(builder);
    }

    protected abstract void buildSendable(SendableBuilder builder);

    public abstract void initMech(Mechanism2d mech);

    public abstract Command rootMech();
}
