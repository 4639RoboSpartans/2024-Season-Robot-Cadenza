package frc.robot.led;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Objects;

public abstract class LEDStrip extends SubsystemBase {
    private static LEDStrip instance;

    public static LEDStrip getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, () -> new PhysicalLEDStrip(0, 64));
    }

    public abstract void usePattern(LEDPattern pattern);

    public abstract void update();

    @Override
    public void periodic() {
        update();
    }

    public abstract void resetToBlank();
}
