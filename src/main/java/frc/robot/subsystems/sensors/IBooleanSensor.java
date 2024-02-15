package frc.robot.subsystems.sensors;

import java.util.function.BooleanSupplier;

public interface IBooleanSensor extends BooleanSupplier {
    boolean get();
}
