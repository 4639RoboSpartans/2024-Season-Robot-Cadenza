package frc.robot.constants;

import edu.wpi.first.math.controller.PIDController;

public record PIDConstants(double kp, double ki, double kd) {
    public PIDConstants(double kp) {
        this(kp, 0);
    }

    public PIDConstants(double kp, double ki) {
        this(kp, ki, 0);
    }

    public PIDController create() {
        return create(1);
    }

    public PIDController create(double kPMultiplier) {
        return new PIDController(kp * kPMultiplier, ki, kd);
    }
}
