package frc.robot.constants;

import edu.wpi.first.math.controller.PIDController;

@SuppressWarnings("unused")
public record PIDTemplate(double kp, double ki, double kd) {
    public PIDTemplate(double kp) {
        this(kp, 0);
    }

    public PIDTemplate(double kp, double ki) {
        this(kp, ki, 0);
    }

    public PIDTemplate withKp(double kp) {
        return new PIDTemplate(kp, ki, kd);
    }

    public PIDTemplate withKi(double ki) {
        return new PIDTemplate(kp, ki, kd);
    }

    public PIDTemplate withKd(double kd) {
        return new PIDTemplate(kp, ki, kd);
    }

    public PIDTemplate adjustKp(double multiplier) {
        return new PIDTemplate(kp * multiplier, ki, kd);
    }

    public PIDTemplate adjustKi(double multiplier) {
        return new PIDTemplate(kp, ki * multiplier, kd);
    }

    public PIDTemplate adjustKd(double multiplier) {
        return new PIDTemplate(kp, ki, kd * multiplier);
    }

    public PIDController create() {
        return create(1);
    }

    public PIDController create(double kPMultiplier) {
        return new PIDController(kp * kPMultiplier, ki, kd);
    }
}
