package frc.robot.led;

import edu.wpi.first.wpilibj.util.Color8Bit;

public final class SolidLEDPattern implements LEDPattern {
    private final Color8Bit color;

    public SolidLEDPattern(Color8Bit color) {
        this.color = color;
    }

    @Override
    public Color8Bit get(int led, double time) {
        return color;
    }
}
