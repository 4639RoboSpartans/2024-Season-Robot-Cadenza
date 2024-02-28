package frc.robot.led;

public interface LEDStrip {
    void usePattern(LEDPattern pattern);

    void update();

    void stop();
}
