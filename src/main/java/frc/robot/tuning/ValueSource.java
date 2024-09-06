package frc.robot.tuning;

public interface ValueSource {
    String name();
    default void update() {}
}
