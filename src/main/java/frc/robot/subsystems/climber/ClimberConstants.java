package frc.robot.subsystems.climber;

import frc.lib.util.PIDTemplate;

public class ClimberConstants {
    public static final double CLIMBER_SPEED = 0.9;

    public static final double CLIMBER_SCALAR_TO_MECHANISM = 1 / 100.0;

    public static final PIDTemplate positionController = new PIDTemplate(
            0, 0, 0
    );

    public class IDs {
        public static final int CLIMBER_LEFT = 15;
        public static final int CLIMBER_RIGHT = 16;
    }
}
