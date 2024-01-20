package frc.robot.oi;

public class OI {
    private final Controller driverController;
    private final Controller operatorController;

    public OI() {
        driverController = new Controller(0);
        operatorController = new Controller(1);
    }

    public Controller getOperatorController() {
        return operatorController;
    }

    public Controller getDriverController() {
        return driverController;
    }

    public enum Axes {
        LEFT_STICK_X(0),
        LEFT_STICK_Y(1, true),
        LEFT_TRIGGER(2),
        RIGHT_TRIGGER(3),
        RIGHT_STICK_X(4),
        RIGHT_STICK_Y(5, true);

        private final int id;
        private final boolean shouldInvert;

        Axes(int id) {
            this(id, false);
        }

        Axes(int id, boolean shouldInvert) {
            this.id = id;
            this.shouldInvert = shouldInvert;
        }

        public int getID() {
            return id;
        }

        public boolean shouldInvert() {
            return shouldInvert;
        }
    }

    public enum Buttons {
        A_BUTTON(1),
        B_BUTTON(2),
        X_BUTTON(3),
        Y_BUTTON(4),
        LEFT_BUMPER(5),
        RIGHT_BUMPER(6),
        BACK_BUTTON(7),
        START_BUTTON(8),
        LEFT_STICK(9),
        RIGHT_STICK(10),
        POW_UP(11),
        POW_RIGHT(12),
        POW_DOWN(13),
        POW_LEFT(14);

        private final int id;

        Buttons(int id) {
            this.id = id;
        }

        public int getID() {
            return id;
        }
    }
}
