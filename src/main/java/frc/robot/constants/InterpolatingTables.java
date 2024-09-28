package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.tuning.TunableTableSource;

import static frc.robot.subsystems.shooter.ShooterConstants.SHOOTER_LOWER_OFFSET;

public class InterpolatingTables {
    private static TunableTableSource table;
    private static double[][] shots;
    private static InterpolatingDoubleTreeMap angleTable, speedTable;

    private static double ANGLE_OFFSET = 0;

    public static void initializeTables() {
        angleTable = new InterpolatingDoubleTreeMap();
        speedTable = new InterpolatingDoubleTreeMap();

        shots = new double[][]{
            {1.2, 24.25, SHOOTER_LOWER_OFFSET - 0.095},
            {2.1, 26.5, SHOOTER_LOWER_OFFSET - 0.048},
            {2.6, 30, SHOOTER_LOWER_OFFSET - 0.0325},
            {3.1, 34.5, SHOOTER_LOWER_OFFSET - 0.02},
            {4.0, 55.75, SHOOTER_LOWER_OFFSET - 0.01},
            {4.1, 56.5, SHOOTER_LOWER_OFFSET - 0.005},
            {4.3, 58.5, SHOOTER_LOWER_OFFSET}
        };

        table = new TunableTableSource(
                "Shooter/Shooter Setpoints",
                7, 3,
                new String[]{"dist", "speed", "angle"},
                shots
        );

        for (double[] shot : shots) {
            speedTable.put(shot[0], shot[1]);
            angleTable.put(shot[0], shot[2] + ANGLE_OFFSET);
        }
    }

    public static InterpolatingDoubleTreeMap getSpeedTable() {
        return speedTable;
    }

    public static InterpolatingDoubleTreeMap getAngleTable() {
        return angleTable;
    }

    public static void update() {
        for (int i = 0; i < shots.length; i++) {
            for (int j = 0; j < shots[0].length; j++) {
                shots[i][j] = table.getCellAsDouble(i, j);
            }
        }
    }
}
