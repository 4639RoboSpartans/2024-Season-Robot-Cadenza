package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import static frc.robot.constants.RobotInfo.ShooterInfo.ShooterLowerOffset;

public class InterpolatingTables {
    private static InterpolatingDoubleTreeMap angleTable, speedTable;

    private static double ANGLE_OFFSET = 0;

    public static void initializeTables() {
        angleTable = new InterpolatingDoubleTreeMap();
        speedTable = new InterpolatingDoubleTreeMap();

        double[][] shots = {
                {1.2, 24.25, ShooterLowerOffset - 0.083},
                {2.1, 26.5, ShooterLowerOffset - 0.06},
                {3.1, 34.5, ShooterLowerOffset - 0.03},
                {4.0, 55.75, ShooterLowerOffset - 0.02},
                {4.1, 56.5, ShooterLowerOffset - 0.01},
                {4.3, 58.5, ShooterLowerOffset}
        };

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
}