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
                {1.1, 24.25, ShooterLowerOffset - 0.085},
                {1.2, 24.5, ShooterLowerOffset - 0.083},
                {1.3, 24.5, ShooterLowerOffset - 0.083},
                {1.4, 24.5, ShooterLowerOffset - 0.0785},
                {1.5, 25, ShooterLowerOffset - 0.0745},
                {1.6, 25, ShooterLowerOffset - 0.071},
                {1.7, 25, ShooterLowerOffset - 0.066},
                {1.8, 25.5, ShooterLowerOffset - 0.0635},
                {1.9, 26, ShooterLowerOffset - 0.0605},
                {2.0, 26, ShooterLowerOffset - 0.058},
                {2.1, 26.5, ShooterLowerOffset - 0.055},
                {2.2, 26.5, ShooterLowerOffset - 0.0505},
                {2.3, 26.75, ShooterLowerOffset - 0.0425},
                {2.4, 27, ShooterLowerOffset - 0.0395},
                {2.5, 27.25, ShooterLowerOffset - 0.0384},
                {2.6, 27.65, ShooterLowerOffset - 0.036},
                {2.7, 28.25, ShooterLowerOffset - 0.035},
                {2.8, 29.25, ShooterLowerOffset - 0.0325},
                {2.9, 30.6, ShooterLowerOffset - 0.0295},
                {3.0, 32.25, ShooterLowerOffset - 0.0274},
                {3.1, 34.5, ShooterLowerOffset - 0.0245},
                {3.2, 37, ShooterLowerOffset - 0.0185},
                {3.3, 39.5, ShooterLowerOffset - 0.0164},
                {3.4, 42, ShooterLowerOffset - 0.014},
                {3.5, 45, ShooterLowerOffset - 0.014},
                {3.6, 47.5, ShooterLowerOffset - 0.01},
                {3.7, 50, ShooterLowerOffset - 0.008},
                {3.8, 52.5, ShooterLowerOffset - 0.008},
                {3.9, 54.5, ShooterLowerOffset - 0.007},
                {4.0, 55.75, ShooterLowerOffset - 0.0055},
                {4.1, 56.5, ShooterLowerOffset - 0.0045},
                {4.2, 57.25, ShooterLowerOffset - 0.0035},
                {4.3, 58.5, ShooterLowerOffset - 0.0025}
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
