package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.AutoHelper;

import java.util.ArrayList;

public class AutoFactory {
    public static Command Rush01_12 = Commands.sequence(
            AutoHelper.SOTFCommand("S1-A1"),
            AutoHelper.intakeWhileMoving("A1-C1"),
            AutoHelper.SOTFCommand("C1-C2 SOTF"),
            AutoHelper.followThenShoot("C2-SS")
    );
    public static Command Rush01_13 = Commands.sequence(
            AutoHelper.SOTFCommand("S1-A1"),
            AutoHelper.intakeWhileMoving("A1-C1"),
            AutoHelper.SOTFCommand("C1-C2 SOTF"),
            AutoHelper.SOTFCommand("C2-C3 SOTF"),
            AutoHelper.followThenShoot("C3-SS")
    );
    public static Command Spikes = Commands.sequence(
            AutoHelper.SOTFCommand("S2-A2 SOTF"),
            AutoHelper.SOTFCommand("A2-A1 SOTF"),
            AutoHelper.SOTFCommand("A1-A3 SOTF"),
            AutoHelper.shoot()
    );
    public static Command[] getAutos() {
        return new Command[] {
                Rush01_12,
                Rush01_13,
                Spikes,
                testAuto
        };
    }
    public static Command testAuto = AutoHelper.intakeWhileMoving("S1-A1");
}
