package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SubsystemManager;
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
    public static Command Rush0_45 = Commands.sequence(
            AutoHelper.followThenShoot("S3-C5 SOTF.1"),
            AutoHelper.intakeWhileMoving("S3-C4 SOTF.2"),
            AutoHelper.SOTFCommand("C5-C4 SOTF"),
            AutoHelper.followThenShoot("C4-SS")
    );
    public static Command Spikes = Commands.sequence(
            AutoHelper.SOTFCommand("S2-A2 SOTF"),
            AutoHelper.SOTFCommand("A2-A1 SOTF"),
            AutoHelper.SOTFCommand("A1-A3 SOTF"),
            AutoHelper.shoot()
    );
    public static Command FU_Auto = Commands.sequence(
            AutoHelper.intakeWhileMoving("A1-FU.1"),
            AutoHelper.follow("A1-FU.2"),
            AutoHelper.followThenShoot("A1-FU.3")
    );
    public static Command[] getAutos() {
        return new Command[] {
                Rush01_12,
                Rush01_13,
                Rush0_45,
                Spikes,
                FU_Auto,
                testAuto("S1-A1 SOTF")
        };
    }
    public static Command testAuto(String pathName) {
        return AutoHelper.follow(pathName).withName(pathName);
    }
}
