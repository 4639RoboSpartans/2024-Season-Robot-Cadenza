package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.CommandsUtil;
import frc.robot.CommandFactory;

import java.util.ArrayList;
import java.util.List;

public class Autos {
    private static int startPos = 1;
    private static int[] spikeNotes = {2, 3, 1};
    private static int[] rushNotes = {1, 2, 3, 4, 5};

    public static Command getAutoCommand() {
        List<Command> commands = new ArrayList<Command>();
        switch (startPos) {
            case 0, 3:
                commands.add(AutoHelper.SOTFCommand("S" + startPos + "-A" + spikeNotes[0] + " SOTF"));
                for (int i = 0; i < spikeNotes.length - 1; i++) {
                    commands.add(AutoHelper.SOTFCommand(
                            "A" + spikeNotes[i] + "-A" + spikeNotes[i + 1] + " SOTF"));
                }
                break;
            case 1:
                commands.add(CommandFactory.pureShoot());
                commands.add(AutoHelper.intakeWhileMoving("S2-A" + spikeNotes[0]));
                for (int i = 0; i < spikeNotes.length - 1; i++) {
                    commands.add(AutoHelper.SOTFCommand(
                            "A" + spikeNotes[i] + "-A" + spikeNotes[i + 1] + " SOTF"));
                }
                break;
        }
        int midPos = spikeNotes[spikeNotes.length - 1];
        if (rushNotes.length > 0) {
            commands.add(AutoHelper.intakeWhileMoving("A" + midPos + "-C" + rushNotes[0]));
            for (int i = 0; i < rushNotes.length - 1; i++) {
                commands.add(AutoHelper.SOTFCommand("C" + rushNotes[i] + "-C" + rushNotes[i + 1] + " SOTF"));
            }
            commands.add(AutoHelper.followThenShoot("C" + rushNotes[rushNotes.length - 1] + "-SS"));
        }
        return CommandsUtil.sequence(commands);
    }
}
