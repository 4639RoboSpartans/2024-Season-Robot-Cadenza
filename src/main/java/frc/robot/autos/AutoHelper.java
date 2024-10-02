package frc.robot.autos;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CommandFactory;
import frc.robot.subsystems.swerve.SwerveConstants;

public class AutoHelper {

    public static Command follow(String pathName) {
        return CommandFactory.followPathCommand(pathName);
    }

    public static Command intakeWhileMoving(String pathName) {
        ChoreoTrajectory traj = Choreo.getTrajectory(pathName);
        double time = traj.getTotalTime();
        double startTime = time - SwerveConstants.TIME_BEFORE_INTAKE_START;
        return Commands.parallel(
                follow(pathName),
                startTime < 0 ?
                        CommandFactory.intakeCommand().withTimeout(1) :
                        Commands.sequence(
                                new WaitCommand(startTime),
                                CommandFactory.intakeCommand().withTimeout(1)
                        )
        );
    }

    public static Command SOTFCommand(String pathName) {
        /*
         * max velocity of trajectory during SOTF portion must not be too high (<=2 m/s),
         * should generate the path so that bot is facing the speaker during SOTF portion
         */
        ChoreoTrajectory traj = Choreo.getTrajectory(pathName);
        return Commands.parallel(
                AutoHelper.follow(pathName),
                Commands.sequence(
                        Commands.deadline(
                                new WaitCommand(traj.getTotalTime() - SwerveConstants.TIME_BEFORE_INTAKE_START),
                                CommandFactory.shootCommand()
                        ),
                        CommandFactory.intakeCommand().alongWith(CommandFactory.shooterIdleCommand()).withTimeout(1)
                )
        );
    }

    public static Command followThenShoot(String pathname) {
        return Commands.sequence(
                CommandFactory.followPathCommand(pathname),
                shoot()
        );
    }

    public static Command shoot() {
        return CommandFactory.aimCommand().withTimeout(1)
                .andThen(CommandFactory.pureShoot());
    }
}
