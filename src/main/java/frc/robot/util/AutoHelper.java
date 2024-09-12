package frc.robot.util;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.intake.ExtendIntakeCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.shooter.AutoSpeakerCommand;
import frc.robot.commands.shooter.ShooterSpinupCommand;
import frc.robot.constants.Controls;
import frc.robot.constants.RobotInfo;
import frc.robot.constants.RobotInfo.SwerveInfo;
import frc.robot.led.LEDStrip;
import frc.robot.oi.OI;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.intake.IIntakeSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;

public class AutoHelper {
    private static ISwerveDriveSubsystem swerve = SubsystemManager.getSwerveDrive();
    private static IShooterSubsystem shooter = SubsystemManager.getShooter();
    private static IIntakeSubsystem intake = SubsystemManager.getIntake();
    private static IHopperSubsystem hopper = SubsystemManager.getHopper();
    private static LEDStrip ledStrip = SubsystemManager.getLedStrip();
    private static OI oi = SubsystemManager.getOI();

    public static Command follow(String pathName) {
        return swerve.followChoreoPath(pathName, true);
    }

    public static Command intakeWhileMoving(String pathName) {
        ChoreoTrajectory traj = Choreo.getTrajectory(pathName);
        double time = traj.getTotalTime();
        double startTime = time - RobotInfo.SwerveInfo.TIME_BEFORE_INTAKE_START;
        Command ret = Commands.parallel(
                follow(pathName),
                startTime < 0 ?
                        new IntakeCommand(intake, hopper, ledStrip, oi) :
                        Commands.sequence(
                                new WaitCommand(startTime),
                                new IntakeCommand(intake, hopper, ledStrip, oi)
                        )
        );
        return ret;
    }

    public static Command SOTFCommand(String pathName) {
        /*
         * max velocity of traj during SOTF portion must not be too high (<=2 m/s),
         * should generate path so that bot is facing speaker during SOTF portion
         */
        ChoreoTrajectory traj = Choreo.getTrajectory(pathName);
        Command shooterAutoSpinupCommand = new ShooterSpinupCommand(shooter).onlyWhile(Controls.spinupTrigger);
        Command shooterSOTFCommand = new AutoSpeakerCommand(shooter, hopper, ledStrip);
        Command ret = Commands.deadline(
                AutoHelper.follow(pathName),
                Commands.sequence(
                        Commands.deadline(
                                new WaitCommand(traj.getTotalTime() - SwerveInfo.TIME_BEFORE_INTAKE_START),
                                Commands.sequence(
                                        Commands.waitUntil(Controls.spinupTrigger),
                                        shooterAutoSpinupCommand.until(Controls.canSOTF),
                                        shooterSOTFCommand)
                        ),
                        new IntakeCommand(intake, hopper, ledStrip, oi)
                )
        );
        return ret;
    }

    public static Command followThenShoot(String pathname) {
        ChoreoTrajectory traj = Choreo.getTrajectory(pathname);
        Command ret = Commands.sequence(
                swerve.followChoreoPath(traj, true),
                shoot()
        );
        return ret;
    }

    public static Command shoot() {
        return Commands.parallel(
                Commands.race(new WaitCommand(Robot.isReal()?1.5:0),
                        new AutoSpeakerCommand(shooter, hopper, ledStrip)),
                new ExtendIntakeCommand(intake)
        );
    }


    public static Command ampPrepCommand() {
        return Commands.parallel(
                        Commands.run(intake::outtake),
                        Commands.run(() -> intake.setExtended(IIntakeSubsystem.ExtensionState.EXTENDED))
                )
                .until(() -> !hopper.hasNote());
    }
}
