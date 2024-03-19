package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DisplayInfo;
import frc.robot.constants.RobotInfo.ShooterInfo.ShootingMode;
import frc.robot.led.LEDPattern;
import frc.robot.led.LEDStrip;
import frc.robot.led.PhasingLEDPattern;
import frc.robot.led.SolidLEDPattern;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;

public class ShootCommand extends Command {
    private final IShooterSubsystem shooter;
    private final IHopperSubsystem hopper;
    private final ShootingMode mode;
    private final LEDStrip ledStrip;

    public ShootCommand(IShooterSubsystem shooter, IHopperSubsystem hopper, LEDStrip ledStrip, ShootingMode mode) {
        this.shooter = shooter;
        this.hopper = hopper;
        this.mode = mode;
        this.ledStrip = ledStrip;

        addRequirements(shooter, hopper, ledStrip);
    }

    @Override
    public void initialize() {
        System.out.println("Starting AutoShoot!");
    }

    @Override
    public void execute() {
        shooter.setShootingMode(mode);

        if(shooter.isReady()) {
            ledStrip.usePattern(DisplayInfo.readyPattern);
            hopper.run(false);
        }
        else {
            ledStrip.usePattern(DisplayInfo.notReadyPattern);
            hopper.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return hopper.getIR().isActive() && hopper.getIR().rawSensorIsClear();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShootingMode(ShootingMode.IDLE);
        hopper.stop();
        ledStrip.usePattern(LEDPattern.BLANK);
    }
}