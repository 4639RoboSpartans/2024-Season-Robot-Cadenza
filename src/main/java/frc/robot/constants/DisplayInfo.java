package frc.robot.constants;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.led.LEDPattern;
import frc.robot.led.PhasingLEDPattern;
import frc.robot.led.SolidLEDPattern;

public class DisplayInfo {
    public static final Color8Bit readyColor = new Color8Bit(0, 255, 0);
    public static final LEDPattern readyPattern = new PhasingLEDPattern(readyColor, 2);
    public static final Color8Bit notReadyColor = new Color8Bit(255, 0, 0);
    public static final LEDPattern notReadyPattern = new SolidLEDPattern(notReadyColor);
}
