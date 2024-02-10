package frc.robot.subsystems.lights;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.MathUtils;
import com.ctre.phoenix.led.Animation;

public class LightsSubsystem extends SubsystemBase {
    private LightsIO lightsIO;

     //Team colors
    //  public static final Color orange = new Color(255, 25, 0);
    //  public static final Color black = new Color(0, 0, 0);
 
    //  // Indicator colors
    //  public static final Color white = new Color(255, 230, 220);
    //  public static final Color green = new Color(56, 209, 0);
    //  public static final Color blue = new Color(8, 32, 255);
    //  public static final Color red = new Color(255, 0, 0);


    public LightsSubsystem(LightsIO lightsIO) {
        this.lightsIO = lightsIO;
        // setDefaultCommand(defaultCommand());
    }

    //Flash, solid, rainbow animations

    // public Command defaultCommand() {
    //     return runOnce(() -> set(orange));
    // }

    // public Command patternCommand(Pattern pattern) {
    //     return runOnce(() -> setPattern(pattern));
    // }

    public Command animationCommand(Animation animation) {
        return runOnce(() -> setAnimation(animation));
    }

    private void setAnimation(Animation animation) {
        lightsIO.setAnimation(animation);
    }

    public static class Color {
        public int red;
        public int green;
        public int blue;

        public Color(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }

        /**
         * Highly imperfect way of dimming the LEDs. It does not maintain color or
         * accurately adjust perceived brightness.
         *
         * @param dimFactor
         * @return The dimmed color
         */
        public Color dim(double dimFactor) {
            int newRed = (int) (MathUtils.ensureRange(red * dimFactor, 0, 200));
            int newGreen = (int) (MathUtils.ensureRange(green * dimFactor, 0, 200));
            int newBlue = (int) (MathUtils.ensureRange(blue * dimFactor, 0, 200));

            return new Color(newRed, newGreen, newBlue);
        }
    }

    // private void setPattern(Pattern pattern) {
    //     lightsIO.setPattern(pattern);
    // }
}
