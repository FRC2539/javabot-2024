package frc.robot.subsystems.lights;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import frc.lib.math.MathUtils;
import frc.robot.Constants;

public class LightsIOCANdle implements LightsIO {
    private static final CANdle candle = new CANdle(Constants.LightsConstants.CANDLE_PORT);
    
    

}
