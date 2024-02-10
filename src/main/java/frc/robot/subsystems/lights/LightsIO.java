package frc.robot.subsystems.lights;

import frc.lib.math.MathUtils;

public interface LightsIO {

    public void setAnimation(Animation animation);

    public static class Animation {
        public double speed;

        public Animation(double speed) {
            this.speed = speed;
        }
    }

}
