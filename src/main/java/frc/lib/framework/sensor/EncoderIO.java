package frc.lib.framework.sensor;

public interface EncoderIO {
    public void update();

    public double getAngle();

    public double getRate();

    public boolean isConnected();
}
