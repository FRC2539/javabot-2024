package frc.lib.math;

public class Conversions {

    /**
     * @param rotations Rotations from the Motor
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return Radians of Rotation of Mechanism
     */
    public static double motorToRadians(double rotations, double gearRatio) {
        return rotations * (Math.PI * 2 / gearRatio);
    }

    /**
     * @param radians Radians of Rotation of Mechanism
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return Rotations of the Motor
     */
    public static double radiansToMotor(double radians, double gearRatio) {
        return radians / (Math.PI * 2) * gearRatio;
    }

    /**
     * @param rotations Rotations from the Motor
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @param circumference Circumference of the Mechanism
     * @return Meters of Movement of Mechanism
     */
    public static double motorToMeters(double rotations, double circumference, double gearRatio) {
        return rotations * circumference / gearRatio;
    }

    /**
     * @param meters Meters of Movement of Mechanism
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @param circumference Circumference of the Mechanism
     * @return Rotations Per Second of Motor
     */
    public static double MetersToMotor(double meters, double circumference, double gearRatio) {
        return meters * gearRatio / circumference;
    }

    /**
     * @param RPS Rotations per Second for the Motor
     * @param gearRatioGear Ratio between Motor and Mechanism
     * @return Radians per Second of Rotation of Mechanism
     */
    public static double motorRPSToRadPS(double RPS, double gearRatio) {
        return RPS * (Math.PI * 2) / gearRatio;
    }

    /**
     * @param radians Radians Per Second of Rotation of Mechanism
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @return Rotations Per Second of the Motor
     */
    public static double RadPSToMotorRPS(double RadPS, double gearRatio) {
        return RadPS / (Math.PI * 2) * gearRatio;
    }

    /**
     * @param RPS Rotations per Second for the Motor
     * @param gearRatio Gear Ratio between Motor and Mechanism\
     * @param circumference Circumference of the Mechanism
     * @return Meters per Second of Movement of Mechanism
     */
    public static double motorRPSToMPS(double RPS, double circumference, double gearRatio) {
        return RPS / gearRatio * circumference;
    }

    /**
     * @param MPS Meters Per Second of Movement of Mechanism
     * @param gearRatio Gear Ratio between Motor and Mechanism
     * @param circumference Circumference of the Mechanism
     * @return Rotations Per Second of Motor
     */
    public static double MPSToMotorRPS(double MPS, double circumference, double gearRatio) {
        return MPS * gearRatio / circumference;
    }
}
