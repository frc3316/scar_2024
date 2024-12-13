package frc.robot.constants;

public class ArmConstants {
    public static final int baseCANID = 16;
    public static final int wristPWMchannel = 0;

    public static final double kp = 0;
    public static final double kfBase = 0;

    public static final double squareVolt = -0.03; //rpm
    public static final double circleVolt = 0.1; //rpm
    public static final double offVolt = 0; //rpm

    public static final double armlength1 = 300; //mm
    public static final double armlength2 = 200;//mm

    private static final double gearRatio = 1.0 / 16.0;
    public static final double positionFactor = 360 * gearRatio; // deg/rotations
    public static final double velocityFactor = 360 * gearRatio / 60; // (deg/sec)/RPM


}
  