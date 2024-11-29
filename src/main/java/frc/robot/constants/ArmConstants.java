package frc.robot.constants;

public class ArmConstants {
    public static final int baseCANID = 1;
    public static final int wristPWMchannel = 2;

    public static final double kp = 0;
    public static final double kfBase = 0;

    public static final double squareVelocity = 0;
    public static final double circleVelocity = 0;
    public static final double offVelocity = 0;

    public static final double armlength1 = 300;
    public static final double armlength2 = 200;

    private static final double gearRatio = 1.0 / 192.0;
    public static final double positionFactor = 360 * gearRatio; // deg/rotations
    public static final double velocityFactor = 360 * gearRatio / 60; // (deg/sec)/RPM


}
