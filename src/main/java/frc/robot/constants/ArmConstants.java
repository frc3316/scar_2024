package frc.robot.constants;

public class ArmConstants {
    public static final int baseCANID = 1;
    public static final int wristPWMchannel = 2;

    public static final double kp = 0;

    private static final double gearRatio = 1.0 / 192.0;
    public static final double positionFactor = 360 * gearRatio; // deg/rotations
    public static final double velocityFactor = 360 * gearRatio / 60; // (deg/sec)/RPM
}
