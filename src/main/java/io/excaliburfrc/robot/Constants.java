package io.excaliburfrc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // add a inner class `public static final class` for each subsystem.
  // constants should be declared as `public static final double`
  // constant names should be in `SCREAMING_SNAKE_CASE` or `kUpperCamelCase`
  public static final class DriveConstants {
    public static final int RIGHT_LEADER_ID = 11;
    public static final int RIGHT_FOLLOWER_ID = 12;
    public static final int LEFT_LEADER_ID = 13;
    public static final int LEFT_FOLLOWER_ID = 14;
    public static final double TRACK_WIDTH = 0.7047364141920852;
    public static final double WHEEL_RADIUS = Units.inchesToMeters(6);
    public static final double GEARING = 0.09425070688030161; // 1.0 / 10.71
    public static final double kV_lin = 2.66, kA_lin = 0.433, kV_ang = 2.76, kA_ang = -0.236;
    public static final double PULSE_TO_METER = GEARING * WHEEL_RADIUS * Math.PI;
    //        1 / 23.5; // (CPR * GEARING) / (WHEEL_RADIUS * Math.PI);
    public static final double kP = 0.0387;
    public static final double kS = 0.129;
    public static final double kP_ang = 0.04, kS_ang = 0.023749999999999997;
    public static final double ANGLE_TOLERANCE = 3;
  }
}
