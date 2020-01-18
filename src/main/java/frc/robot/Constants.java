package frc.robot;

/**
 * Class containing numbers used throughout the project - PID values, as well as mathematic constants
 */
public final class Constants {

  public static class CTRE_PID {
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;
    public static int kIzone = 100;
    public static int kPIDLoopIdx = 0;
    public static int kTimeoutMs = 30;
  }
  public static class robotGeometry {
    public static double wheelCircumference = 6; // Inches
    public static double ticksPerRev = 4096;
  }
}