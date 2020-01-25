package frc.robot;

/** Class for mathematic functions */
public class FieldConstants {

    /** Heights of vision camera and targets, ien inches */
    public static class VisionConstants {
        static final double targetHeight = 21;
        static final double cameraHeight = 46.5;
        static final double mountingAngle = 15;

        public static double getTargetheight() {
            return targetHeight;
        }

        public static double getCameraheight() {
            return cameraHeight;
        }

        public static double getMountingAngle() {
            return mountingAngle;
        }
    }

    public static class GamePieces {
        static final double ballRadius = 3.5; //in inches

        public static double getBallradius() {
            return ballRadius;
        }
    }

    public static double hangHeight = 5;

}