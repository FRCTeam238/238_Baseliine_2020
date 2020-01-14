package frc.robot;

/** Class for mathematic functions */
public class Trig238 {

    /** @return linear distance (inches) from camera to target 
    * @param height (inches) between camera and center of target
    * @param angle (degrees) formed between camera and center of target
    */
    public static double calculateDistance(double height, double angle){
        double radians = Math.toRadians(angle);
        double distance = height / radians;
        return distance;
    }
}