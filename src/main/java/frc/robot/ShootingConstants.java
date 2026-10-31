package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public class ShootingConstants {
    // Physical offset of the turret from the center of the robot
    public static final Transform3d ROBOT_TO_TURRET = new Transform3d(-0.19685, 0.0, 0.44, Rotation3d.kZero);
    
    // Position of the target on the field (Update these to the 2026 field map locations!)
    public static final Translation2d BLUE_TARGET = new Translation2d(4.5, 4.0); 
    public static final Translation2d RED_TARGET = new Translation2d(11.9, 4);

    // Kenar/Köşe (Side) hedefleri - Koordinatları sahanıza göre ince ayar yapabilirsiniz
    public static final Translation2d BLUE_UPPER_TARGET = new Translation2d(1.5, 7.0); 
    public static final Translation2d BLUE_LOWER_TARGET = new Translation2d(1.5, 1.0); 
    public static final Translation2d RED_UPPER_TARGET = new Translation2d(15.0, 7.0); 
    public static final Translation2d RED_LOWER_TARGET = new Translation2d(15.0, 1.0); 
}
