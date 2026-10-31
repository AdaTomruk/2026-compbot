package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ShootingConstants;

public class ShootingCalculator {
    private static ShootingCalculator instance;

    // Filters to smooth the velocity of the mechanisms
    private final LinearFilter turretAngleFilter = LinearFilter.movingAverage((int) (0.1 / 0.02));
    private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage((int) (0.1 / 0.02));

    private Rotation2d lastTurretAngle;
    private double lastHoodAngle = Double.NaN;

    public static ShootingCalculator getInstance() {
        if (instance == null) instance = new ShootingCalculator();
        return instance;
    }

    /** * Container for the physical targets to feed to your subsystems.
     * All angles are already converted to degrees for easy implementation.
     */
    public record LaunchingParameters(
        boolean isValid,
        double turretAngleDegrees,
        double turretVelocityDegPerSec,
        double hoodAngleDegrees,
        double hoodVelocityDegPerSec,
        double flywheelRPM) {}

    private static final double minDistance = 1.45;
    private static final double maxDistance = 6.95;
    private static final double phaseDelay = 0.03;


    // Interpolation Maps
    private static final InterpolatingTreeMap<Double, Rotation2d> launchHoodAngleMap =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static final InterpolatingDoubleTreeMap launchFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    static {
        double offset_a = 20;

        // Distance (meters) -> Hood Angle
        launchHoodAngleMap.put(1.45, Rotation2d.fromDegrees(18.2));
        launchHoodAngleMap.put(1.95, Rotation2d.fromDegrees(18.2));
        launchHoodAngleMap.put(2.45, Rotation2d.fromDegrees(18.2));
        launchHoodAngleMap.put(2.95, Rotation2d.fromDegrees(18.2));
        launchHoodAngleMap.put(3.45, Rotation2d.fromDegrees(18.2));
        launchHoodAngleMap.put(4.95, Rotation2d.fromDegrees(25.0));
        launchHoodAngleMap.put(4.45, Rotation2d.fromDegrees(35.0));
        launchHoodAngleMap.put(4.95, Rotation2d.fromDegrees(45.0));
        launchHoodAngleMap.put(5.45, Rotation2d.fromDegrees(55.0));
        launchHoodAngleMap.put(5.95, Rotation2d.fromDegrees(60.0));
        launchHoodAngleMap.put(6.65, Rotation2d.fromDegrees(65.0));
        launchHoodAngleMap.put(6.95, Rotation2d.fromDegrees(68.0));

        // Distance (meters) -> Flywheel RPM
        launchFlywheelSpeedMap.put(1.45, 950.0+offset_a);
        launchFlywheelSpeedMap.put(1.95, 1020.0+offset_a);
        launchFlywheelSpeedMap.put(2.45, 1040.0+offset_a);
        launchFlywheelSpeedMap.put(2.95, 1100.0+offset_a);
        launchFlywheelSpeedMap.put(3.45, 1150.0+offset_a);
        launchFlywheelSpeedMap.put(3.95, 1200.0+offset_a);
        launchFlywheelSpeedMap.put(4.45, 1200.0+offset_a);
        launchFlywheelSpeedMap.put(4.95, 1280.0+offset_a);
        launchFlywheelSpeedMap.put(5.45, 1330.0+offset_a);
        launchFlywheelSpeedMap.put(5.95, 1390.0+offset_a);
        launchFlywheelSpeedMap.put(6.45, 1455.0+offset_a);
        launchFlywheelSpeedMap.put(6.95, 1528.0+offset_a);

        // Distance (meters) -> Time of flight (seconds)
        timeOfFlightMap.put(1.95, 1.1125);
        timeOfFlightMap.put(2.95, 1.0725);
        timeOfFlightMap.put(3.95, 1.27875);
        timeOfFlightMap.put(4.95, 1.32375);
    }

    /**
     * Calculates the exact parameters needed to shoot on the move.
     *
     * @param estimatedPose The robot's current pose on the field.
     * @param robotVelocity The robot-relative speeds.
     * @param fieldVelocity The field-relative speeds.
     * @param target The X/Y coordinates of the Hub/Target.
     */
    public LaunchingParameters calculate(
            Pose2d estimatedPose, 
            ChassisSpeeds robotVelocity, 
            ChassisSpeeds fieldVelocity,
            Translation2d target) {

        // 1. Account for phase delay (latency compensation)
        estimatedPose = estimatedPose.exp(
            new Twist2d(
                robotVelocity.vxMetersPerSecond * phaseDelay,
                robotVelocity.vyMetersPerSecond * phaseDelay,
                robotVelocity.omegaRadiansPerSecond * phaseDelay));

        // 2. Find Turret Pose (offset from robot center)
        Transform2d robotToTurret2d = new Transform2d(
            ShootingConstants.ROBOT_TO_TURRET.getTranslation().toTranslation2d(),
            ShootingConstants.ROBOT_TO_TURRET.getRotation().toRotation2d()
        );
        Pose2d turretPosition = estimatedPose.transformBy(robotToTurret2d);
        double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

        // 3. Imparted velocity calculations
        double robotAngle = estimatedPose.getRotation().getRadians();
        double turretVelocityX = fieldVelocity.vxMetersPerSecond
            + fieldVelocity.omegaRadiansPerSecond * (robotToTurret2d.getY() * Math.cos(robotAngle) - robotToTurret2d.getX() * Math.sin(robotAngle));
        double turretVelocityY = fieldVelocity.vyMetersPerSecond
            + fieldVelocity.omegaRadiansPerSecond * (robotToTurret2d.getX() * Math.cos(robotAngle) - robotToTurret2d.getY() * Math.sin(robotAngle));

        // 4. 20-iteration Lookahead Loop for Shooting on the Move
        double timeOfFlight;
        Pose2d lookaheadPose = turretPosition;
        double lookaheadTurretToTargetDistance = turretToTargetDistance;
        
        for (int i = 0; i < 20; i++) {
            // Clamp the distance to the boundaries of the timeOfFlightMap (1.38m to 5.68m)
            // This prevents a NullPointerException if the calculated distance goes out of bounds.
            double clampedDistance = Math.max(1.38, Math.min(5.68, lookaheadTurretToTargetDistance));
            
            timeOfFlight = timeOfFlightMap.get(clampedDistance);
            
            double offsetX = -turretVelocityX * timeOfFlight;
            double offsetY = -turretVelocityY * timeOfFlight;
            
            lookaheadPose = new Pose2d(
                turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                turretPosition.getRotation());
            
            lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
        }

    // 5. Final Hardware Target Conversions
    boolean isValid = lookaheadTurretToTargetDistance >= minDistance 
               && lookaheadTurretToTargetDistance <= maxDistance;
    double clampedDistance = Math.max(minDistance, Math.min(maxDistance, lookaheadTurretToTargetDistance));

    Rotation2d turretAngle = target.minus(lookaheadPose.getTranslation()).getAngle();
    double hoodAngleRadians = launchHoodAngleMap.get(clampedDistance).getRadians();
        
        if (lastTurretAngle == null) lastTurretAngle = turretAngle;
        if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngleRadians;

        double calculatedTurretVel = turretAngleFilter.calculate(
            turretAngle.minus(lastTurretAngle).getDegrees() / 0.02);
        double calculatedHoodVel = hoodAngleFilter.calculate(
            Math.toDegrees(hoodAngleRadians - lastHoodAngle) / 0.02);

        lastTurretAngle = turretAngle;
        lastHoodAngle = hoodAngleRadians;

        // Telemetry
        SmartDashboard.putNumber("LaunchCalculator/LookaheadDist", lookaheadTurretToTargetDistance);
        SmartDashboard.putBoolean("LaunchCalculator/IsValidTarget", isValid);

        return new LaunchingParameters(
            isValid,
            turretAngle.getDegrees(),
            calculatedTurretVel,
            Math.toDegrees(hoodAngleRadians),
            calculatedHoodVel,
            launchFlywheelSpeedMap.get(clampedDistance)
        );
    }
}