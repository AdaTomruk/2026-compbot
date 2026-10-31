package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShootingConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.FeederSubsystem; // The new NEO funnel subsystem
import frc.robot.subsystems.ShootingCalculator;
import frc.robot.subsystems.ShootingCalculator.LaunchingParameters;

public class AutoAimAndShootCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final TurretSubsystem turret;
    private final HoodSubsystem hood;
    private final FlywheelSubsystem flywheel;
    private final ShootingCalculator calculator;

    /**
     * Fully automated shooting command.
     * Calculates "Shoot on the Move" physics and feeds the ball when ready.
     */
    public AutoAimAndShootCommand(
        CommandSwerveDrivetrain drivetrain, 
        TurretSubsystem turret, 
        HoodSubsystem hood, 
        FlywheelSubsystem flywheel
    ) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.hood = hood;
        this.flywheel = flywheel;
        this.calculator = ShootingCalculator.getInstance();
        
        // Ensure no other commands use these subsystems while aiming
        addRequirements(turret, hood, flywheel);
    }

    @Override
    public void initialize() {
        // Optional: Ensure PID is enabled for the hood when the command starts
        hood.enablePID();
        hood.disableDashboardControl();
    }

    @Override
    public void execute() {
        if (!turret.isCRTSeeded()) {
            flywheel.warmUp();
            return;
        }
        // 1. Get current robot state from Phoenix 6 Swerve
        Pose2d pose = drivetrain.getState().Pose;
        ChassisSpeeds robotSpeeds = drivetrain.getState().Speeds;
        
        // 2. Convert to field-relative speeds for the calculator
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, pose.getRotation());

        // 3. Determine target based on alliance color
        Translation2d target = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red 
                                ? ShootingConstants.RED_TARGET 
                                : ShootingConstants.BLUE_TARGET;

        // 4. Run the physics calculation
        LaunchingParameters params = calculator.calculate(pose, robotSpeeds, fieldSpeeds, target);
        

        if (params.isValid()) {
            // 5. Update mechanism targets based on distance
            turret.setFieldHeading(params.turretAngleDegrees());
            hood.setAngleFromAuto(params.hoodAngleDegrees());
            flywheel.setRPM(params.flywheelRPM());

            // 6. Check if we are "locked on" to the target
            boolean turretReady = turret.atTarget(); //
            boolean hoodReady = hood.atTarget();     //
            boolean flywheelReady = flywheel.atSpeed(params.flywheelRPM()); //

            // 7. Fire the ball through the funnel if all conditions are met
            if (turretReady && hoodReady && flywheelReady) {
            } else {
            }
        } else {
            // If the robot is out of range, stop the feeder and spin flywheels to a neutral speed
            flywheel.warmUp(); //
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Clean up when the button is released
        flywheel.stop(); // or flywheel.warmUp() if you want to stay ready
        turret.stop();   //
        hood.enableDashboardControl();
    }
}