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
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShootingCalculator;
import frc.robot.subsystems.ShootingCalculator.LaunchingParameters;

public class AutoAimSideCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final TurretSubsystem turret;
    private final HoodSubsystem hood;
    private final FlywheelSubsystem flywheel;
    private final ShootingCalculator calculator;

    // Sahanın Y eksenindeki orta noktası (PathPlanner navgrid'e göre saha genişliği 8.07m)
    private static final double FIELD_MIDPOINT_Y = 4.035;

    public AutoAimSideCommand(
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
        
        addRequirements(turret, hood, flywheel);
    }

    @Override
    public void initialize() {
        hood.enablePID();
        hood.disableDashboardControl();
    }

    @Override
    public void execute() {
        if (!turret.isCRTSeeded()) {
            flywheel.warmUp();
            return;
        }

        Pose2d pose = drivetrain.getState().Pose;
        ChassisSpeeds robotSpeeds = drivetrain.getState().Speeds;
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, pose.getRotation());

        // Hangi ittifaktayız ve sahanın üst yarısında mıyız?
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        boolean isUpperHalf = pose.getY() > FIELD_MIDPOINT_Y;

        // Y pozisyonuna göre hedef seçimi
        Translation2d target;
        if (isRed) {
            target = isUpperHalf ? ShootingConstants.RED_UPPER_TARGET : ShootingConstants.RED_LOWER_TARGET;
        } else {
            target = isUpperHalf ? ShootingConstants.BLUE_UPPER_TARGET : ShootingConstants.BLUE_LOWER_TARGET;
        }

        // Seçilen hedefe göre hesaplama yap (Var olan tabloyu kullanır)
        LaunchingParameters params = calculator.calculate(pose, robotSpeeds, fieldSpeeds, target);
        
        if (params.isValid()) {
            turret.setFieldHeading(params.turretAngleDegrees());
            hood.setAngleFromAuto(params.hoodAngleDegrees());
            flywheel.setRPM(1400);

            boolean turretReady = turret.atTarget();
            boolean hoodReady = hood.atTarget();
            boolean flywheelReady = flywheel.atSpeed(1400);

            if (turretReady && hoodReady && flywheelReady) {
            } else {
            }
        } else {
            flywheel.warmUp();
        }
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
        turret.stop();
        hood.enableDashboardControl();
    }
}