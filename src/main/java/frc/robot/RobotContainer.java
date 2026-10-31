package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.generated.TunerConstants;

import frc.robot.commands.AutoAimAndShootCommand; // Import your new command
import frc.robot.commands.AutoAimSideCommand;
import frc.robot.commands.AutoAimTurretCommand;

import frc.robot.commands.AutoAimSideCommand; // İçe aktarmayı unutmayın

import com.pathplanner.lib.auto.NamedCommands;


public class RobotContainer {

    // --- Speeds ---
    private final double MaxSpeed       = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // --- Swerve ---
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // --- Controller ---
    private final CommandXboxController joystick = new CommandXboxController(0);

    // --- Subsystems ---
    public  final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final TurretSubsystem         turret     = new TurretSubsystem(
        () -> drivetrain.getState().Pose.getRotation(),
        () -> drivetrain.getState().Pose
    );
    public final IntakeSubsystem         intake     = new IntakeSubsystem();
    public final IntakeArmSubsystem      intakeArm  = new IntakeArmSubsystem();
    public final FlywheelSubsystem       flywheel   = new FlywheelSubsystem();
    public final HoodSubsystem           hood       = new HoodSubsystem();
    public final FeederSubsystem         feeder     = new FeederSubsystem();

    // --- Auto ---
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        SmartDashboard.putNumber("Turret Target Angle (deg)", 0.0);
        SmartDashboard.putNumber("Hood Target Angle (deg)", Constants.HoodConstants.HOOD_MIN_ANGLE_DEG);
        
        // =========================================================================
        // OTONOM İÇİN PATHPLANNER NAMED COMMANDS (İSİMLENDİRİLMİŞ KOMUTLAR)
        // =========================================================================
        
        // 1. Atış Komutu (Sonsuza kadar takılmaması için 2.5 saniye sınır koyuyoruz)
        NamedCommands.registerCommand("Shoot", 
            new AutoAimAndShootCommand(drivetrain, turret, hood, flywheel).withTimeout(6)
        );

        // 2. Intake Kolunu Aç ve Kapat
        NamedCommands.registerCommand("IntakeArmOpen", Commands.runOnce(intakeArm::openArm, intakeArm));
        NamedCommands.registerCommand("IntakeArmClose", Commands.runOnce(intakeArm::closeArm, intakeArm));

        // 3. Intake Roller'ları Çalıştır ve Durdur
        NamedCommands.registerCommand("IntakeRollersOn", Commands.runOnce(intake::intake, intake));
        NamedCommands.registerCommand("IntakeRollersOff", Commands.runOnce(intake::stopRollers, intake));
        
        // =========================================================================
        autoChooser = AutoBuilder.buildAutoChooser("baran auto");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {
        

        // --- Drivetrain Default ---
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                     .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                     .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> new SwerveRequest.Idle()).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);

        //configureDriveBindings();
        configureTurretBindings();
        configureIntakeBindings();
        configureFeederBindings();
        configureShooterBindings();

        joystick.rightTrigger().whileTrue(
            new AutoAimAndShootCommand(drivetrain, turret, hood, flywheel)
        );


        joystick.povUp().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    }


    private void configureTurretBindings() {
        // Go to SmartDashboard angle
        joystick.x().onTrue(Commands.runOnce(() -> {
            double angle = SmartDashboard.getNumber("Turret Target Angle (deg)", 180);
            turret.setFieldHeading(angle);
        }, turret));

        // Stop turret
        joystick.back().onTrue(Commands.runOnce(turret::stop, turret));
    }


    private void configureIntakeBindings() {
        // Toggle intake arm open/close
        joystick.y().onTrue(Commands.runOnce(intakeArm::toggleArm, intakeArm));

        // Run rollers in (hold)
        joystick.rightBumper()
            .whileTrue(Commands.run(intake::intake, intake));
        
        joystick.leftBumper()
            .whileTrue(Commands.run(intake::stopRollers, intake));

        joystick.leftTrigger()
            .whileTrue(Commands.run(intake::feedToShooter, intake));

        joystick.leftTrigger().onFalse(Commands.runOnce(intake::stopFeedeToShooter, intake));

        joystick.povDown().whileTrue(Commands.run(() -> intakeArm.adjustTargetAngle(0.5), intakeArm));
        joystick.povLeft().whileTrue(Commands.run(() -> intakeArm.adjustTargetAngle(-0.5), intakeArm));

        
    }

    private void configureFeederBindings() {
        // Run feeder forward (hold)
        joystick.start()
            .whileTrue(Commands.run(feeder::runFeeder, feeder))
            .onFalse(Commands.runOnce(feeder::stop, feeder));

        joystick.leftTrigger()
            .whileTrue(Commands.run(feeder::reverseFeeder, feeder));

        joystick.leftTrigger().onFalse(Commands.runOnce(feeder::stop, feeder));


    }

    private void configureShooterBindings() {
    // ── Hood ──────────────────────────────────────────────────────────────

    // Set hood to SmartDashboard angle


}

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}