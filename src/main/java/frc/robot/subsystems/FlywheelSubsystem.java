package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Flywheel shooter subsystem.
 * Controls dual Kraken X60 motors for shooting game pieces.
 * Left motor is the leader; right motor follows automatically.
 */
public class FlywheelSubsystem extends SubsystemBase {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final TalonFX leftFlywheelMotor;
    private final TalonFX rightFlywheelMotor;

    // ── Control Requests ──────────────────────────────────────────────────────
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(false);
    private final NeutralOut neutralRequest = new NeutralOut();
    private final Follower followerRequest = new Follower(LEFT_FLYWHEEL_MOTOR_ID, MotorAlignmentValue.Opposed);

    // ── State Tracking ────────────────────────────────────────────────────────
    private double targetRPM = 0.0;

    public FlywheelSubsystem() {
        // ── Motor Initialization ──────────────────────────────────────────────
        leftFlywheelMotor = new TalonFX(LEFT_FLYWHEEL_MOTOR_ID, CANBUS);
        rightFlywheelMotor = new TalonFX(RIGHT_FLYWHEEL_MOTOR_ID, CANBUS);

        configureMotors();

        // Set right motor to follow the left (leader) motor
        rightFlywheelMotor.setControl(followerRequest);
        SmartDashboard.putNumber("shooter", 1000);

    }

    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // ── 1. Motor Output ───────────────────────────────────────────────────
        config.MotorOutput
            .withInverted(LEFT_FLYWHEEL_INVERTED ? 
                InvertedValue.Clockwise_Positive : 
                InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);

        // ── 2. Current Limits ─────────────────────────────────────────────────
        config.CurrentLimits
            .withSupplyCurrentLimit(FLYWHEEL_SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(FLYWHEEL_STATOR_CURRENT_LIMIT)
            .withStatorCurrentLimitEnable(true);

        // ── 3. PID & Feedforward (Slot 0) ─────────────────────────────────────
        config.Slot0
            .withKP(FLYWHEEL_kP)
            .withKI(FLYWHEEL_kI)
            .withKD(FLYWHEEL_kD)
            .withKS(FLYWHEEL_kS)
            .withKV(FLYWHEEL_kV)
            .withKA(FLYWHEEL_kA);

        // ── 4. Sensor Gearing ─────────────────────────────────────────────────
        config.Feedback
            .withSensorToMechanismRatio(FLYWHEEL_GEAR_RATIO);

        // ── 5. Apply Configurations ───────────────────────────────────────────
        // We apply the exact same base config to both motors. 
        // The Follower request (set to Opposed) handles the right motor's direction.
        leftFlywheelMotor.getConfigurator().apply(config);
        rightFlywheelMotor.getConfigurator().apply(config);
    }

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // Get current speeds
        double leftRPS = leftFlywheelMotor.getVelocity().getValueAsDouble();
        double rightRPS = rightFlywheelMotor.getVelocity().getValueAsDouble();
        double leftRPM = leftRPS * 60.0;
        double rightRPM = rightRPS * 60.0;
        double avgRPM = (leftRPM + rightRPM) / 2.0;

        // SmartDashboard telemetry
        SmartDashboard.putNumber("Flywheel/Left_RPM", leftRPM);
        SmartDashboard.putNumber("Flywheel/Right_RPM", rightRPM);
        SmartDashboard.putNumber("Flywheel/Average_RPM", avgRPM);
        SmartDashboard.putNumber("Flywheel/Target_RPM", targetRPM);
        SmartDashboard.putNumber("Flywheel/RPM_Error", targetRPM - avgRPM);

        SmartDashboard.putNumber("Flywheel/Left_SupplyCurrent_A",
            leftFlywheelMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Right_SupplyCurrent_A",
            rightFlywheelMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Left_StatorCurrent_A",
            leftFlywheelMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Right_StatorCurrent_A",
            rightFlywheelMotor.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putBoolean("Flywheel/At_Speed", atSpeed(targetRPM));
        SmartDashboard.putBoolean("Flywheel/At_Shoot_Speed", atShootSpeed());
    }

    // ── Flywheel Control ──────────────────────────────────────────────────────

    /** Sets flywheel to standard shooting speed. */
    public void shoot() {
        double a = SmartDashboard.getNumber("shooter", 1000);
        setRPM(a);
    }

    /** Sets flywheel to warm-up speed (75% of shooting speed). */
    public void warmUp() {
        setRPM(WARM_UP_RPM);
    }

    /** Sets flywheel to a specific RPM. */
    public void setRPM(double rpm) {
        targetRPM = rpm;
        double rps = rpm / 60.0;
        leftFlywheelMotor.setControl(velocityRequest.withVelocity(rps));
        // rightFlywheelMotor follows automatically
    }

    /** Stops the flywheel. */
    public void stop() {
        targetRPM = 0.0;
        leftFlywheelMotor.setControl(neutralRequest);
        // rightFlywheelMotor coasts with the leader
    }

    // ── Getters ───────────────────────────────────────────────────────────────

    /** @return Current flywheel speed in RPM (average of both motors). */
    public double getRPM() {
        double leftRPS = leftFlywheelMotor.getVelocity().getValueAsDouble();
        double rightRPS = rightFlywheelMotor.getVelocity().getValueAsDouble();
        return ((leftRPS + rightRPS) / 2.0) * 60.0;
    }

    /** @return true when flywheel is at target speed within tolerance. */
    public boolean atSpeed(double targetRPM) {
        return Math.abs(getRPM() - targetRPM) <= FLYWHEEL_AT_SPEED_TOLERANCE_RPM;
    }

    /** @return true when flywheel is at standard shooting speed. */
    public boolean atShootSpeed() {
        
        if (getRPM() < (targetRPM/100)*90){
            return true;
        }
        return false;

    }
}