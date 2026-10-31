// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.ADXL345_I2C.AllAxes;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.VisionConstants;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final Field2d rawVisionField = new Field2d();

    private final RobotContainer m_robotContainer;

    private boolean hasInitializedHeading = false;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    private final boolean kUseLimelight = true;
    private boolean m_seededInAutonomous = false;

    public Robot() {
        DataLogManager.start();
        m_robotContainer = new RobotContainer();

        SmartDashboard.putData("Vision/Raw Vision Field", rawVisionField);
    }

    @Override
    public void robotPeriodic() {
        initializeHeadingOnce();

        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        if (kUseLimelight) {
            var driveState = m_robotContainer.drivetrain.getState();
            double headingDeg = driveState.Pose.getRotation().getDegrees();
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

            // 1. Get the current physical angle of the turret
            double turretAngleRads = Math.toRadians(m_robotContainer.turret.getMechanismDegrees());

            // 2. Chain transforms: Robot Origin -> Turret Base -> Turret Spin -> Camera Lens
            Pose3d robotOrigin = new Pose3d();
            Pose3d dynamicCameraPose = robotOrigin
                .transformBy(ShootingConstants.ROBOT_TO_TURRET)
                .transformBy(new Transform3d(new Translation3d(), new Rotation3d(0, 0, turretAngleRads)))
                .transformBy(VisionConstants.TURRET_TO_CAMERA);

            // 3. Send the updated Camera-to-Robot transform to the Limelight
            LimelightHelpers.setCameraPose_RobotSpace(
                VisionConstants.LIMELIGHT_NAME, 
                dynamicCameraPose.getX(), 
                dynamicCameraPose.getY(), 
                dynamicCameraPose.getZ(), 
                Units.radiansToDegrees(dynamicCameraPose.getRotation().getX()), // Roll
                Units.radiansToDegrees(dynamicCameraPose.getRotation().getY()), // Pitch
                Units.radiansToDegrees(dynamicCameraPose.getRotation().getZ())  // Yaw
            );

            // 4. Send the Robot Heading to the Limelight (Required for MegaTag2)
            LimelightHelpers.SetRobotOrientation(
                VisionConstants.LIMELIGHT_NAME, 
                headingDeg, 
                Units.radiansToDegrees(driveState.Speeds.omegaRadiansPerSecond), 
                0, 0, 0, 0
            );
            
            // 5. Fetch the MegaTag2 Pose Estimate
            var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LIMELIGHT_NAME);
            
            // 6. Validate and apply dynamic standard deviations
            if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
                rawVisionField.setRobotPose(llMeasurement.pose);    
                // Base trust values (lower = trust vision more)
                double xyStdDev = 0.05; 
                
                // Increase standard deviation (trust vision less) as distance increases
                xyStdDev += Math.pow(llMeasurement.avgTagDist, 2.0) * 0.1;
                
                // Trust vision much more if we see multiple tags
                if (llMeasurement.tagCount > 1) {
                    xyStdDev *= 0.2;
                }

                // Push to the swerve pose estimator
                m_robotContainer.drivetrain.addVisionMeasurement(
                    llMeasurement.pose, 
                    llMeasurement.timestampSeconds,
                    VecBuilder.fill(xyStdDev, xyStdDev, 9999999.0) // 9999999.0 ignores vision rotation, perfectly trusting the Pigeon2
                );
            }}
        }

    @Override
    public void disabledInit() {
        m_seededInAutonomous = false;
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {

        m_robotContainer.turret.seedMotorFromCRT();
        m_seededInAutonomous = true;
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        if (!m_seededInAutonomous) {
            m_robotContainer.turret.seedMotorFromCRT();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    private void initializeHeadingOnce(){
        if (hasInitializedHeading) {
            return;
        }

        var alliance = DriverStation.getAlliance();

        if(alliance.isPresent() == false){
            return;
        }

        if(alliance.get() == DriverStation.Alliance.Red){
            m_robotContainer.drivetrain.seedFieldCentric(Rotation2d.fromDegrees(180));
        }
        else{
            m_robotContainer.drivetrain.seedFieldCentric(Rotation2d.fromDegrees(0));
        }

        hasInitializedHeading = true;
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
