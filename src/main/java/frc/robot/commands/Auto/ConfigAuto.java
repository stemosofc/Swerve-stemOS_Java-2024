// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class ConfigAuto {

    SwerveSubsystem swerve;

    public ConfigAuto(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

     public void setupPathPlanner()
    {
        AutoBuilder.configureHolonomic(
            swerve::getPose, // Robot pose supplier
            swerve::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            swerve::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            swerve::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                                         new PIDConstants(5.0, 0.0, 0.0),
                                         // Translation PID constants
                                         new PIDConstants(swerve.getSwerveController().config.headingPIDF.p,
                                                          swerve.getSwerveController().config.headingPIDF.i,
                                                          swerve.getSwerveController().config.headingPIDF.d),
                                         // Rotation PID constants
                                         4.5,
                                         // Max module speed, in m/s
                                         swerve.getSwerveDriveConfiguration().getDriveBaseRadiusMeters(),
                                         // Drive base radius in meters. Distance from robot center to furthest module.
                                         new ReplanningConfig()
                                         // Default path replanning config. See the API for the options here
            ),
            () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
                },
            swerve // Reference to this subsystem to set requirements
                                  );
    }

}
