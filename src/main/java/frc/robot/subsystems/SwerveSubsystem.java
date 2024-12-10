// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Tracao;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

/** 
 * Classe de subsistema onde fazemos a ponte do nosso código para YAGSL
 */
public class SwerveSubsystem extends SubsystemBase {
    // Objeto global da SwerveDrive (Classe YAGSL)
    SwerveDrive swerveDrive;

    // Método construtor da classe
    public SwerveSubsystem(File directory) {
        // Seta a telemetria como nível mais alto
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        // Acessa os arquivos do diretório .JSON
        try {
        swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.Dimensoes.MAX_SPEED);
        } catch (Exception e) {
          throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(true);
        setupPathPlanner();
    }
    
    @Override
    public void periodic() {
      // Dentro da função periódica atualizamos nossa odometria
      swerveDrive.updateOdometry();
    }

    public void setupPathPlanner()
    {
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                                         new PIDConstants(5.0, 0.0, 0.0),
                                         // Translation PID constants
                                         new PIDConstants(5, 0, 0),
                                         // Rotation PID constants
                                         4.5,
                                         // Max module speed, in m/s
                                         swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
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
            this // Reference to this subsystem to set requirements
                                  );
    }

  //Movimenta o robô com o joystick esquerdo, e mira o robo no ângulo no qual o joystick está apontando
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 3); 
      double yInput = Math.pow(translationY.getAsDouble(), 3); 
      // Faz o robô se mover
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getYaw().getRadians(),
                                                                      swerveDrive.getMaximumVelocity()));
    });
  }

  //Movimenta o robô com o joystick esquerdo, e gira o robô na intensidade na qual o joystick direito está para o lado
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 3); 
      double yInput = Math.pow(translationY.getAsDouble(), 3); 
      // Faz o robô se mover
      swerveDrive.drive(new Translation2d(xInput * swerveDrive.getMaximumVelocity(),
                                          yInput * swerveDrive.getMaximumVelocity()),
                        angularRotationX.getAsDouble() * swerveDrive.getMaximumAngularVelocity(),
                        true,
                        false);
    });
  }

    public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

    // Função drive que chamamos em nossa classe de comando Teleoperado
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) 
    {
      swerveDrive.drive(translation, rotation, fieldRelative, false);
    }

    // Função para obter a velocidade desejada a partir dos inputs do gamepad
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
    {
      return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, 
      getHeading().getRadians());
    }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput) 
  {
    return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, 0, 0, 0, Tracao.MAX_SPEED);
  }

  // Função que retorna a posição do robô (translação e ângulo), (Usado no autônomo)
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }
  
  // Retorna a velocidade relativa ao campo
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  // Retorna a configuração do swerve
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  // Retorna o objeto de controle, o qual é usado para acessar as velocidades máximas por exemplo
  public SwerveController getSwerveController() {
    return swerveDrive.getSwerveController();
  }

  // Ângulo atual do robô
  public Rotation2d getHeading() {
    return swerveDrive.getYaw();
  }

  // Reseta a odometria para uma posição indicada (Usado no autônomo)
  public void resetOdometry(Pose2d posicao) {
    swerveDrive.resetOdometry(posicao);
  }

  // Seta a velocidade do chassi (Usado no autônomo)
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

    public Command getAutonomousCommand(String pathName, boolean setOdomToStart)
  {
    
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }
}
