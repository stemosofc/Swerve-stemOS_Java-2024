// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Dimensoes;
import frc.robot.Constants.Tracao;
import frc.robot.commands.Auto.ConfigAuto;
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

    // Objeto global autônomo
    ConfigAuto autonomo;

    // Método construtor da classe
    public SwerveSubsystem(File directory) {
        // Seta a telemetria como nível mais alto
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        // Acessa os arquivos do diretório .JSON
        try {
          
        swerveDrive = new SwerveParser(directory).createSwerveDrive(Tracao.MAX_SPEED, 
        Dimensoes.angleConversion, Dimensoes.driveConversion);
       
        } catch (Exception e) {
          throw new RuntimeException(e);
        }

        autonomo = new ConfigAuto(this);
        
        autonomo.setupPathPlanner();
    }
    
    @Override
    public void periodic() {
      // Dentro da função periódica atualizamos nossa odometria
      swerveDrive.updateOdometry();
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

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput) {
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

  public ChassisSpeeds discretize(ChassisSpeeds speeds) {
    var desiredDeltaPose = new Pose2d(
      speeds.vxMetersPerSecond * Tracao.dt, 
      speeds.vyMetersPerSecond * Tracao.dt, 
      new Rotation2d(speeds.omegaRadiansPerSecond * Tracao.dt * Tracao.constantRotation)
    );
    var twist = new Pose2d().log(desiredDeltaPose);

    return new ChassisSpeeds((twist.dx / Tracao.dt), (twist.dy / Tracao.dt), (speeds.omegaRadiansPerSecond));
  }


    public Command getAutonomousCommand(String pathName, boolean setOdomToStart)
  {
    
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }
}
