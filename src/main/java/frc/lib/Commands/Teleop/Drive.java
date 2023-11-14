// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.filter.SlewRateLimiter;

import frc.lib.math.Utils;
import frc.lib.subsystems.SwerveChassis;

public class Drive extends Command {
  
  private SwerveChassis m_swerveChassis;
  private XboxController m_controller;

  /* Speed Constants*/
  double kMaxSpeed = 5.0; // meters per second
  double kMaxAngularSpeed = 4.0; // radians per second

  /* Constant */
  double maxAcceleration = 12; // meters per second squared
  double maxRotation = 9; // radians per second squared

  /* Limiters */
  SlewRateLimiter xLimiter = new SlewRateLimiter(maxAcceleration);
  SlewRateLimiter yLimiter = new SlewRateLimiter(maxAcceleration);
  SlewRateLimiter rLimiter = new SlewRateLimiter(maxRotation);

  /** Creates a new Drive. */
  public Drive(SwerveChassis swerveChassis, XboxController controller) {
    this.m_swerveChassis = swerveChassis;
    this.m_controller = controller;
    
    addRequirements(m_swerveChassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_controller.getRightBumper()) {
      kMaxSpeed = 2;
      kMaxAngularSpeed = 2;
      // maxAcceleration = 2.5;
      // maxRotation = 6.33;
    } else {
      kMaxSpeed = 5.0;
      kMaxAngularSpeed = 4.0;
      // maxAcceleration = 12.0;
      // maxRotation = 16.0;
    }

    double xInput = Utils.applyAxisFilter(-m_controller.getLeftY()) * kMaxSpeed; // Meter per second
    double yInput = Utils.applyAxisFilter(m_controller.getLeftX()) * kMaxSpeed; // Meter per second
    double rInput = Utils.applyAxisFilter(m_controller.getRightX()) * kMaxAngularSpeed; // Radians per second

    ChassisSpeeds speeds = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
      xLimiter.calculate(xInput),
      yLimiter.calculate(yInput),
      rLimiter.calculate(rInput),
      m_swerveChassis.getOdometry().getRotation()),
      0.02);

    m_swerveChassis.setSpeed(speeds);
  }

  // Called once the command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end 
  @Override
  public boolean isFinished() {
    return false;
  }

}
