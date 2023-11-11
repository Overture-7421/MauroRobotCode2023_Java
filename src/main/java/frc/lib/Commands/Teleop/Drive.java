// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Commands.Teleop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandHelper;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;

import frc.lib.subsystems.SwerveChassis;
import frc.lib.math.Utils;

public class Drive extends CommandBase {
  /** Creates a new Drive. */
  public Drive(SwerveChassis swerveChassis, XboxController controller) : m_swerveChassis(swerveChassis), joystick(controller){
    AddRequirements(m_swerveChassis);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (joystick.GetRightBumper()) {
      kMaxSpeed = 2;
      kMaxAngularSpeed = 3.5;
    } else {
      kMaxSpeed = 5.0;
      kMaxAngularSpeed = 9.0;
    }

    meters_per_second_t xInput = Utils.applyAxisFilter(-joystick.GetLeftY())*kMaxSpeed;
    meters_per_second_t yInput = Utils.applyAxisFilter(-joystick.GetLeftX())*kMaxSpeed;
    radians_per_second_t rInput = Utils.applyAxisFilter(-joystick.GetRightX())*kMaxAngularSpeed;

    ChassisSpeeds chassisSpeeds = FromFieldRelativeSpeeds(
      xLimiter.Calculate(xInput),
      yLimiter.Calculate(yInput),
      rLimiter.Calculate(rInput),
      m_swerveChassis.getOdometry(),Rotation());

    m_swerveChassis.setSpeed(chassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  private SwerveChassis m_swerveChassis;

  private double kMaxSpeed = 5.0;
  private double kMaxAngularSpeed = 9.0;

  private meters_per_second_squared_t maxAcceleration = 12;
  private radians_per_second_squared_t maxRotation = 16;

  private SlewRateLimiter<meters_per_second> xLimiter = maxAcceleration;
  private SlewRateLimiter<meters_per_second> yLimiter = maxAcceleration;
  private SlewRateLimiter<radians_per_second> rLimiter = maxRotation;

  private XboxController joystick;




}
