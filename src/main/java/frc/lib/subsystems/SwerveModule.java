// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.enums.NeutralMode.ControllerNeutralMode;
import frc.lib.motorcontrollers.OverTalonFX;
import frc.lib.sensors.OverCANCoder;

public class SwerveModule extends SubsystemBase {
  //Declaration of motors and sensors
  private OverTalonFX m_driveMotor;
  private OverTalonFX m_turningMotor;
  private OverCANCoder m_CanCoder;

  // Feedforward and Voltage
  private SimpleMotorFeedforward m_FeedForward;

  //State
  private SwerveModuleState m_State = new SwerveModuleState();

  //Ratios
  private double m_wheelDiameter = 1;
  private double m_driveMotorGearRatio = 1;
  private double m_turningMotorGearRatio = 1;

  // Name
  private String m_name;

  /**
   * Swerve Module Constructor
   * 
   * @param rotatorID - ID of the motor that rotates the wheel
   * @param wheelID - ID of the motor that drives the wheel
   * @param canCoderID - ID of the canCoder that is attached to the rotator
   * @param offset - Offset of the canCoder
   * @param moduleName - Name of the module
   * @param canBus - Can Bus of the module
   */
  public SwerveModule(int rotatorID, int wheelID, int canCoderID, double offset, String moduleName, String canBus) {
    m_driveMotor = new OverTalonFX(wheelID, ControllerNeutralMode.BRAKE, true, canBus);
    m_turningMotor = new OverTalonFX(rotatorID, ControllerNeutralMode.BRAKE, true, canBus);
    m_CanCoder = new OverCANCoder(canCoderID, offset, canBus);

    m_turningMotor.setContinousWrap();
    m_turningMotor.setFusedCANCoder(canCoderID);
    m_turningMotor.setClosedLoopVoltageRamp(0.1);
    m_turningMotor.SetSupplyCurrentLimit(true, 20, 30, 0.5);
    m_turningMotor.setPositionVoltage(0, false);

    m_driveMotor.zeroPosition();
    // m_driveMotor.setClosedLoopTorqueRamp(0.1);
    // m_driveMotor.setTorqueCurrentLimit(40, -40, 0.1);
    m_driveMotor.setClosedLoopVoltageRamp(0.1);
    m_driveMotor.SetSupplyCurrentLimit(true, 20, 30, 0.5);

    m_name = moduleName;
    m_name = m_name.trim();
  }

  /**
   * Gets the Feedforward
   * 
   * @return - Feedforward
   */
  public SimpleMotorFeedforward get_FeedForward() {
    return m_FeedForward;
  }

  /**
   * Sets Rotator Motor PID Values
   * 
   * @param kP - Proportional Value
   * @param kI - Integral Value
   * @param kD - Derivative Value
   */
  public void setRotatorPIDValues(double kP, double kI, double kD) {
    m_turningMotor.setPIDValues(kP, kI, kD, 0, 0);

  };

  /**
   * Sets Drive Motor PID Values
   * 
   * @param kP - Proportional Value
   * @param kI - Integral Value
   * @param kD - Derivative Value
   */
  public void setDrivePIDValues(double kP, double kI, double kD) {
    m_driveMotor.setPIDValues(kP, kI, kD, 0, 0);
  };

  /**
   * Sets FeedForward Values
   * 
   * @param ks - Static Value
   * @param kv - Velocity Value
   * @param ka - Acceleration Value
   */
  public void setFFConstants(double ks, double kv, double ka) {
    m_FeedForward = new SimpleMotorFeedforward(ks, kv, ka);
  }

  /**
   * Sets the gear ratio of both motors
   * 
   * @param turn
   * @param wheel
   */
  public void setGearRatio(double turn, double wheel) {
    m_turningMotorGearRatio = turn;
    m_driveMotorGearRatio = wheel;
    m_turningMotor.setRotorToSensorRatio(m_turningMotorGearRatio);
    m_driveMotor.setRotorToSensorRatio(m_driveMotorGearRatio);
  }

  /**
   * Sets the wheel diameter
   * 
   * @param wheelDiameter - Diameter of the wheel
   */
  public void setWheelDiameter(double wheelDiameter) {
    m_wheelDiameter = wheelDiameter;
  }

  /**
   * Gets the speed of the wheel
   * 
   * @return - Speed of the wheel
   */
  public double getSpeed() {
    return m_driveMotor.getVelocity(m_wheelDiameter, m_driveMotorGearRatio);
  }

  /**
   * Set Module Speed
   * 
   * @param speed - Speed of the wheel
   */
  public double setSpeed(double speed) {
    return ((speed / (m_wheelDiameter * Math.PI)));
    // return 2.4 * speed;
  }

  /**
   * Gets the distance traveled by the module
   * 
   * @return - Distance traveled by the module
   */
  public double getDistance() {
    return m_driveMotor.getDistance(m_wheelDiameter, m_driveMotorGearRatio);
  }

  /**
   * Gets the angle of the module
   * 
   * @return - Angle of the module
   */
  public double getAngle() {
    return m_CanCoder.getSensorAbsolutePosition();

  }

  /**
   * Gets the state of the module
   * 
   * @return - State of the module
   */
  public SwerveModuleState getState() {
    SwerveModuleState state = new SwerveModuleState(getSpeed(), Rotation2d.fromDegrees(getAngle()));

    // state.speedMetersPerSecond = getSpeed();
    // state.angle = Rotation2d.fromDegrees(getAngle());

    return state;
  };

  /**
   * Sets the state of the module
   * 
   * @param state - State of the module
   */
  public void setState(SwerveModuleState state) {
    m_State = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getAngle()));
  };

  /**
   * Gets the module position
   * 
   * @return - Module position
   */
  public SwerveModulePosition getPosition() {
    SwerveModulePosition position = new SwerveModulePosition();

    position.angle = Rotation2d.fromDegrees(getAngle());
    position.distanceMeters = getDistance();

    return position;
  };

  /**
   * Sets the voltage of the module 
   */
  public void setVoltages() {
    m_turningMotor.setPositionVoltage(m_State.angle.getDegrees() / 360.0, false);
    // m_driveMotor.setVoltage(m_Feedforward.calculate(m_State.speedMetersPerSecond), false);
    m_driveMotor.setVoltage(setSpeed(m_State.speedMetersPerSecond), false);

  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber(m_name + "/Speed", getSpeed());
    // SmartDashboard.putNumber(m_name + "/Target", m_State.angle.getDegrees());
    // SmartDashboard.putNumber(m_name + "/Angle", getAngle());
  }
}
