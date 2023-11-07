package frc.lib.motorcontrollers;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;

import frc.lib.motorcontrollers.NeutralMode.ControllerNeutralMode;

public class OverTalonFX extends TalonFX {

    private TalonFXConfiguration config = new TalonFXConfiguration();

    /**
     * Constructor for OverTalonFX
     * 
     * @param id          The ID of the TalonFX
     * @param neutralMode The neutral mode of the TalonFX
     * @param inverted    Whether or not the TalonFX is inverted
     * @param bus         The bus of the TalonFX
     */
    public OverTalonFX(int id, ControllerNeutralMode neutralMode, boolean inverted, double getRatio, String bus) {
        super(id, bus);

        setNeutralMode(neutralMode);

        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        if (inverted) {
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        } else {
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }

        getConfigurator().apply(config);

    }

    /**
     * Sets the neutral mode of the TalonFX
     * 
     * @param neutralMode The neutral mode of the TalonFX
     */
    public void setNeutralMode(ControllerNeutralMode neutralMode) {
        switch (neutralMode) {
            case BRAKE:
                config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                break;
            case COAST:
                config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                break;
        }
        getConfigurator().apply(config);
    }

    /**
     * Sets the sensor to mechanism ratio of the TalonFX
     * 
     * @param gearRatio The gear ratio of the TalonFX
     */
    public void setSensorToMechanismRatio(double gearRatio) {
        config.Feedback.SensorToMechanismRatio = gearRatio;
        getConfigurator().apply(config);
    }

    /**
     * Sets the rotor to sensor ratio of the TalonFX
     * 
     * @param gearRatio The gear ratio of the TalonFX
     */
    public void setRotorToSensorRatio(double gearRatio) {
        config.Feedback.RotorToSensorRatio = gearRatio;
        getConfigurator().apply(config);
    }

    /**
     * Sets the remote CANCoder of the TalonFX
     * 
     * @param deviceID The devide ID of the remote CANCoder
     */
    public void setRemoteCANCoder(int deviceID) {
        config.Feedback.FeedbackRemoteSensorID = deviceID;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        getConfigurator().apply(config);
    }

    /**
     * Sets the fused CANCoder of the TalonFX
     * 
     * @param deviceID The device ID of the fused CANCoder
     */
    public void setFusedCANCoder(int deviceID) {
        config.Feedback.FeedbackRemoteSensorID = deviceID;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        getConfigurator().apply(config);
    }

    /**
     * Sets the closed Loop voltage ramp rate of the TalonFX
     * 
     * @param ramp The closed Loop voltage ramp rate of the TalonFX
     */
    public void setClosedLoopVoltageRamp(double ramp) {
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ramp;
        getConfigurator().apply(config);
    }

    /**
     * Sets the closed Loop torque ramp rate of the TalonFX
     * 
     * @param ramp The closed Loop torque ramp rate of the TalonFX
     */
    public void setClosedLoopTorqueRamp(double ramp) {
        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = ramp;
        getConfigurator().apply(config);
    }

    /**
     * Sets the supply current limit of the TalonFX
     * 
     * @param enable                  Whether ir not the supply current limit is
     *                                enabled
     * @param currentLimit            The supply current limit of the TalonFX
     * @param triggerThresholdCurrent The trigger threshold current of the Talon FX
     * @param triggerThresholdTime    The trigger threshold time of the TalonFX
     */
    public void SetSupplyCurrentLimit(boolean enable, double currentLimit, double triggerThresholdCurrent,
            double triggerThresholdTime) {
        config.CurrentLimits.SupplyCurrentLimitEnable = enable;
        config.CurrentLimits.SupplyCurrentLimit = currentLimit;
        config.CurrentLimits.SupplyCurrentThreshold = triggerThresholdCurrent;
        config.CurrentLimits.SupplyTimeThreshold = triggerThresholdTime;
        getConfigurator().apply(config);
    }

    /**
     * Sets the torque current limit of the TalonFX
     * 
     * @param peakForward  The peak forward voltage of the TalonFX
     * @param peakBackward The peak reverse voltage of the TalonFX
     * @param deadband     The deadband of the TalonFX
     */
    public void setTorqueCurrentLimit(double peakForward, double peakBackward, double deadband) {
        config.TorqueCurrent.PeakForwardTorqueCurrent = peakForward;
        config.TorqueCurrent.PeakReverseTorqueCurrent = peakBackward;
        config.TorqueCurrent.TorqueNeutralDeadband = deadband;
        getConfigurator().apply(config);

    }

    /**
     * Sets the TalonFX to follow another TalonFX
     * 
     * @param masterID The ID of the TalonFX to follow
     * @param inverted Whether or not the TalonFX is inverted
     */
    public void setFollow(int masterID, boolean inverted) {
        setControl(new Follower(masterID, inverted));
    }

    /**
     * Sets the TalonFX position to 0
     */
    public void zeroPosition() {
        setPosition(0);
    }

    /**
     * Sets the TalonFX position
     * 
     * @param position The position to set the TalonFX to
     */
    public void setSensorPosition(double position) {
        setPosition(position);
    }

    /**
     * Gets the TalonFX position in meters
     * 
     * @param wheelDiameter The diameter of the wheel
     * @param gearRatio     The gear ratio of the TalonFX
     * @return The TalonFX position in meters
     */
    public double getDistance(double wheelDiameter, double gearRatio) {
        double sensorPosition = getPosition().getValueAsDouble();
        return (sensorPosition * wheelDiameter * Math.PI) / gearRatio;
    }

    /**
     * Gets the TalonFX position in meters
     * 
     * @param wheelDiameter The diameter of the wheel
     * @return The TalonFX position in meters
     */
    public double getDistance(double wheelDiameter) {
        double sensorPosition = getPosition().getValueAsDouble();
        return (sensorPosition * wheelDiameter * Math.PI);
    }

    /**
     * Gets the TalonFX velocity in meters per seconds
     * 
     * @param wheelDiameter The diameter of the wheel
     * @param gearRatio     The gear ratio of the TalonFX
     * @return The TalonFX position in meters
     */
    public double getVelocity(double wheelDiameter, double gearRatio) {
        double sensorVelocity = getVelocity().getValueAsDouble();
        return (sensorVelocity * wheelDiameter * Math.PI) / gearRatio;
    }

    /**
     * Gets the TalonFX velocity in meters per seconds
     * 
     * @param wheelDiameter The diameter of the wheel
     * @return The TalonFX position in meters
     */
    public double getVelocity(double wheelDiameter) {
        double sensorVelocity = getVelocity().getValueAsDouble();
        return (sensorVelocity * wheelDiameter * Math.PI);
    }

    /**
     * Gets the TalonFX absolute position
     * 
     * @return The TalonFX absolute position
     */
    public double getAbsolutePosition() {
        return getPosition().getValueAsDouble();
    }

    /**
     * Sets the TalonFX voltage
     * 
     * @param voltage   The voltage to set the TalonFX to
     * @param enableFOC Whether or not to enable FOC
     */
    public void setVoltage(double voltage, boolean enableFOC) {
        VoltageOut voltageOut = new VoltageOut(0);
        voltageOut.EnableFOC = enableFOC;
        setControl(voltageOut.withOutput(voltage));
    }

    /**
     * Sets the TalonFX Velocity using Voltageç
     * 
     * @param velocity  The velocity to set the TalonFX to
     * @param enableFOC Whether or not to enable FOC
     */
    public void setVelocityVoltage(double velocity, boolean enableFOC) {
        VelocityVoltage velocityOut = new VelocityVoltage(0);
        velocityOut.EnableFOC = enableFOC;
        setControl(velocityOut.withVelocity(velocity));
    }

    /**
     * Sets the TalonFX Duty Cycle
     * 
     * @param dutyCycle The duty cycle to set the TalonFX to
     * @param enableFOC Whether or not to enable FOC
     */
    public void setDutyCycle(double dutyCycle, boolean enableFOC) {
        DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
        dutyCycleOut.EnableFOC = enableFOC;
        setControl(dutyCycleOut.withOutput(dutyCycle));
    }

    /**
     * Sets the TalonFX position using voltage
     * 
     * @param position  The position to set the TalonFX to
     * @param enableFOC Whether or not to enable FOC
     */
    public void setPositionVoltage(double position, boolean enableFOC) {
        PositionVoltage positionVoltage = new PositionVoltage(0);
        positionVoltage.EnableFOC = enableFOC;
        setControl(positionVoltage.withPosition(position));
    }

    /**
     * Sets the TalonFX Motion Magic position using Voltage
     * 
     * @param position    The position to set the TalonFX to
     * @param feedForward The feedforward to set the TalonFX to
     * @param enableFOC   Whether or not to enable FOC
     */
    public void setMotionMagicPosition(double position, double feedForward, boolean enableFOC) {
        MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
        motionMagicVoltage.FeedForward = feedForward;
        motionMagicVoltage.EnableFOC = enableFOC;
        setControl(motionMagicVoltage.withPosition(position));
    }

    /**
     * Sets the TalonFX velocity using Torque FOC
     * 
     * @param velocity The velocity to set the TalonFX to
     */
    public void setVelocityTorqueCurrentFOC(double velocity) {
        VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        setControl(velocityTorqueCurrentFOC.withVelocity(velocity));
    }

    /**
     * Sets the TalonFX PID values
     * 
     * @param kP The P value of the TalonFX
     * @param kI The I value of the TalonFX
     * @param kD The D value of the TalonFX
     * @param kS The S value of the TalonFX
     * @param kV The V value of the TalonFX
     */
    public void setPIDValues(double kP, double kI, double kD, double kS, double kV) {
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;
        slot0Configs.kS = kS;
        slot0Configs.kV = kV;

        getConfigurator().apply(slot0Configs);
    }

    /**
     * Sets the TalonFX Motion Magic values
     * 
     * @param cruiseVelocity The cruise velocity of the TalonFX
     * @param acceleration   The acceleration of the TalonFX
     * @param jerk           The jerk of the TalonFX
     */
    public void configureMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = cruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = acceleration;
        motionMagicConfigs.MotionMagicJerk = jerk;

        getConfigurator().apply(motionMagicConfigs);
    }

    /**
     * Sets the TalonFX continous wrap
     */
    public void setContinousWrap() {
        config.ClosedLoopGeneral.ContinuousWrap = true;
        getConfigurator().apply(config);
    }

}