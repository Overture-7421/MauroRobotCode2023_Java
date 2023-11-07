package frc.lib.sensors;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

public class OverCANCoder extends CANcoder {

    private CANcoderConfiguration config = new CANcoderConfiguration();

    /**
     * Constructor for OverCANCoder
     * 
     * @param id     CAN ID of the CANCoder
     * 
     * @param offset Offset of the CANCoder
     * 
     * @param bus    CAN Bus of the CANCoder
     */
    public OverCANCoder(int id, double offset, String bus) {
        super(id, bus);
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = offset / 360.0;
        getConfigurator().apply(config);
    }

    /**
     * Gets the CANCoder's absolute position
     * 
     * @return CANCoder's absolute position
     */
    public double getSensorAbsolutePosition() {
        return getAbsolutePosition().refresh().getValueAsDouble();
    }
}
