package frc.robot;


//import com.ctre.phoenix.sensors.AbsoluteSensorRange;
//import com.ctre.phoenix.sensors.CANCoderConfiguration;
//import com.ctre.phoenix.sensors.SensorInitializationStrategy;
//import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;

//import frc.robot.subsystems.swerve.SwerveConfig;

public final class CTREConfigs {
  
    public CANcoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
     
        swerveCanCoderConfig = new CANcoderConfiguration();

        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1);
        swerveCanCoderConfig.MagnetSensor.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive); // if backwards change
        //swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        //swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}