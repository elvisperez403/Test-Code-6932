
package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.util.swerveUtil.CTREModuleState;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

/**
 * a Swerve Modules using REV Robotics motor controllers and CTRE CANcoder absolute encoders.
 */
public class SwerveMod implements SwerveModule
{
    public int moduleNumber;
    private Rotation2d angleOffset;

    private SparkMax mAngleMotor;
    private SparkMax mDriveMotor;

    public static CANcoder angleEncoder;
    private RelativeEncoder relAngleEncoder;
    private RelativeEncoder relDriveEncoder;

    private SparkMaxConfig configDrive;
    private SparkMaxConfig configAngle;


    public SwerveMod(int moduleNumber, RevSwerveModuleConstants moduleConstants)
    {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        configAngle = new SparkMaxConfig();
        configDrive = new SparkMaxConfig();
        /* Angle Motor Config */
        mAngleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        relAngleEncoder = mAngleMotor.getEncoder();
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new SparkMax(moduleConstants.driveMotorID,  MotorType.kBrushless);
        relDriveEncoder = mDriveMotor.getEncoder();
        configDriveMotor();


         /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configEncoders();


       // lastAngle = getState().angle;
    }


    private void configEncoders()
    {     
        // absolute encoder   
      
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        angleEncoder.getConfigurator().apply(new SwerveConfig().canCoderConfig);
       
        /*relDriveEncoder = mDriveMotor.getEncoder();
        relDriveEncoder.setPosition(0);
        configDrive.encoder
            .positionConversionFactor(SwerveConfig.driveRevToMeters)
            .velocityConversionFactor(SwerveConfig.driveRpmToMetersPerSecond);*/    

        resetToAbsolute();
        /*mDriveMotor.configure(configDrive, null, null);
        mAngleMotor.configure(configAngle, null, null);*/
    }

    private void configAngleMotor()
    {

        configAngle
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(SwerveConfig.angleContinuousCurrentLimit);
        configAngle.encoder
            .positionConversionFactor(SwerveConfig.DegreesPerTurnRotation)
            .velocityConversionFactor(SwerveConfig.DegreesPerTurnRotation / 60);
        configAngle.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(SwerveConfig.angleKP, SwerveConfig.angleKI, SwerveConfig.angleKD, SwerveConfig.angleKF)
            .outputRange(-SwerveConfig.anglePower, SwerveConfig.anglePower);
            
    
        mAngleMotor.configure(configAngle, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);       
       
    }

    private void configDriveMotor()
    {        
  
        configDrive
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(SwerveConfig.driveContinuousCurrentLimit);
        configDrive.encoder
            .positionConversionFactor(SwerveConfig.driveRevToMeters)
            .velocityConversionFactor(SwerveConfig.driveRpmToMetersPerSecond);
        configDrive.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(SwerveConfig.driveKP, SwerveConfig.driveKI, SwerveConfig.driveKD, SwerveConfig.driveKF)
            .outputRange(-SwerveConfig.drivePower, SwerveConfig.drivePower);
        
        mDriveMotor.configure(configDrive, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
              
    }



    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop)
    {
        
        
        // CTREModuleState functions for any motor type.
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);

        if(mDriveMotor.getFaults().sensor)
        {
            DriverStation.reportWarning("Sensor Fault on Drive Motor ID:"+mDriveMotor.getDeviceId(), false);
        }

        if(mAngleMotor.getFaults().sensor)
        {
            DriverStation.reportWarning("Sensor Fault on Angle Motor ID:"+mAngleMotor.getDeviceId(), false);
        }
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop)
    {
       
        if(isOpenLoop)
        {
            double percentOutput = desiredState.speedMetersPerSecond / SwerveConfig.maxSpeed;
            mDriveMotor.set(percentOutput);
            return;
        }
 
        double velocity = desiredState.speedMetersPerSecond;
        
        SparkClosedLoopController controller = mDriveMotor.getClosedLoopController();
        controller.setReference(velocity, ControlType.kVelocity);
        
    }

    private void setAngle(SwerveModuleState desiredState)
    {
        if(Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConfig.maxSpeed * 0.01)) 
        {
            mAngleMotor.stopMotor();
            return;

        }
        Rotation2d angle = desiredState.angle; 
        //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        SparkClosedLoopController controller = mAngleMotor.getClosedLoopController(); // i believe this is now closed loop controller
        
        double degReference = angle.getDegrees();
     
       
        
        controller.setReference(degReference, ControlType.kPosition);
        
    }

   

    private Rotation2d getAngle()
    {
        return Rotation2d.fromDegrees(relAngleEncoder.getPosition());
    }

    public Rotation2d getCanCoder()
    {
        
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble());
        //return getAngle();
    }

    public int getModuleNumber() 
    {
        return moduleNumber;
    }

    public void setModuleNumber(int moduleNumber) 
    {
        this.moduleNumber = moduleNumber;
    }

    private void resetToAbsolute()
    {
    
        double absolutePosition =getCanCoder().getDegrees() - angleOffset.getDegrees();
        relAngleEncoder.setPosition(absolutePosition);
    }

  

    public SwerveModuleState getState()
    {
        return new SwerveModuleState(
            relDriveEncoder.getVelocity(),
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(
            relDriveEncoder.getPosition(), 
            getAngle()
        );
    }
}