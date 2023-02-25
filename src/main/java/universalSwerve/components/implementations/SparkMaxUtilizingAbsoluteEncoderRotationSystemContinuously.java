package universalSwerve.components.implementations;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import universalSwerve.components.IRotationSystem;
import universalSwerve.utilities.AngleUtilities;
import universalSwerve.utilities.PIDFConfiguration;

public class SparkMaxUtilizingAbsoluteEncoderRotationSystemContinuously implements IRotationSystem
{
    private CANSparkMax mSparkMax;
    private double mInitialAngle;
    private double mGearRatioBetweenMotorAndWheel;
	private SparkMaxAbsoluteEncoder mAbsoluteEncoder;


	/*
	The gear ratio passed in here should be a fraction, so if it's 100:1, pass in 0.01
	*/
    public SparkMaxUtilizingAbsoluteEncoderRotationSystemContinuously(CANSparkMax pSparkMax, double pGearRatioBetweenMotorAndWheel, PIDFConfiguration pPidfConfiguration)
    {
        mSparkMax = pSparkMax;
		mAbsoluteEncoder = mSparkMax.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
		SetSparkMaxParameters(pPidfConfiguration);
        mGearRatioBetweenMotorAndWheel = pGearRatioBetweenMotorAndWheel;

    }

    private void SetSparkMaxParameters(PIDFConfiguration pPidfConfiguration)
    {
		mSparkMax.restoreFactoryDefaults();
        mSparkMax.setInverted(true);
		mSparkMax.setOpenLoopRampRate(0.35);
		mSparkMax.setSmartCurrentLimit(20);		
		mSparkMax.getPIDController().setFeedbackDevice(mSparkMax.getEncoder());
		mSparkMax.getEncoder().setPosition(0);
		mSparkMax.getPIDController().setFF(pPidfConfiguration.F());
		mSparkMax.getPIDController().setP(pPidfConfiguration.P());
		mSparkMax.getPIDController().setI(pPidfConfiguration.I());
		mSparkMax.getPIDController().setD(pPidfConfiguration.D());
		
		mSparkMax.getPIDController().setOutputRange(-1, 1);
		
    }

    @Override
    public void Initialize() 
    {
		//Pretty sure nothing needs to be done 
    }

	
    @Override
    public double GetCurrentAngle() {
        
        return AngleUtilities.Normalize(mAbsoluteEncoder.getPosition() * 360.0);
    }
	@Override
    public double GetRawCurrentAngle() {
        
        return GetCurrentAngle();
    }



    @Override
    public void SetAngle(double pTargetAngle) {
        mSparkMax.getPIDController().setReference(pTargetAngle, CANSparkMax.ControlType.kPosition);
        
    }
    

    /**
	Returns the number of full rotations of the turn motor
	*/
	private double GetAngle()
	{
		return mSparkMax.getEncoder().getPosition();
	}

	



}
