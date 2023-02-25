package universalSwerve.components.implementations;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import universalSwerve.components.ITranslationSystem;
import universalSwerve.utilities.Conversions;
import universalSwerve.utilities.PIDFConfiguration;

public class FalconTranslationSystem implements ITranslationSystem{

    private WPI_TalonFX mFalcon;
    private double mWheelDiameter;
    private double mGearRatio;

    /*
        Diameter is in inches
        Gear ratio is a fraction, so a 10:1 should be passed in as 0.1
    */
    public FalconTranslationSystem(WPI_TalonFX pFalcon, boolean pInvertMotor, PIDFConfiguration pPidfConfiguration, double pWheelDiameter, double pGearRatio)
    {
        mFalcon = pFalcon;
        InitializeFalcon(pInvertMotor, pPidfConfiguration);
        mWheelDiameter = pWheelDiameter;
        mGearRatio = pGearRatio;
    }

    @Override
    public void Initialize() {


        
    }


    public void ResetDriveEncodersToZero()
	{
		mFalcon.setSelectedSensorPosition(0);
	}

	private double ENCODER_TICKS_PER_REVOLUTION_OF_MOTOR = 2048;
	
	private double ConvertDriveEncoderToDistanceTravelled(double pEncoderTicks)
	{
		double numberOfMotorRotations = pEncoderTicks / ENCODER_TICKS_PER_REVOLUTION_OF_MOTOR ;
		double numberOfWheelRotations = numberOfMotorRotations  * mGearRatio;
		double distanceTravelled = Math.abs(numberOfWheelRotations * Math.PI * mWheelDiameter);
		return  distanceTravelled;
		

	}

	public double GetDistanceTravelled()
	{
		return ConvertDriveEncoderToDistanceTravelled(this.mFalcon.getSelectedSensorPosition());
	}


    @Override
    public void SetVelocity(double pVelocity) {
        //pVelocity is in inches per second
        //Let's convert that to RPMS
        //SmartDashboard.putNumber("VelocityInchesPerSecond", pVelocity);
        double rpms = Conversions.InchesPerSecondToRPMS(pVelocity, this.mWheelDiameter);        
        //SmartDashboard.putNumber("rpms", rpms);
        double rpmsAtMotor = rpms / mGearRatio;
        //SmartDashboard.putNumber("rpmsAtMotor", rpmsAtMotor);
        double targetVelocity = Conversions.RPMsToFalconVelocityUnit(rpmsAtMotor);
        //SmartDashboard.putNumber("TargetVelocity", targetVelocity);
        mFalcon.set(ControlMode.Velocity, targetVelocity);
        
    }

    public double GetVelocity()
    {
        return Conversions.RPMstoInchesPerSecond(mFalcon.getSelectedSensorVelocity(), this.mWheelDiameter);
    }

    private void InitializeFalcon(boolean pInvertMotor, PIDFConfiguration pPidfConfiguration)
    {
        mFalcon.setInverted(pInvertMotor);
        mFalcon.setNeutralMode(NeutralMode.Brake);
        mFalcon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,0);
        mFalcon.config_kF(0, pPidfConfiguration.F());
        mFalcon.config_kP(0, pPidfConfiguration.P());
        mFalcon.config_kI(0, pPidfConfiguration.I());
        mFalcon.config_kD(0, pPidfConfiguration.D());
  
    }
  
}
