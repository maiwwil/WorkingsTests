package universalSwerve;

import universalSwerve.components.Wheel;
import universalSwerve.hardware.IGyroscope;
import universalSwerve.utilities.AngleUtilities;
import universalSwerve.utilities.Conversions;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive 
{
    public enum SwerveNudgingDirection
    {
        NONE,
        NORTH,
        SOUTH,
        EAST,
        WEST
    }

    private Wheel mNEWheel;
    private Wheel mSEWheel;
    private Wheel mSWWheel;
    private Wheel mNWWheel;

    private Wheel[] mWheels;

    private IGyroscope mGyroscope;

    private SwerveDriveKinematics mSwerveDriveKinematics;

    private DrivingStyle mDrivingStyle = DrivingStyle.FIELD_RELATIVE;

    private double mMaxLinearSpeed; //meters per second, 2022 was 3.0
    private double mMaxRotationalSpeed; //degrees per second, 2022 was 120

    private double mLastIntentionalAngle = 0;
    private boolean mIsIntentionallyTurning = false;
    private double mNudgingSpeed = 0.2;

	public enum DrivingStyle
	{
		FIELD_RELATIVE,
		ROBOT_RELATIVE
	}

    public SwerveDrive(Wheel pNEWheel, Wheel pSEWheel, Wheel pSWWheel, Wheel pNWWheel, IGyroscope pGyroscope,
        double pMaxLinearSpeed, double pMaxRotationalSpeed, double pNudgingSpeed)
    {
        mNEWheel = pNEWheel;
        mSEWheel = pSEWheel;
        mSWWheel = pSWWheel;
        mNWWheel = pNWWheel;
        mWheels = new Wheel[4];
        mWheels[0] = mNEWheel;
        mWheels[1] = mSEWheel;
        mWheels[2] = mSWWheel;
        mWheels[3] = mNWWheel;

        mGyroscope = pGyroscope;

        mSwerveDriveKinematics = new SwerveDriveKinematics(
            mNEWheel.GetCenterToWheelTranslation(),
            mSEWheel.GetCenterToWheelTranslation(),
            mSWWheel.GetCenterToWheelTranslation(),
            mNWWheel.GetCenterToWheelTranslation());

        mMaxLinearSpeed = pMaxLinearSpeed;
        mMaxRotationalSpeed = pMaxRotationalSpeed;
        mNudgingSpeed = pNudgingSpeed;
    }
    
    public void Initialize()
    {
        for(int i = 0; i < mWheels.length; i++)
        {
            mWheels[i].Initialize();
        }
    }
   
    private SwerveModuleState[] CalculateModuleTargetStates(double linearSpeedFrontBackComponent, double linearSpeedLeftRightComponent, double requestedRotationalSpeed, double gyroAngle, DrivingStyle pDrivingStyle)
    {
        //First parameter is m/s forward, second is m/s to the left, third is radians/second counter clockwise
        //This works without gyro:
        //ChassisSpeeds requestedChassisSpeeds = 
        //	new ChassisSpeeds(linearSpeedFrontBackComponent, -1.0 * linearSpeedLeftRightComponent, -1.0 * Math.toRadians(requestedRotationalSpeed));
        //Now add in gyro:
        
        ChassisSpeeds requestedChassisSpeeds;
        

        double linearSpeedFrontBackComponent_MetersPerSecond = Conversions.InchesPerSecondToPetersPerSecond(linearSpeedFrontBackComponent);
        double linearSpeedLeftRightComponent_MetersPerSecond = Conversions.InchesPerSecondToPetersPerSecond(linearSpeedLeftRightComponent);

        
        if(pDrivingStyle == DrivingStyle.FIELD_RELATIVE)
        {
            requestedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                linearSpeedFrontBackComponent_MetersPerSecond, 
                    -1.0 * linearSpeedLeftRightComponent_MetersPerSecond, 
                    -1.0 * Math.toRadians(requestedRotationalSpeed),
                    Rotation2d.fromDegrees(AngleUtilities.ConvertOurAnglesToSwerveKinematicsAngles(gyroAngle)));
        }
        else //robot relative
        {
            requestedChassisSpeeds = new ChassisSpeeds(linearSpeedFrontBackComponent_MetersPerSecond, -1.0 * linearSpeedLeftRightComponent_MetersPerSecond, -1.0 * Math.toRadians(requestedRotationalSpeed));
        }								
        
        return mSwerveDriveKinematics.toSwerveModuleStates(requestedChassisSpeeds);
    }


    private static double GetLinearSpeedLeftRightComponent(double pAngle, double pRequestedLinearSpeed)
	{
		//the angle here is our normal coordinates
		//0 is straight ahead, 90 is to the right, etc.
		//The speed is the total speed that we want, i.e., the hypotenuse of the velocity vector
		//this assumes positive speeds only.
		if(pAngle == 0)
		{
			return 0;
		}
		else if(pAngle < 90)
		{
			return pRequestedLinearSpeed * Math.sin( Math.toRadians(pAngle));
		}
		else if (pAngle == 90)
		{
			return pRequestedLinearSpeed;
		}
		else if (pAngle < 180)
		{
			return pRequestedLinearSpeed * Math.cos( Math.toRadians(pAngle-90.0));
		}
		else if(pAngle == 180)
		{
			return 0;
		}
		else if(pAngle < 270)
		{
			return -1.0 * pRequestedLinearSpeed * Math.sin(Math.toRadians(pAngle - 180.0));
		}
		else if(pAngle == 270)
		{
			return -1.0 * pRequestedLinearSpeed;
		}
		else if(pAngle < 360)
		{
			return -1.0 * pRequestedLinearSpeed * Math.cos(Math.toRadians(pAngle - 270.0));
		}
		else
		{
			return 0;//probably not a real case
		}
	}

	private static double GetLinearSpeedFrontBackComponent(double pAngle, double pRequestedLinearSpeed)
	{
		//the angle here is our normal coordinates
		//0 is straight ahead, 90 is to the right, etc.
		//The speed is the total speed that we want, i.e., the hypotenuse of the velocity vector
		//this assumes positive speeds only.
		if(pAngle == 0)
		{
			return pRequestedLinearSpeed;
		}
		else if(pAngle < 90)
		{
			return pRequestedLinearSpeed * Math.cos( Math.toRadians(pAngle));
		}
		else if (pAngle == 90)
		{
			return 0;
		}
		else if (pAngle < 180)
		{
			return -1.0* pRequestedLinearSpeed * Math.sin( Math.toRadians(pAngle-90.0));
		}
		else if(pAngle == 180)
		{
			return -pRequestedLinearSpeed;
		}
		else if(pAngle < 270)
		{
			return -1.0 * pRequestedLinearSpeed * Math.cos(Math.toRadians(pAngle - 180.0));
		}
		else if(pAngle == 270)
		{
			return 0;
		}
		else if(pAngle < 360)
		{
			return pRequestedLinearSpeed * Math.sin(Math.toRadians(pAngle - 270.0));
		}
		else
		{
			return 0;//probably not a real case
		}
	}


      /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. If this is used with the PIDController class's
   * continuous input functionality, the furthest a wheel will ever rotate is 90 degrees.
   *
   * ATS:  This is copied straight from WPILib's SwerveModuleState class, but I don't like the fact that they use 90 here
   * In particular, if your wheel is spinning quickly, and you direct a 91 degree move, it would be stupid to 
   * chuck the motor into reverse just to save 2 degrees.  So we'll use an arbitrary 110 instead. 
   * We could come back to this and be a bit more clever and say "if the wheel is hardly spinning use 90 otherwise something a bit bigger" 
   * but that's a task for another day...
   * 
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   * @return Optimized swerve module state.
   */
    public static SwerveModuleState Optimize(SwerveModuleState desiredState, Rotation2d currentAngle) 
    {
        var delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > 110.0) //ATS switched 90 to 110
        {
                return new SwerveModuleState(-desiredState.speedMetersPerSecond, desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
        } 
        else
        {
            return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
    }


    private static SwerveModuleState OptimizeTargetState(SwerveModuleState pTargetState, double pCurrentWheelAngle)    
    {
        SwerveModuleState optimizedTargetState = Optimize(
                            pTargetState, 
                            Rotation2d.fromDegrees(AngleUtilities.ConvertOurAnglesToSwerveKinematicsAngles(pCurrentWheelAngle))
        );
        return optimizedTargetState;
    }

    /*
        Computes the module states for the given requested input
        pXTranslationComponent: [-1, 1]:  
            The percentage of the maximum translation speed to be given in the X direction.
            X here is Field relative when in Field relative mode or Robot Releative when in robot relative mode
            1 means "right" in the context above, -1 means "left in the context above
            This is generally mapped directly onto the left joystick X value when normal driving
        pYTranslationComponent: [-1, 1]:  
            The percentage of the maximum translation speed to be given in the Y direction.
            Y here is Field relative when in Field relative mode or Robot Releative when in robot relative mode
            1 means "forward" in the context above, -1 means "backwards" in the context above
            This is generally mapped directly onto the left joystick Y value when normal driving
        pTranslationVelocityPercentage: [0,1]:
            The percentage of the maximum linear speed for the module
        pTranslationVelocityPercentage: [-1,1]:
            The percentage of the maximum rotational speed for the module
            +1 is clockwise, -1 is counte clockwise
        
    */
    private void CalculateAndApplyModuleStatesForSwerveInput(double pXTranslationComponent, double pYTranslationComponent, double pTranslationVelocityPercentage, double pRotationSpeedPercentage, DrivingStyle pDrivingStyle)
	{
		double controllerAngle = AngleUtilities.ConvertLinearComponentsToAngle(pXTranslationComponent, pYTranslationComponent);
		
		double requestedLinearSpeed = pTranslationVelocityPercentage * mMaxLinearSpeed;

		double linearSpeedLeftRightComponent = GetLinearSpeedLeftRightComponent(controllerAngle, requestedLinearSpeed);		

		double linearSpeedFrontBackComponent = GetLinearSpeedFrontBackComponent(controllerAngle, requestedLinearSpeed);
		
		double requestedRotationalSpeed = pRotationSpeedPercentage * mMaxRotationalSpeed;
		
		// pass the swerver calculator each component and let it build (internally the target states)
		SwerveModuleState[] targetModelStates = CalculateModuleTargetStates(linearSpeedFrontBackComponent, linearSpeedLeftRightComponent, requestedRotationalSpeed, mGyroscope.GetCurrentAngle(), pDrivingStyle);
		// the states/targets have been calculated above, now assign
        for(int i = 0; i < mWheels.length; i++)
		{			
            //if(i==3)//just test NW
            {
                Wheel wheel = mWheels[i];
                SmartDashboard.putNumber("CurrentWheelAngle", wheel.GetCurrentAngle());
                SmartDashboard.putNumber("RawCurrentWheelAngle", wheel.GetRawCurrentAngle());
                SwerveModuleState targetState = targetModelStates[i];

                //this takes care of the "drive it in reverse if your are close to the right angle already" thing:
                SwerveModuleState optimizedTargetState = OptimizeTargetState(targetState, wheel.GetCurrentAngle());
                //SwerveModuleState optimizedTargetState =targetState;

                double targetAngle = AngleUtilities.ConvertSwerveKinematicsAnglesToOurAngles(optimizedTargetState.angle.getDegrees());
                SmartDashboard.putNumber("optimizedTargetState.speedMetersPerSecond", optimizedTargetState.speedMetersPerSecond);
                wheel.SetWheelTargetAngle(targetAngle);
                wheel.SetWheelVelocity(Conversions.MetersPerSecondToInchesPerSecond(optimizedTargetState.speedMetersPerSecond));
            }
		}		
	}

    public void StandardSwerveDrive(double pXTranslationComponent, double pYTranslationComponent, double pTranslationVelocityPercentage, double pRotationSpeedPercentage)
    {
        if(Math.abs(pTranslationVelocityPercentage) < 0.01)
        {
            this.mLastIntentionalAngle = mGyroscope.GetCurrentAngle();
            this.mIsIntentionallyTurning = false;
        }
        else
        {
            this.mIsIntentionallyTurning = true;
        }
        
        CalculateAndApplyModuleStatesForSwerveInput(pXTranslationComponent, pYTranslationComponent, pTranslationVelocityPercentage, pRotationSpeedPercentage, mDrivingStyle);

    }

    public void Nudge(SwerveNudgingDirection pDirection)
    {
        double xNudgingComponent = 0;
        double yNudgingComponent = 0;
        switch (pDirection) {
            case NORTH:
                yNudgingComponent = 1;
                xNudgingComponent = 0;
                break;
            case EAST:
                yNudgingComponent = 0;
                xNudgingComponent = 1;
                break;
            case SOUTH:
                yNudgingComponent = -1;
                xNudgingComponent = 0;
                break;
            case WEST:
                yNudgingComponent = 0;
                xNudgingComponent = -1;
                break;			
            default:
                yNudgingComponent = 0;
                xNudgingComponent = 0;
                break;
        }


        this.mLastIntentionalAngle =  mGyroscope.GetCurrentAngle();

        CalculateAndApplyModuleStatesForSwerveInput(xNudgingComponent, yNudgingComponent, mNudgingSpeed, 0, DrivingStyle.ROBOT_RELATIVE);
    }
}
