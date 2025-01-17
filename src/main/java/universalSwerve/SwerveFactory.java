package universalSwerve;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import universalSwerve.components.Wheel;
import universalSwerve.components.WheelLabel;
import universalSwerve.components.implementations.FalconTranslationSystem;
import universalSwerve.components.implementations.SparkMaxAndLampreyRotationSystem;
import universalSwerve.components.implementations.SparkMaxUtilizingAbsoluteEncoderRotationSystemContinuously;
import universalSwerve.hardware.implementations.ADIS16470Gyro;
import universalSwerve.hardware.implementations.LampreyWheelAngleReader;
import universalSwerve.utilities.PIDFConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveFactory 
{

    /**
     *  
     * 
     */
    public static SwerveDrive Create2023Swerve()
    {        

        
        //Locations:
        double WHEELS_LEFT_RIGHT_FROM_CENTER = 23.75/2.0;
        double WHEELS_FRONT_BACK_FROM_CENTER = 26.5/2.0;

        //Wheel details:
        double WHEEL_DIAMETER = 3.0;
        double DRIVE_GEAR_RATIO = 1.0/4.71;

        double TURN_GEAR_RATIO = 203.0/9424.0;

        double MAX_LINEAR_SPEED = 5 * 12.0; //inches per second // original was 15.76 (scaled down to 5)        double MAX_ROTATIONAL_SPEED = 120; //degrees per second
        double MAX_ROTATIONAL_SPEED = 120; //degrees per second
        double NUDGING_SPEED = 2;

        ADIS16470Gyro gyro = new ADIS16470Gyro(edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis.kX, true);

        //Drive FPID Configurations:
        PIDFConfiguration neDriveFPIDCongiruation = new PIDFConfiguration(0.083, 0, 0, 0.04856 * 1.026* 1.1);
        PIDFConfiguration seDriveFPIDCongiruation = new PIDFConfiguration(0.063, 0, 0, 0.045 * 1.037 * 1.021 * 1.3);
        PIDFConfiguration swDriveFPIDCongiruation = new PIDFConfiguration(0.073, 0, 0, 0.0455 * 1.1);
        PIDFConfiguration nwDriveFPIDCongiruation = new PIDFConfiguration(0.083, 0, 0,  0.046);
        PIDFConfiguration[] driveFPIDConfigurations = new PIDFConfiguration[]{ neDriveFPIDCongiruation, seDriveFPIDCongiruation, swDriveFPIDCongiruation, nwDriveFPIDCongiruation};



        //Turn FPID Configurations:
        PIDFConfiguration neTurnFPIDCongiruation = new PIDFConfiguration(0.13,0,0,0);
        PIDFConfiguration seTurnFPIDCongiruation = new PIDFConfiguration(0.13,0,0,0);
        PIDFConfiguration swTurnFPIDCongiruation = new PIDFConfiguration(0.13,0,0,0);
        PIDFConfiguration nwTurnFPIDCongiruation = new PIDFConfiguration(0.13,0,0,0);
        PIDFConfiguration[] turnFPIDConfigurations= new PIDFConfiguration[]{neTurnFPIDCongiruation, seTurnFPIDCongiruation, swTurnFPIDCongiruation, nwTurnFPIDCongiruation };
        

        //Drive Ports:
        int neDriveChannel = 12;
        int seDriveChannel = 17;
        int swDriveChannel = 2;
        int nwDriveChannel = 7;

        //Turn Ports:
        int neTurnChannel = 19;
        int seTurnChannel = 18;
        int swTurnChannel = 62;
        int nwTurnChannel = 1;

        class DriveFalconCreator { WPI_TalonFX CreateDriveFalcon(int pDriveChannel) { return new WPI_TalonFX(pDriveChannel); }}

        //Drive Falcons:
        DriveFalconCreator driveFalconCreator = new DriveFalconCreator();
        WPI_TalonFX neDriveFalcon = driveFalconCreator.CreateDriveFalcon(neDriveChannel);
        WPI_TalonFX seDriveFalcon = driveFalconCreator.CreateDriveFalcon(seDriveChannel);
        WPI_TalonFX swDriveFalcon = driveFalconCreator.CreateDriveFalcon(swDriveChannel);
        WPI_TalonFX nwDriveFalcon = driveFalconCreator.CreateDriveFalcon(nwDriveChannel);
        WPI_TalonFX[] driveFalcons = new WPI_TalonFX[] {neDriveFalcon, seDriveFalcon, swDriveFalcon, nwDriveFalcon};


        class TurnSparkMaxCreator { CANSparkMax CreateTurnSparkMax(int pTurnChannel) { 
            CANSparkMax returnValue = new CANSparkMax(pTurnChannel, MotorType.kBrushless); 
            SparkMaxAbsoluteEncoder absoluteEncoder = returnValue.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
            returnValue.getPIDController().setFeedbackDevice(absoluteEncoder);
            returnValue.setSmartCurrentLimit(20);
            return returnValue;
        }}

        //Turn Spak Maxes
        TurnSparkMaxCreator turnSparkMaxCreator = new TurnSparkMaxCreator();
        CANSparkMax neTurnSparkMax = turnSparkMaxCreator.CreateTurnSparkMax(neTurnChannel);
        CANSparkMax seTurnSparkMax = turnSparkMaxCreator.CreateTurnSparkMax(seTurnChannel);
        CANSparkMax swTurnSparkMax = turnSparkMaxCreator.CreateTurnSparkMax(swTurnChannel);
        CANSparkMax nwTurnSparkMax = turnSparkMaxCreator.CreateTurnSparkMax(nwTurnChannel);
        CANSparkMax[] turnSparkMaxes = new CANSparkMax[] { neTurnSparkMax,seTurnSparkMax, swTurnSparkMax, nwTurnSparkMax};

        Wheel[] wheels = new Wheel[4];

      
        for(int i = 0; i < 4; i++)
        {
            wheels[i] = new Wheel(WheelLabel.FromInt(i), 
                ((i == 0 || i == 1) ? 1.0 : -1.0) * WHEELS_LEFT_RIGHT_FROM_CENTER,
                ((i == 0 || i == 3) ? 1.0 : -1.0) * WHEELS_FRONT_BACK_FROM_CENTER,                
            new FalconTranslationSystem(driveFalcons[i], true, driveFPIDConfigurations[i], WHEEL_DIAMETER, DRIVE_GEAR_RATIO),
            new SparkMaxUtilizingAbsoluteEncoderRotationSystemContinuously(turnSparkMaxes[i], TURN_GEAR_RATIO, turnFPIDConfigurations[i]));
        }

       SwerveDrive returnValue = new SwerveDrive(wheels[0], wheels[1], wheels[2], wheels[3], gyro, MAX_LINEAR_SPEED, MAX_ROTATIONAL_SPEED, NUDGING_SPEED);

       return returnValue;

    }





    public static SwerveDrive Create2022Swerve()
    {        
        //Locations:
        double WHEELS_LEFT_RIGHT_FROM_CENTER = 10.625;
        double WHEELS_FRONT_BACK_FROM_CENTER = 12.1875;

        //Wheel details:
        double WHEEL_DIAMETER = 3.25;
        double DRIVE_GEAR_RATIO = 9.0 / 30.0 * 24.0 / 36.0 * 34.0 / 54.0;

        double TURN_GEAR_RATIO = (29.0/84.0) * (21.0/76.0) * (13.0/68.0) * (20.0/32.0);;

        double MAX_LINEAR_SPEED = 3 * 12; //inches per second //original was 9 * 12
        double MAX_ROTATIONAL_SPEED = 120; //degrees per second
        double NUDGING_SPEED = 2;

        ADIS16470Gyro gyro = new ADIS16470Gyro(edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis.kX, true);

        //Drive FPID Configurations:
        PIDFConfiguration neDriveFPIDCongiruation = new PIDFConfiguration(0.083, 0, 0, 0.04856 * 1.026* 1.1);
        PIDFConfiguration seDriveFPIDCongiruation = new PIDFConfiguration(0.063, 0, 0, 0.045 * 1.037 * 1.021 * 1.3);
        PIDFConfiguration swDriveFPIDCongiruation = new PIDFConfiguration(0.073, 0, 0, 0.0455 * 1.1);
        PIDFConfiguration nwDriveFPIDCongiruation = new PIDFConfiguration(0.083, 0, 0,  0.046);
        PIDFConfiguration[] driveFPIDConfigurations = new PIDFConfiguration[]{ neDriveFPIDCongiruation, seDriveFPIDCongiruation, swDriveFPIDCongiruation, nwDriveFPIDCongiruation};



        //Turn FPID Configurations:
        PIDFConfiguration neTurnFPIDCongiruation = new PIDFConfiguration(0.13,0,0,0);
        PIDFConfiguration seTurnFPIDCongiruation = new PIDFConfiguration(0.13,0,0,0);
        PIDFConfiguration swTurnFPIDCongiruation = new PIDFConfiguration(0.13,0,0,0);
        PIDFConfiguration nwTurnFPIDCongiruation = new PIDFConfiguration(0.13,0,0,0);
        PIDFConfiguration[] turnFPIDConfigurations= new PIDFConfiguration[]{neTurnFPIDCongiruation, seTurnFPIDCongiruation, swTurnFPIDCongiruation, nwTurnFPIDCongiruation };
        

        //Drive Ports:
        int neDriveChannel = 12;
        int seDriveChannel = 17;
        int swDriveChannel = 2;
        int nwDriveChannel = 7;

        //Turn Ports:
        int neTurnChannel = 19;
        int seTurnChannel = 18;
        int swTurnChannel = 62;
        int nwTurnChannel = 1;

        class DriveFalconCreator { WPI_TalonFX CreateDriveFalcon(int pDriveChannel) { return new WPI_TalonFX(pDriveChannel); }}

        //Drive Falcons:
        DriveFalconCreator driveFalconCreator = new DriveFalconCreator();
        WPI_TalonFX neDriveFalcon = driveFalconCreator.CreateDriveFalcon(neDriveChannel);
        WPI_TalonFX seDriveFalcon = driveFalconCreator.CreateDriveFalcon(seDriveChannel);
        WPI_TalonFX swDriveFalcon = driveFalconCreator.CreateDriveFalcon(swDriveChannel);
        WPI_TalonFX nwDriveFalcon = driveFalconCreator.CreateDriveFalcon(nwDriveChannel);
        WPI_TalonFX[] driveFalcons = new WPI_TalonFX[] {neDriveFalcon, seDriveFalcon, swDriveFalcon, nwDriveFalcon};


        class TurnSparkMaxCreator { CANSparkMax CreateTurnSparkMax(int pTurnChannel) { 
            CANSparkMax returnValue = new CANSparkMax(pTurnChannel, MotorType.kBrushless); 
            returnValue.setSmartCurrentLimit(20);
            return returnValue;
        }}

        //Turn Spak Maxes
        TurnSparkMaxCreator turnSparkMaxCreator = new TurnSparkMaxCreator();
        CANSparkMax neTurnSparkMax = turnSparkMaxCreator.CreateTurnSparkMax(neTurnChannel);
        CANSparkMax seTurnSparkMax = turnSparkMaxCreator.CreateTurnSparkMax(seTurnChannel);
        CANSparkMax swTurnSparkMax = turnSparkMaxCreator.CreateTurnSparkMax(swTurnChannel);
        CANSparkMax nwTurnSparkMax = turnSparkMaxCreator.CreateTurnSparkMax(nwTurnChannel);
        CANSparkMax[] turnSparkMaxes = new CANSparkMax[] { neTurnSparkMax,seTurnSparkMax, swTurnSparkMax, nwTurnSparkMax};

        

        //Lampreys:
        LampreyWheelAngleReader neLampreyAngleReader = new LampreyWheelAngleReader(4, 0);
        LampreyWheelAngleReader seLampreyAngleReader = new LampreyWheelAngleReader(1, 0);
        LampreyWheelAngleReader swLampreyAngleReader = new LampreyWheelAngleReader(2, 0);
        LampreyWheelAngleReader nwLampreyAngleReader = new LampreyWheelAngleReader(3, 0);
        LampreyWheelAngleReader[] lampreyAngleReaders = new LampreyWheelAngleReader[] {neLampreyAngleReader, seLampreyAngleReader, swLampreyAngleReader, nwLampreyAngleReader };


        Wheel[] wheels = new Wheel[4];

      
        for(int i = 0; i < 4; i++)
        {
            wheels[i] = new Wheel(WheelLabel.FromInt(i), 
                ((i == 0 || i == 1) ? 1.0 : -1.0) * WHEELS_LEFT_RIGHT_FROM_CENTER,
                ((i == 0 || i == 3) ? 1.0 : -1.0) * WHEELS_FRONT_BACK_FROM_CENTER,                
            new FalconTranslationSystem(driveFalcons[i], true, driveFPIDConfigurations[i], WHEEL_DIAMETER, DRIVE_GEAR_RATIO),
            new SparkMaxAndLampreyRotationSystem(turnSparkMaxes[i], TURN_GEAR_RATIO, turnFPIDConfigurations[i], lampreyAngleReaders[i]));
        }

       SwerveDrive returnValue = new SwerveDrive(wheels[0], wheels[1], wheels[2], wheels[3], gyro, MAX_LINEAR_SPEED, MAX_ROTATIONAL_SPEED, NUDGING_SPEED);

       return returnValue;

    }



    
}
