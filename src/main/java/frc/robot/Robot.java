// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

import javax.management.RuntimeErrorException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
    private CANSparkMax mNETurnSparkMax;
    private CANSparkMax mSETurnSparkMax;
    private CANSparkMax mSWTurnSparkMax;
    private CANSparkMax mNWTurnSparkMax;

    private CANSparkMax mIntakeLowerRollerSparkMax;
    private CANSparkMax mIntakeUpperRollerSparkMax;

    private CANSparkMax mArmLowerJointSparkMax;
    private CANSparkMax mArmUpperJointSparkMax;

    private TalonFX mNEDriveFalcon;
    private TalonFX mSEDriveFalcon;
    private TalonFX mSWDriveFalcon;
    private TalonFX mNWDriveFalcon;
    
    private SparkMaxAbsoluteEncoder mNEEncoder;
    private SparkMaxAbsoluteEncoder mSEEncoder;
    private SparkMaxAbsoluteEncoder mSWEncoder;
    private SparkMaxAbsoluteEncoder mNWEncoder;

    private SparkMaxAbsoluteEncoder mArmLowerEncoder;
    private SparkMaxAbsoluteEncoder mArmUpperEncoder;

    private PneumaticHub mPneumaticsHub; 

    private DoubleSolenoid mIntakeSolenoid;
    private DoubleSolenoid mLeftClawSolenoid;
    private DoubleSolenoid mRightClawSolenoid;

    private XboxController mController;

  

  public void robotInit()
  {
    
      mNETurnSparkMax = InitializeTurnSparkMax(17);
      mSETurnSparkMax = InitializeTurnSparkMax(15);
      mSWTurnSparkMax = InitializeTurnSparkMax(4);
      mNWTurnSparkMax = InitializeTurnSparkMax(2);

      mNEEncoder = mNETurnSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
      mSEEncoder = mSETurnSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
      mSWEncoder = mSWTurnSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
      mNWEncoder = mNWTurnSparkMax.getAbsoluteEncoder(Type.kDutyCycle);


      mIntakeLowerRollerSparkMax = InitializeIntakeSparkMax(3);
      mIntakeUpperRollerSparkMax = InitializeIntakeSparkMax(16);

      mArmLowerJointSparkMax = InitializeArmSparkMax(14);
      mArmUpperJointSparkMax = InitializeArmSparkMax(13);

      mArmLowerEncoder = mArmLowerJointSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
      mArmUpperEncoder = mArmUpperJointSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

      mNEDriveFalcon = InitializeDriveFalcon(19);
      mSEDriveFalcon = InitializeDriveFalcon(18);
      mSWDriveFalcon = InitializeDriveFalcon(1);
      mNWDriveFalcon = InitializeDriveFalcon(2);

      mPneumaticsHub = new PneumaticHub(31);

      mIntakeSolenoid = mPneumaticsHub.makeDoubleSolenoid(10,13);
      mLeftClawSolenoid = mPneumaticsHub.makeDoubleSolenoid(11,14);
      mLeftClawSolenoid = new DoubleSolenoid(1 ,PneumaticsModuleType.REVPH,12,15);


      mController = new XboxController(0);
  }

  private CANSparkMax InitializeTurnSparkMax(int pCANId)
  {
    CANSparkMax returnValue = new CANSparkMax(pCANId, MotorType.kBrushless);
    returnValue.restoreFactoryDefaults();
    returnValue.setSmartCurrentLimit(20);
    returnValue.getPIDController().setFeedbackDevice(returnValue.getAbsoluteEncoder(Type.kDutyCycle));
    returnValue.burnFlash();
    
    return returnValue;
  }

  private CANSparkMax InitializeIntakeSparkMax(int pCANId)
  {
    CANSparkMax returnValue = new CANSparkMax(pCANId, MotorType.kBrushless);
    returnValue.restoreFactoryDefaults();
    returnValue.setSmartCurrentLimit(20);
    returnValue.burnFlash();
    return returnValue;
    
  }

  private CANSparkMax InitializeArmSparkMax(int pCANId)
  {
    
    CANSparkMax returnValue = new CANSparkMax(pCANId, MotorType.kBrushless);
    returnValue.restoreFactoryDefaults();
    returnValue.setSmartCurrentLimit(40);
    returnValue.getPIDController().setFeedbackDevice(returnValue.getAbsoluteEncoder(Type.kDutyCycle));
    returnValue.burnFlash();
    
    return returnValue;
  }

  private TalonFX InitializeDriveFalcon(int pCANId)
  {
    TalonFX returnValue = new TalonFX(pCANId);
    returnValue.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 60, 2));
    return returnValue;
  }
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() 
  {
    SmartDashboard.putNumber("mNEEncoder", mNEEncoder.getPosition());
    SmartDashboard.putNumber("mSEEncoder", mSEEncoder.getPosition());
    SmartDashboard.putNumber("mSWEncoder", mSWEncoder.getPosition());
    SmartDashboard.putNumber("mNWEncoder", mNWEncoder.getPosition());

    SmartDashboard.putNumber("mArmLowerEncoder", mArmLowerEncoder.getPosition());
    SmartDashboard.putNumber("mArmUpperEncoder", mArmUpperEncoder.getPosition());   


  }

  private void TestFalconControl(TalonFX pTalonFX, boolean pRun)
  {
    if(pRun)
    {
      pTalonFX.set(ControlMode.PercentOutput, 0.1);
    }
    else
    {
      pTalonFX.set(ControlMode.PercentOutput, 0);
    }
  }


  private void TestSparkMaxControl(CANSparkMax pSparkMax, boolean pRun)
  {
    if(pRun)
    {
      pSparkMax.set(0.1);
    }
    else
    {
      pSparkMax.set(0);
    }
  }

  private void TestPneumaticsMaxControl(DoubleSolenoid pSolenoid, boolean pRun)
  {
    if(pRun)
    {
      pSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    else
    {
      pSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void teleopPeriodic()
  {
    if (!mController.getLeftBumper())
      {
      TestFalconControl(mNEDriveFalcon, mController.getStartButton() && mController.getYButton());
      TestFalconControl(mSEDriveFalcon, mController.getStartButton() && mController.getBButton());
      TestFalconControl(mSWDriveFalcon, mController.getStartButton() && mController.getAButton());
      TestFalconControl(mNWDriveFalcon, mController.getStartButton() && mController.getXButton());

      TestSparkMaxControl(mNETurnSparkMax, !mController.getStartButton() && mController.getYButton());
      TestSparkMaxControl(mSETurnSparkMax, !mController.getStartButton() && mController.getBButton());
      TestSparkMaxControl(mSWTurnSparkMax, !mController.getStartButton() && mController.getAButton());
      TestSparkMaxControl(mNWTurnSparkMax, !mController.getStartButton() && mController.getXButton());
    }
    else
    {
      TestPneumaticsMaxControl(mIntakeSolenoid,  mController.getYButton());
      TestPneumaticsMaxControl(mLeftClawSolenoid, mController.getBButton());
      TestPneumaticsMaxControl(mRightClawSolenoid, mController.getAButton());
    }
    TestSparkMaxControl(mArmLowerJointSparkMax, mController.getRightTriggerAxis() > 0.5);
    TestSparkMaxControl(mArmUpperJointSparkMax, mController.getLeftTriggerAxis() > 0.5);

    TestSparkMaxControl(mIntakeLowerRollerSparkMax, mController.getLeftBumper());
    TestSparkMaxControl(mIntakeUpperRollerSparkMax, mController.getRightBumper());

    


  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
 
  } 
    

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
