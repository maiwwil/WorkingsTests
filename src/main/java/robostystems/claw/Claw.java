package robostystems.claw;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*; //import static class representing the value of the DoubleSolenoid (sm Sam noted)
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Constants;

public class Claw { //a bunch of variables/objects that are going to be used
  private boolean mOpen; // to use in conditionals (if needed)
  private long mLastTapTime;
  private int mTapCounter;
  private DoubleSolenoid mRightDoubleSolenoid;
  private DoubleSolenoid mLeftDoubleSolenoid;

  public Claw (DoubleSolenoid pRightDoubleSolenoid, DoubleSolenoid pLeftDoubleSolenoid) {
    mOpen = false;
    mLastTapTime = 0;
    mTapCounter = 0;
    mRightDoubleSolenoid = pRightDoubleSolenoid;
    mLeftDoubleSolenoid = pLeftDoubleSolenoid;
  }
    
  public void OpenClaw() {
    if(!mOpen){
      mLeftDoubleSolenoid.set(kForward);
      mRightDoubleSolenoid.set(kForward);
      mOpen = true;  
    }
  }
  
  public void CollapseClaw() {
    if(mOpen){
      mLeftDoubleSolenoid.set(kReverse);
      mRightDoubleSolenoid.set(kReverse);
      mOpen = false;
    }  
  }
  
  public void Tap() {
    if ((System.currentTimeMillis() - mLastTapTime) >= 100) { //Only tap if 100 ms has passed since last tap 
        if (mTapCounter%2 == 0) { 
        mRightDoubleSolenoid.set(kForward);
        mLeftDoubleSolenoid.set(kReverse);
        }
        else {
        mRightDoubleSolenoid.set(kReverse);
        mLeftDoubleSolenoid.set(kForward);
        }
        mLastTapTime = System.currentTimeMillis(); //Sets the last tap time to current time
        mTapCounter++;
    }
  }
}