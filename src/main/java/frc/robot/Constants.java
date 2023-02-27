package frc.robot;

public class Constants {
    // Difference of two measured values
    public final static double LOWER_JOINT_ABSOLUTE_TO_RELATIVE_RATIO = 112; //4*4*3*56/24 //Math.abs(-359.921326-(-471.838196));
    
    public final static double LOWER_JOINT_ENCODER_ZERO = 0.44000;
    public final static double LOWER_JOINT_RELATIVE_ENCODER_ZERO = LOWER_JOINT_ENCODER_ZERO * LOWER_JOINT_ENCODER_ZERO;

    public static class Claw {
        public final static int DOUBLE_SOLENOID_RIGHT_ID = 1;
        public final static int DOUBLE_SOLENOID_LEFT_ID = 2;
        public final static int FORWARD_CHANNEL_RIGHT = 3;
        public final static int REVERRSE_CHANNEL_RIGHT = 4;
        public final static int FORWARD_CHANNEL_LEFT = 5;
        public final static int REVERSE_CHANNEL_LEFT = 6;
    }

}           
