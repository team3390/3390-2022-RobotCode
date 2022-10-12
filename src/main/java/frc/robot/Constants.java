package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class Constants {

    public static final class MotorConstants {
        public static final int kLeftLeader = 15;
        public static final int kLeftMotor2 = 14;
        public static final int kLeftMotor3 = 13;

        public static final int kRightLeader = 0;
        public static final int kRightMotor2 = 1;
        public static final int kRightMotor3 = 2;

        public static final int kIntakeMotor = 12;

        public static final int kShooter1 = 16;
        public static final int kShooter2 = 18;
        
        public static final int kShooterFeederUp = 17;
        public static final int kShooterFeederDown = 19;
        public static final int kShooterFeederDown2 = 20;
        public static final int kShooterFeederFront = 3;
    }

    public static final class Pneumatics {
        public static final PneumaticsModuleType P_MODULE_TYPE = PneumaticsModuleType.CTREPCM;

        public static final int kVitesseSolenoidPort = 0;
        public static final int kIntakeSolenoidPort = 1;
    }

    public static final class DIO {
        public static final int[] kRightEncoderPorts = { 0, 1 };
        public static final int[] kLeftEncoderPorts = { 2, 3 };
    }

}
