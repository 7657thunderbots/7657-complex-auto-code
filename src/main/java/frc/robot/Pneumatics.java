package frc.robot;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Pneumatics {
    public static int forwardChannel1 =0;
    public static int reverseChannel1 =1;
    public static int forwardChannel2 =15;
    public static int reverseChannel2 =14;

    private static final int PH_CAN_ID = 1;
    public PneumaticHub m_pH = new PneumaticHub(PH_CAN_ID);
    
    public DoubleSolenoid doubleSolenoid1 = m_pH.makeDoubleSolenoid(0,1);
    DoubleSolenoid mdoubleSolenoid = m_pH.makeDoubleSolenoid(15,14);
    public Pneumatics(){  
      doubleSolenoid1.set(DoubleSolenoid.Value.kForward);
    }

    public void Run_Pneumatics(){
        switch (doubleSolenoid1.get()) {
            case kOff:
              SmartDashboard.putString("Get Solenoid", "kOff");
              break;
            case kForward:
              SmartDashboard.putString("Get Solenoid", "kForward");
              break;
            case kReverse:
              SmartDashboard.putString("Get Solenoid", "kReverse");
              break;
            default:
              SmartDashboard.putString("Get Solenoid", "N/A");
              break;
            }
            
            if (SmartDashboard.getBoolean("Enable Compressor Digital", false)) {
                SmartDashboard.putBoolean("Enable Compressor Digital", false);

            m_pH.enableCompressorDigital();
            }
            
            if (SmartDashboard.getBoolean("Disable Compressor", false)) {
                SmartDashboard.putBoolean("Disable Compressor", false);
          
              m_pH.disableCompressor();
              }
    

                switch (mdoubleSolenoid.get()) {
                case kOff:
                  SmartDashboard.putString("Get Solenoid", "kOff");
                  break;
                case kForward:
                  SmartDashboard.putString("Get Solenoid", "kForward");
                  break;
                case kReverse:
                  SmartDashboard.putString("Get Solenoid", "kReverse");
                  break;
                default:
                  SmartDashboard.putString("Get Solenoid", "N/A");
                  break;
                } 
              }
}
