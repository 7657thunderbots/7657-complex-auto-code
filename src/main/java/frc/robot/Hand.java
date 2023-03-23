package frc.robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;


public class Hand {
  int timer;
    public double hkP = 0.05;
    private final double hkI = 0.05;
   private final double hkD = 0.001;
    private final double hiLimit = 0;
    private double EerrorSum = 0;
   private double ElastError=0;
    public CANSparkMax hand;
    public RelativeEncoder hande;
    private double lastTimestamp = 0;
    public double hsetpoint =0;
    public final Timer wait = new Timer();

public Hand() {
    hand = new CANSparkMax(7, MotorType.kBrushless);
    hande = hand.getEncoder();
    

}
public void Hand_Run(){
   
 if(Math.abs(hsetpoint-hande.getPosition())<1) {
    hand.set(0);
   }
   else if (Math.abs(hsetpoint-hande.getPosition())>1){

    double Eerror = hsetpoint-hande.getPosition();
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    if (Math.abs(Eerror) < hiLimit) {
      EerrorSum += Eerror * dt;
    }

    double EerrorRate = (Eerror - ElastError) / dt;

    double houtput = hkP * Eerror + hkI * EerrorSum + hkD * EerrorRate;
  
    hand.set(houtput);

    // update last- variables
     lastTimestamp = Timer.getFPGATimestamp();
    ElastError = Eerror; 
}
SmartDashboard.putNumber("Hand",hande.getPosition());
}
}

