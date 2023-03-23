
package frc.robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;


public class Shoulder{
    public double skP = 0.05;
    private final double skI = 0.05;
   private final double skD = 0.001;
    private final double siLimit = 0;
    private double EerrorSum = 0;
   private double SlastError=0;
    public CANSparkMax shoulder;
    public RelativeEncoder shouldere;
    private double lastTimestamp = 0;
    public double Ssetpoint =0;
public Shoulder() {
    shoulder = new CANSparkMax(8, MotorType.kBrushless);
    shouldere = shoulder.getEncoder();
}
public void Shoulder_Run(){
    double Eerror = Ssetpoint- shouldere.getPosition();
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    if (Math.abs(Eerror) < siLimit) {
      EerrorSum += Eerror * dt;
    }

    double EerrorRate = (Eerror - SlastError) / dt;

    double houtput = skP * Eerror + skI * EerrorSum + skD * EerrorRate;

    
    shoulder.set(houtput);

    // update last- variables
     lastTimestamp = Timer.getFPGATimestamp();
    SlastError = Eerror; 
    SmartDashboard.putNumber("Shoulder",shouldere.getPosition());
}
}

