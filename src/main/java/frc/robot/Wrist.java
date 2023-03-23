package frc.robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;


public class Wrist{
    public double EkP = 0.06;
    private final double EkI = 0.05;
   private final double EkD = 0.001;
    private final double EiLimit = 0;
    private double EerrorSum = 0;
   private double ElastError=0;
    public CANSparkMax wrist;
    public RelativeEncoder wriste;
    private double lastTimestamp = 0;
    public double Wsetpoint =0;
    //private Pneumatics pneumatics;
public Wrist() {
    //pneumatics = new Pneumatics();
    wrist = new CANSparkMax(6, MotorType.kBrushless);
    wriste = wrist.getEncoder();
}
public void Wrist_Run(){
    double Eerror = Wsetpoint- wriste.getPosition();
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    if (Math.abs(Eerror) < EiLimit) {
      EerrorSum += Eerror * dt;
    }

    double EerrorRate = (Eerror - ElastError) / dt;

    double houtput = EkP * Eerror + EkI * EerrorSum + EkD * EerrorRate;

    
    wrist.set(houtput);

    // update last- variables
     lastTimestamp = Timer.getFPGATimestamp();
    ElastError = Eerror; 
    SmartDashboard.putNumber("Wrist",wriste.getPosition());
}
}
