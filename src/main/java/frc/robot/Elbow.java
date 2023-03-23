package frc.robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;

public class Elbow {
    public double EkP = 0;
    private final double EkI = 0.0;
   private final double EkD = 0.01;
    private final double EiLimit = 1;
    private double EerrorSum = 0;
   private double ElastError=0;
    public CANSparkMax Elbowmax;
    public RelativeEncoder Elbowencoder;
    private double lastTimestamp = 0;
    public double Esetpoint =0;

public Elbow() {
    
    Elbowmax = new CANSparkMax(9, MotorType.kBrushless);
    Elbowencoder = Elbowmax.getEncoder();
}
public void ElbowRun(){
    double Eerror = Esetpoint-Elbowencoder.getPosition();
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    if (Math.abs(Eerror) < EiLimit) {
      EerrorSum += Eerror * dt;
    }

    double EerrorRate = (Eerror - ElastError) / dt;

    double houtput = EkP * Eerror + EkI * EerrorSum + EkD * EerrorRate;

    
    Elbowmax.set(houtput);

    // update last- variables
     lastTimestamp = Timer.getFPGATimestamp();
    ElastError = Eerror; 

SmartDashboard.putNumber("Elbow",Elbowencoder.getPosition());
}
}
