package frc.robot;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;

public class color_sensor {
    
    public boolean cone= false;
    public boolean cube=false;
    
    public final I2C.Port i2cPort = I2C.Port.kOnboard;
    public double red = 0;
    public double blue = 0;
    public double green = 0;
    public double IR = 0;
    public double proximity=0;
    public final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    public color_sensor(){
        
    
    }

    public void run_color_sensor(){
        Color detectedColor = m_colorSensor.getColor();
        double IR = m_colorSensor.getIR();
        int proximity = m_colorSensor.getProximity();
        red = detectedColor.red;
        blue =detectedColor.blue;
        green =detectedColor.green;
        
        if (detectedColor.blue>.25){
            cube=true;
          cone=false;
          }
          else if(detectedColor.blue<.22){
            cone=true;
            cube=false;
          }
          else {
            cube=false;
            cone=false;
          }
    }
}
