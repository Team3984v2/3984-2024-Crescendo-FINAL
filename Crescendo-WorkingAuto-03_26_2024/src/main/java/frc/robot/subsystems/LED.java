package frc.robot.subsystems;
import com.ctre.phoenix.CANifier.PWMChannel;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{

    AddressableLED led1;
    PWM bleh;
    public LED(int id){
        //led1 = new AddressableLED(id);
        bleh = new PWM(id);
    }
    
    public void periodic(){
    }
}
