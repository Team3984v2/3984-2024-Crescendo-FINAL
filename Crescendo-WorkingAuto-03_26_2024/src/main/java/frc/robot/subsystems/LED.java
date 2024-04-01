package frc.robot.subsystems;
import java.util.Stack;

import com.ctre.phoenix.CANifier.PWMChannel;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase{

    AddressableLED led1;
    DigitalOutput light;
    String state;
    Stack<String> queue;
    public LED(int id){
        //led1 = new AddressableLED(id);
        light = new DigitalOutput(id);
        state = "idle";
        queue = new Stack<>();
    }
    public void setState(Boolean state){
        light.set(state);
    }
    public void setState(String state){
        this.state = state;
        if (state.equals("idle")){
            light.set(true);
        }
        else if(state.equals("")){

        }
    }
    public void periodic(){
    }
}
