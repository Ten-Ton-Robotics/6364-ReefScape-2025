package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;
//         public static final DigitalInput m_koral_sensor = new DigitalInput(0);

public class koralSensorWrapper extends DigitalInput{
    boolean initial_val;

    public koralSensorWrapper(int id, boolean initial_state){
        super(id);
        initial_val = initial_state;
    }

    public boolean getinitialized(){
        initial_val = super.get();
        return initial_val;
    }
    

}
