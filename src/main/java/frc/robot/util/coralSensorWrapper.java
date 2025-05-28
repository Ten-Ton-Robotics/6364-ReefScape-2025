package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;
//         public static final DigitalInput m_CoralSensor = new DigitalInput(0);

public class coralSensorWrapper extends DigitalInput{
    boolean initial_val;

    public coralSensorWrapper(int id, boolean initial_state){
        super(id);
        initial_val = initial_state;
    }

    public boolean getinitialized(){
        initial_val = super.get();
        return initial_val;
    }
    

}
