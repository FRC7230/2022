package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
public class Js{
    private final Joystick m_stick = new Joystick(0);
    public Joystick getJs(){
        return m_stick;
    }
}