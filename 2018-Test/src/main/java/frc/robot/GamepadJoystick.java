package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

public class GamepadJoystick extends Joystick {
    public GamepadJoystick(int port) {
        super(port);
    }

    // setting Button
    public Button btn_X = new JoystickButton(this, 1);
    public Button btn_A = new JoystickButton(this, 2);
    public Button btn_B = new JoystickButton(this, 3);
    public Button btn_Y = new JoystickButton(this, 4);

    public Button btn_topL = new JoystickButton(this, 5);
    public Button btn_topR = new JoystickButton(this, 6);
    public Button btn_triggerL = new JoystickButton(this, 7);
    public Button btn_triggerR = new JoystickButton(this, 8);

    public Button btn_Start = new JoystickButton(this, 9);
    public Button btn_Back = new JoystickButton(this, 10);

    // setting POVButton
    public POVButton POV_North = new POVButton(this, 0);
    public POVButton POV_NorthEast = new POVButton(this, 45);
    public POVButton POV_East = new POVButton(this, 90);
    public POVButton POV_SouthEast = new POVButton(this, 135);
    public POVButton POV_South = new POVButton(this, 180);
    public POVButton POV_SouthWest = new POVButton(this, 225);
    public POVButton POV_West = new POVButton(this, 270);
    public POVButton POV_NorthWest = new POVButton(this, 315);

    // Setting Analog Stick
    public double get_LStickX(){
        return getRawAxis(0);
    }
    public double get_LStickY(){
        return getRawAxis(1);
    }
    public double get_RStickX(){
        return getRawAxis(2);
    }
    public double get_RStickY(){
        return getRawAxis(3);
    }
}