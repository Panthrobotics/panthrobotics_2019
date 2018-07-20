package main.src.org.usfirst.frc.team3337.drive;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.Timer;

public class ToggleButton extends Button
{
    private static ArrayList<ToggleButton> tbuttons = new ArrayList<>();
    private Button button;
    public static Timer limit;

    private boolean toggleOn, lastButtonState, justPressed, currentButtonState;
    
    public void reset()
    {
        toggleOn        = false;
        lastButtonState = false;
        justPressed     = false;
    }

    public ToggleButton(Button joyButton)
    {
        button = joyButton;
        tbuttons.add(this);
        reset();
    } 

    private void updateToggleState()
    {
        currentButtonState = button.get();
        if (currentButtonState && !lastButtonState )
        {
            toggleOn = !toggleOn;
            justPressed = true;
        }
        else
            justPressed = false;
        lastButtonState = currentButtonState;
    }

    public static void updateToggleButtons()
    {
        for(ToggleButton tbutton: tbuttons)
        {
            tbutton.updateToggleState();
        }
    }
    
    public boolean get()
    {
        return toggleOn;
    }
    
    public void setToggleState(boolean state)
    {
        toggleOn = state;
    }
    
    public boolean justPressed()
    {
        return justPressed;
    }
}