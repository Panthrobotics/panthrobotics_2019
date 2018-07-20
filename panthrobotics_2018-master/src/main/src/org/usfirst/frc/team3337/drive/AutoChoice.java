package main.src.org.usfirst.frc.team3337.drive;

import java.util.Arrays;
import java.util.ArrayList;
import java.lang.IllegalArgumentException;

public enum AutoChoice
{
	GO_STRAIGHT, OUR_SWITCH, SCALE, OTHER_SWITCH;

	public static ArrayList<AutoChoice> possibleOptions()
	{
		AutoChoice[] arr = {GO_STRAIGHT, OUR_SWITCH, SCALE, OTHER_SWITCH};
		return new ArrayList<>(Arrays.asList(arr));
	}

	public String toString()
	{
		switch (this)
		{ 
			case GO_STRAIGHT:		return "Going Straight";
			case OUR_SWITCH:		return "Going to Switch";
			case SCALE:				return "Gears to Scale";
			case OTHER_SWITCH:		return "Gears to other Switch";
			default:				throw new IllegalArgumentException("Enum Value not specified");
		}
	}
}