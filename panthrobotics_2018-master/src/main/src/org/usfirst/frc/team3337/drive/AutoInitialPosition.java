package main.src.org.usfirst.frc.team3337.drive;

import java.util.Arrays;
import java.util.ArrayList;
import java.lang.IllegalArgumentException;

public enum AutoInitialPosition
{
	LEFT, MIDDLE, RIGHT;

	public static ArrayList<AutoInitialPosition> possibleOptions()
	{
		AutoInitialPosition[] arr = {LEFT, MIDDLE, RIGHT};
		return new ArrayList<>(Arrays.asList(arr));
	}

	public String toString()
	{
		switch (this)
		{ 
			case LEFT:			return "Starting from Left";
			case MIDDLE:		return "Starting From Middle";
			case RIGHT:			return "Starting From Right";
			default:			throw new IllegalArgumentException("Enum Value not specified");
		}
	}
}