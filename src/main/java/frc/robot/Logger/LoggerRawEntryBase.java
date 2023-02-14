package frc.robot.Logger;

//A extensible class that should be used to make custom log entries that generate certain text -- 
//In raw form, so you can add multiple lines or un-time-stamped text into a Logger.
//Override the convertToRawLogText() method, and write the rest of the child class yourself.
//If you want a newline in the LoggerRawEntry, include one in the convertToRawLogText() implementation.

public class LoggerRawEntryBase {
    public String convertToRawLogText() {
        return "Nothing\n";
    }
}

