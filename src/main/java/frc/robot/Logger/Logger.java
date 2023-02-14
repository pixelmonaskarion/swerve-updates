package frc.robot.Logger;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.DateFormat;
import java.util.Calendar;
//import java.util.Collection;
import java.util.TimeZone;
 
//next:
//Make categories extensible
//Clean up
//Unit test
//Wrappers that handle exceptions themselves
//Maybe some useful "example" LoggerEntryBase extensions for use

public class Logger {
    //It's reccomended to use a minimal amount of Logger objects, singular ones per program are usually only needed.
    //only differences in multiple Logger objects is seperate buffers.
    //Additionally, it is more efficient to operate a singular logger object throughout the entire scope
    //of a program because it then keeps track of the searched existing archived logs.

    public enum Categories {
        SENSORS,TESTS
    }
    //to add more log types, add a log name to categories, then
    //add a corresponding filename in filenames (in the same order),
    //then add an extra ,"" in the buffer variable (each log has an)
    //individual buffer
    private String[] filenames = {"sensors", "tests"};
    private final String extension = ".log";
    private final String archivedir = "archives/";
    private String[] buffer = new String[] {"" , ""};
    private String TIMEZONE;
    private int logcounter = 1;

    public static void main(String[] args) {
        test();
    }
    public static void test() {
        try {
            Logger l = new Logger();
            Categories s = Categories.SENSORS;
            Categories t = Categories.TESTS;
            l.addLine("test1", l.currentTimeStamp(), s);
            l.flush(s);
            l.addRaw("test2!!!!!😒\n",s);
            l.flush(s);
            l.clearLog(s);
            l.archive(s);
            l.addLine("test3", l.currentTimeStamp(), t);
            l.flush(t);
            l.clearLog(t);
            l.archive(t);

        }
        catch (IOException i) {

        }
        finally {}
    }
    public void flush(Categories LogCategory) throws IOException {
        int n = LogCategory.ordinal();
        FileWriter o = new FileWriter(filenames[n]+extension, true);
        try {
            o.write(buffer[n]);
            buffer[n] = new String("");
         } finally {
            if (o != null) {
               o.close();
            }
        } 
    }
    public void clearBuffer(Categories LogCategory) {
        int n = LogCategory.ordinal();
        buffer[n] = "";
    }

    public void clearLog(Categories LogCategory) throws IOException {
        int n = LogCategory.ordinal();
        FileWriter o = new FileWriter(filenames[n]+extension);
        try {
            o.write(buffer[n]);
        } finally {
            if (o!= null) {
                o.close();
            }
        }
    }
    public void archive(Categories LogCategory) throws IOException {
        //the following algorithm moves logs into an archive folder with a number, and repopulates
        //the numbers if some are deleted. If this behavior isn't wanted, either avoid it by
        //not deleting log files, or by reviewing timestamps on log files, or by
        //implementing new code that either checks the top log number and continues from there, or
        //stores the next log number in a seperate file that isn't destroyed in between machine runs
        String filename = filenames[LogCategory.ordinal()];
        Path path;
        String attempt = "";
        {
            File f = new File(archivedir);
            f.mkdir();
        }
        do {
            attempt = archivedir+filename+logcounter+extension;
            path = Paths.get(attempt);
            logcounter++;
        } while(Files.exists(path));   
        
        //Attempt now stores a good path

        Path success = Files.move(
            Paths.get(filename+extension),
            Paths.get(attempt));
        if (success == null) {
            throw new IOException("Failed to access source or target destination");
        };

    }
    public void addRaw(String info, Categories LogCategory) {
        int n = LogCategory.ordinal();
        buffer[n] = buffer[n] + info;
    }

    public String currentTimeStamp() {
        Calendar c = Calendar.getInstance(TimeZone.getTimeZone(TIMEZONE));
        DateFormat f = DateFormat.getDateTimeInstance(DateFormat.MEDIUM, DateFormat.MEDIUM);
        return f.format(c.getTime());
    }

    public void addLine(String logline, String timestamp, Categories LogCategory) {
        //wrapper to add timestamp and newline to a log string entry
        addRaw(timestamp + ": " + logline + "\n", LogCategory);
    }

    public void addEntry(LoggerRawEntryBase entry, Categories LogCategory) {
        addRaw(entry.convertToRawLogText(), LogCategory);
    }
    
    public void addEntry(LoggerEntryBase entry, String timestamp, Categories LogCategory) {
        addLine(entry.convertToLogText(), timestamp, LogCategory);
    }
}
