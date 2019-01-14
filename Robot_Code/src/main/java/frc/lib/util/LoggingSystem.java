package frc.lib.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

import java.io.*;
import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.attribute.BasicFileAttributes;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.TimeZone;

public class LoggingSystem {

    private File base;
    private PrintWriter printWriter;
    private List<String> smartDashKeys;
    private String toWrite;
    private Notifier loggerThread;
    private boolean initSuccess;
    private Runnable runnable = () -> logLine();

    public LoggingSystem() {
        initSuccess = false;
        smartDashKeys = new ArrayList<>();
        loggerThread = new Notifier(runnable);
        try {
            base = getMount();
            System.out.println(base.getAbsolutePath());
            printWriter = new PrintWriter(new BufferedWriter(new FileWriter(base)));
            initSuccess = true;
        } catch (Exception e) {
            DriverStation.reportError("Failed to initialize log on file!", false);
            //e.printStackTrace();
        }
    }

    public static void WriteBuildInfoToDashboard() {
        String COMP_MSG = "THIS IS THE COMPETITION SOFTWARE CONFIG! CHECK IF ROBOT MATCHES!\n";

        String PRAC_MSG = "THIS IS THE PRACTICE SOFTWARE CONFIG! CHECK IF ROBOT MATCHES!\n";
        DriverStation.reportWarning(Constants.IS_COMP_BOT ? COMP_MSG : PRAC_MSG, false);
        try {
            //get the path of the currently executing jar file
            String currentJarFilePath = Robot.class.getProtectionDomain().getCodeSource().getLocation().toURI().getPath();
            Path filePath = Paths.get(currentJarFilePath);

            //get file system details from current file
            BasicFileAttributes attr = Files.readAttributes(filePath, BasicFileAttributes.class);
            Date utcFileDate = new Date(attr.lastModifiedTime().toMillis());

            // convert from UTC to local time zone
            SimpleDateFormat outputFormatter = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
            outputFormatter.setTimeZone(TimeZone.getTimeZone("US/Eastern"));
            String newDateString = outputFormatter.format(utcFileDate);

            // write the build date & time to the operator's console log window
            DriverStation.reportWarning("== Robot Name == " + Constants.ROBOT_NAME + "| Build Date and Time: " + newDateString + "|", false);
            if (Constants.ENABLE_MP_TEST_MODE) DriverStation.reportWarning("MP TEST MODE IS ENABLED!", false);
        } catch (URISyntaxException e) {
            DriverStation.reportWarning("Error determining filename of current JAR file", true);
            //e.printStackTrace();
        } catch (IOException e) {
            DriverStation.reportWarning("General Error trying to determine current JAR file", true);
            //e.printStackTrace();
        }

    }

    public void addWatchKey(String key) {
        smartDashKeys.add(key);
    }

    public void enablePrint(boolean enable) {
        if (initSuccess) {
            if (enable) {
                loggerThread.startPeriodic(Constants.LOGGING_UPDATE_RATE);
            } else {
                loggerThread.stop();
            }
        } else {
            DriverStation.reportWarning("logger called to init on Null file stream", false);
        }
    }

    private void logLine() {
        if (printWriter != null) {
            toWrite = "" + Timer.getFPGATimestamp() + "\t";
            for (String key : smartDashKeys) {
                toWrite += "" + SmartDashboard.getNumber(key, 0.0) + "\t";
            }
            toWrite += "\r\n";
            //System.out.println(toWrite);
            printWriter.write(toWrite);
            printWriter.flush();
        }
    }
    private File getMount() {
        File media = new File("/media");
        File logging_path = null;
        for(File mount : media.listFiles())
        {
            logging_path = new File(mount.getAbsolutePath() + "/logging");
            if(logging_path.isDirectory()) {
                System.out.println(logging_path.getAbsolutePath());
                break;
            }
            logging_path = null;
        }
        if (!logging_path.equals(null)) {
            SimpleDateFormat outputFormatter = new SimpleDateFormat("yyyyMMdd_HHmmss_SSS");
            outputFormatter.setTimeZone(TimeZone.getTimeZone("US/Eastern"));
            String newDateString = outputFormatter.format(new Date());
            // build the new filename
            String fileName = newDateString + "_LOG.tsv";
            // build the full file path name
            return new File(logging_path.getAbsolutePath() + File.separator + fileName);
        }
        return null;
    }

    /*private File getMount() {
        File mountPoint;
        // find the mount point
        File testPoint = new File(Constants.DRIVE_PATH_1 + "/logging");
        if (testPoint.isDirectory()) { //robotDriveV4 exists on sda
            mountPoint = testPoint;
        } else {
            testPoint = new File(Constants.DRIVE_PATH_2 + "/logging");
            if (testPoint.isDirectory()) {//robotDriveV4 exists on sdb
                mountPoint = testPoint;
            } else {
                mountPoint = null;
            }
        }
        if (mountPoint != null) {
            SimpleDateFormat outputFormatter = new SimpleDateFormat("yyyyMMdd_HHmmss_SSS");
            outputFormatter.setTimeZone(TimeZone.getTimeZone("US/Eastern"));
            String newDateString = outputFormatter.format(new Date());
            // build the new filename
            String fileName = newDateString + "_LOG.tsv";
            // build the full file path name
            return new File(mountPoint.getAbsolutePath() + File.separator + fileName);
        }
        return null;
    }*/


}
