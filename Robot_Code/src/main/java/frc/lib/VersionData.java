package frc.lib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

public class VersionData{

    public static void doVersionID(){
        try {
            File version;
            if(RobotBase.isReal()) version = new File(Filesystem.getDeployDirectory(),"version.dat");
            else version = new File(Filesystem.getLaunchDirectory(),"src\\main\\deploy\\version.dat");
            System.out.println(version.getPath());
            BufferedReader reader = new BufferedReader(new FileReader(version));
            String contents = reader.readLine();
            int versionid = Integer.parseInt(contents.substring(contents.indexOf("=") + 1,contents.indexOf(';')));
            DriverStation.reportWarning("Build version ID: " + versionid, false);
        } catch (IOException e) {
            DriverStation.reportError("Failed to read version.dat in deploy directory!", e.getStackTrace());
        }
        
    }

}