package se.oru.coordination.coordination_oru.tests.testsPedestrians;

import se.oru.coordination.coordination_oru.TrackingCallback;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulationWithPedestrians;
import se.oru.coordination.coordination_oru.util.ColorPrint;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class MyTrackingCallback extends TrackingCallback {

    static TrajectoryEnvelopeCoordinatorSimulationWithPedestrians tec;
    int robot_id;
    static int criticalSections = 0;
    static String pathFileName;

    MyTrackingCallback(int robot_id, String pathFileName, TrajectoryEnvelopeCoordinatorSimulationWithPedestrians tec){
        this.robot_id = robot_id;
        this.tec = tec;
        this.pathFileName = pathFileName;
    }

    @Override
    public void onTrackingStart() {
        // TODO Auto-generated method stub
    }

    @Override
    public void onTrackingFinished() {
        // TODO Auto-generated method stub
        try {
            ColorPrint.positive("Waiting time for robot " + robot_id);
            ColorPrint.positive(": " + tec.getRobotStoppageTime(robot_id));
        } catch (NullPointerException npe) {
            ColorPrint.error("Null Pointer Exception: " + npe.getMessage() + "\n");
            npe.printStackTrace();
            return;
        }
        //System.out.println("Does Robot" + robot_id + " have more missions? " + Missions.hasMissions(robot_id));
        writeToLogFile(String.valueOf(robot_id) + ", " + tec.getRobotStoppageTime(robot_id) + ", " + tec.getRobotStops(robot_id) + "\n", pathFileName + "_waiting_times.txt");
    }

    @Override
    public String[] onPositionUpdate() {
        // TODO Auto-generated method stub
        //FPA
        //System.out.println("There are now " + tec.getCurrentCriticalSections().size() + " critical sections");
        int currentCriticalSections = tec.getCurrentCriticalSections().size();
        if(currentCriticalSections != criticalSections) {
            criticalSections = currentCriticalSections;
            writeToLogFile(String.valueOf(criticalSections) + "\n", pathFileName + "_critical_sections.txt");
        }

        return null;
    }

    @Override
    public void onNewGroundEnvelope() {
        // TODO Auto-generated method stub

    }

    @Override
    public void beforeTrackingStart() {
        // TODO Auto-generated method stub

    }

    @Override
    public void beforeTrackingFinished() {
        // TODO Auto-generated method stub

    }

    public static synchronized void writeToLogFile(String token, String fileName) {
        File file = new File("journal_logs/" + fileName);
        FileWriter fileWriter = null;
        try {
            fileWriter = new FileWriter(file, true);
            fileWriter.write(token);
        } catch (IOException e) {
            ColorPrint.error("Couldn't write to file.");
            e.printStackTrace();
        } finally {
            try {
                fileWriter.close();
            } catch (IOException e) {
                ColorPrint.error("Couldn't close file.");
                e.printStackTrace();
            }
        }

    }
}