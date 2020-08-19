package se.oru.coordination.coordination_oru.tests.testsPedestrians;

import java.io.File;
import java.io.FileWriter;
import java.io.FilenameFilter;
import java.io.IOException;
import java.util.*;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.TrackingCallback;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.simulation2D.*;
import se.oru.coordination.coordination_oru.util.ColorPrint;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.RVizVisualization;

@DemoDescription(desc = "One-shot navigation of several pedestrians and a robot coordinating on static paths that overlap in a straight portion.")
public class MultiplePedestriansAndRobot {

    public static void main(String[] args) throws InterruptedException {

        double MAX_ACCEL = 1.0;
        double MAX_VEL = 1.0;
        String pathFileName = "RRTStar_noup_0";
        String pedestrianPathDir = "chitt_tests/pedestrians_atc_testing_1114_1352866100";
        String robotPathDir = "chitt_tests/LongPathNew/T1_1352866100";

        final double threshold = 5.0;

        //Instantiate a trajectory envelope coordinator.
        //The TrajectoryEnvelopeCoordinatorSimulation implementation provides
        // -- the factory method getNewTracker() which returns a trajectory envelope tracker
        // -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
        //You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
        final TrajectoryEnvelopeCoordinatorSimulationWithPedestrians tec = new TrajectoryEnvelopeCoordinatorSimulationWithPedestrians(MAX_VEL, MAX_ACCEL);
        tec.addComparator(new Comparator<RobotAtCriticalSection>() {
            @Override
            public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
                int returnValue = 1;
                CriticalSection cs = o1.getCriticalSection();
                RobotReport robotReport1 = o1.getTrajectoryEnvelopeTracker().getRobotReport();
                RobotReport robotReport2 = o2.getTrajectoryEnvelopeTracker().getRobotReport();

                if(tec.isUncontrollable(o2.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID())) {
                    double o2DistToCP = ((TrajectoryEnvelopeTrackerPedestrian) o2.getTrajectoryEnvelopeTracker()).computeDistance(robotReport2.getPathIndex(), cs.getTe2Start());
                    if(o2DistToCP < threshold) returnValue = 1;
                    else returnValue = ((cs.getTe1Start() - robotReport1.getPathIndex()) - (cs.getTe2Start() - robotReport2.getPathIndex()));
                }

                else if (tec.isUncontrollable(o1.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID())) {
                    double o1DistToCP = ((TrajectoryEnvelopeTrackerPedestrian) o1.getTrajectoryEnvelopeTracker()).computeDistance(robotReport1.getPathIndex(), cs.getTe1Start());
                    if(o1DistToCP < threshold) returnValue = -1;
                    else returnValue = ((cs.getTe1Start() - robotReport1.getPathIndex()) - (cs.getTe2Start() - robotReport2.getPathIndex()));
                }

                // If both are robots. We don't need it now, but for the sake of future prosperity...
                else { returnValue = ((cs.getTe1Start() - robotReport1.getPathIndex()) - (cs.getTe2Start() - robotReport2.getPathIndex())); }

                return returnValue;
            }
        });
        tec.addComparator(new Comparator<RobotAtCriticalSection>() {
            @Override
            public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
                return (o2.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID() - o1.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID());
            }
        });

        // Pedestrian Footprints
		// A small circle of radius 0.3m
		Coordinate[] pedestrianFootprint = {
				new Coordinate(0.3000, 0.0000),
				new Coordinate(0.2427, 0.1763),
				new Coordinate(0.0927, 0.2853),
				new Coordinate(-0.0927, 0.2853),
				new Coordinate(-0.2427, 0.1763),
				new Coordinate(-0.3000, 0.0000),
				new Coordinate(-0.2427, -0.1763),
				new Coordinate(-0.0927, -0.2853),
				new Coordinate(0.0927, -0.2853),
				new Coordinate(0.2427, -0.1763)
		};

        // Robot Footprints
        // [[0.5, 0.2], [0.5, -0.2], [-0.2, -0.2], [-0.2, 0.2]]
        Coordinate f1 = new Coordinate(0.5, 0.2);
        Coordinate f2 = new Coordinate(0.5, -0.2);
        Coordinate f3 = new Coordinate(-0.2, -0.2);
        Coordinate f4 = new Coordinate(-0.2, 0.2);

        //Need to setup infrastructure that maintains the representation
        tec.setupSolver(0, 100000000);

        final ArrayList<Integer> nums = new ArrayList<Integer>();
        // Filter names
        FilenameFilter matchingNameFilter = new FilenameFilter() {
            @Override
            public boolean accept(File file, String name) {
                return name.contains("person");
            }
        };

        File pedestrianDir = new File(pedestrianPathDir);
        for (final File f : pedestrianDir.listFiles(matchingNameFilter)) {
            nums.add(Integer.parseInt(f.getName().split("person")[1].split(".txt")[0]));
        }

        // Add robot
        nums.add(1729);

        //JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
        RVizVisualization viz = new RVizVisualization();
        viz.setMap("maps/atc.yaml");
        int[] nums_primitive = new int[nums.size()];
        for (int i = 0; i < nums_primitive.length; i++) {
            nums_primitive[i] = nums.get(i);
        }
        RVizVisualization.writeRVizConfigFile(nums_primitive);
        //BrowserVisualization viz = new BrowserVisualization();
        //viz.setInitialTransform(39, -1.8, 1.4);
        tec.setVisualization(viz);

        PedestrianTrajectory[] pedestrianTrajectories = new PedestrianTrajectory[nums_primitive.length];
        long startTime = 0;
        
        for (int i = 0; i < nums.size(); i++) {

            // One robot. Others behave as pedestrians.
            if (nums.get(i) != 1729) {
                pedestrianTrajectories[i] = new PedestrianTrajectory(pedestrianPathDir + "/person" + nums.get(i) + ".txt");
            } else {
                tec.setFootprint(nums.get(i), f1, f2, f3, f4);
                tec.setForwardModel(nums.get(i), new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTrackingPeriod(), tec.getTemporalResolution()));
                PoseSteering[] robotPath = Missions.loadPathFromFile(robotPathDir + "/" + pathFileName + ".path");
                tec.placeRobot(nums.get(i), robotPath[0].getPose());
                Mission m1 = new Mission(nums.get(i), robotPath);
                tec.addMissions(m1);
                tec.computeCriticalSections();
                Thread.sleep(5000);
                tec.startTrackingAddedMissions();
                startTime = tec.getCurrentTimeInMillis();
            }
        }

        int added_missions = 0;

        while(added_missions != nums_primitive.length) {
            long timeNow = tec.getCurrentTimeInMillis();

            for (int i = 0; i < nums.size(); i++) {
                PedestrianTrajectory pI = pedestrianTrajectories[i];
                // One robot. Others behave as pedestrians.
                // When the current time is greater than the start time of a pedestrian, we start that pedestrian
                if (nums.get(i) != 1729 && timeNow > startTime + pI.getTimeStamps().get(0) ) {
                    tec.setFootprint(nums.get(i), pedestrianFootprint);
                    tec.addUncontrollableRobots(nums.get(i));
                    tec.setForwardModel(nums.get(i), new PedestrianForwardModel());

                    tec.addPedestrianTrajectory(nums.get(i), pI);
                    tec.placeRobot(nums.get(i), pI.getPose(0));
                    Mission m1 = new Mission(nums.get(i), pI.getPoseSteeringAsArray());
                    ColorPrint.info("Adding mission for Robot " + nums.get(i));
                    tec.addMissions(m1);
                }

                final int finalI = i;
                // Add a tracking callback for each ID
                tec.addTrackingCallback(nums.get(i), new TrackingCallback() {

                    @Override
                    public void onTrackingStart() {
                        // TODO Auto-generated method stub
                    }

                    @Override
                    public void onTrackingFinished() {

                        ColorPrint.positive("Waiting time for robot " + nums.get(finalI) + ": " + tec.getRobotStoppageTime(nums.get(finalI)));

                        //FPA
                        System.out.println("Does Robot" + nums.get(finalI) + " have more missions? " + Missions.hasMissions(nums.get(finalI)));
                        MyTrackingCallback.writeToLogFile(String.valueOf(nums.get(finalI)) + ", " + tec.getRobotStoppageTime(nums.get(finalI)) + ", " + tec.getRobotStops(nums.get(finalI)) + "\n", pathFileName + "_waiting_times.txt");

                    }

                    @Override
                    public String[] onPositionUpdate() {
                        int currentCriticalSections = tec.getCurrentCriticalSections().size();
                        if (currentCriticalSections != MyTrackingCallback.criticalSections) {
                            MyTrackingCallback.criticalSections = currentCriticalSections;
                            MyTrackingCallback.writeToLogFile(String.valueOf(MyTrackingCallback.criticalSections) + "\n", pathFileName + "_critical_sections.txt");
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
                });

                Thread.sleep(100);
            }

            tec.computeCriticalSections();
            tec.startTrackingAddedMissions();
        }
    }
    
}
