package se.oru.coordination.coordination_oru.tests.testsPedestrians;

import java.io.File;
import java.io.FilenameFilter;
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
import se.oru.coordination.coordination_oru.simulation2D.PedestrianForwardModel;
import se.oru.coordination.coordination_oru.simulation2D.PedestrianTrajectory;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulationWithPedestrians;
import se.oru.coordination.coordination_oru.util.ColorPrint;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.RVizVisualization;

@DemoDescription(desc = "One-shot navigation of several pedestrians and a robot coordinating on static paths that overlap in a straight portion.")
public class MultiplePedestriansAndRobot {

    public static void main(String[] args) throws InterruptedException {

        double MAX_ACCEL = 1.0;
        double MAX_VEL = 1.0;

        final double threshold = 2.0;

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

                if((cs.getTe1Start() - robotReport1.getPathIndex()) < threshold && (cs.getTe2Start() - robotReport2.getPathIndex()) < threshold) {

                    // If o2 is Human, he passes first.
                    if (tec.isUncontrollable(o2.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID())) {
                        returnValue = 1;
                    }

                    // If o1 is Human, he passes first.
                    else if (tec.isUncontrollable(o1.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID())) {
                        returnValue = -1;
                    }

                    else
                        returnValue = ((cs.getTe1Start() - robotReport1.getPathIndex()) - (cs.getTe2Start() - robotReport2.getPathIndex()));
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

        ArrayList<Integer> nums = new ArrayList<Integer>();
        String pedestrianPathDir = "pedsim_testing_1-1";
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

        //JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
        RVizVisualization viz = new RVizVisualization();
        viz.setMap("maps/pedsim_office.yaml");
        int[] nums_primitive = new int[nums.size()];
        for (int i = 0; i < nums_primitive.length; i++) {
            nums_primitive[i] = nums.get(i).intValue();
        }
        RVizVisualization.writeRVizConfigFile(nums_primitive);
        //BrowserVisualization viz = new BrowserVisualization();
        //viz.setInitialTransform(39, -1.8, 1.4);
        tec.setVisualization(viz);
        
        for (int i = 0; i < nums.size(); i++) {

            // One robot. Others behave as pedestrians.
            if (nums.get(i) != 1729) {
                tec.setFootprint(nums.get(i), pedestrianFootprint);
                tec.addUncontrollableRobots(nums.get(i));
                tec.setForwardModel(nums.get(i), new PedestrianForwardModel());

                PedestrianTrajectory p1 = new PedestrianTrajectory(pedestrianPathDir + "/person" + nums.get(i) + ".txt");
                tec.addPedestrianTrajectory(nums.get(i), p1);
                tec.placeRobot(nums.get(i), p1.getPose(0));
                Mission m1 = new Mission(nums.get(i), p1.getPoseSteeringAsArray());
                tec.addMissions(m1);
            } else {
                tec.setFootprint(nums.get(i), f1, f2, f3, f4);
                tec.setForwardModel(nums.get(i), new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTrackingPeriod(), tec.getTemporalResolution()));
                PoseSteering[] robotPath = Missions.loadPathFromFile(pedestrianPathDir + "/person1729.txt");
                tec.placeRobot(nums.get(i), robotPath[0].getPose());
                Mission m1 = new Mission(nums.get(i), robotPath);
                tec.addMissions(m1);
            }
            
            // Add a tracking callback for each ID
            int finalI = i;
            tec.addTrackingCallback(nums.get(i), new TrackingCallback() {
				
				@Override
				public void onTrackingStart() {
					// TODO Auto-generated method stub
				}
				
				@Override
				public void onTrackingFinished() {
					// TODO Auto-generated method stub
					ColorPrint.positive("Waiting time for robot " + nums.get(finalI) + ": " + tec.getRobotStoppageTime(nums.get(finalI)));
				}
				
				@Override
				public String[] onPositionUpdate() {
					// TODO Auto-generated method stub
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

            Thread.sleep(200);
        }

        tec.computeCriticalSections();
        tec.startTrackingAddedMissions();
    }
    
}
