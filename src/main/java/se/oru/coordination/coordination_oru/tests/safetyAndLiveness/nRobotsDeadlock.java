package se.oru.coordination.coordination_oru.tests.safetyAndLiveness;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Comparator;
import java.util.HashMap;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.BrowserVisualization;
import se.oru.coordination.coordination_oru.util.JTSDrawingPanelVisualization;
import se.oru.coordination.coordination_oru.util.Missions;
import se.oru.coordination.coordination_oru.util.RVizVisualization;

@DemoDescription(desc = "Coordination with deadlock-inducing ordering heuristic (paths obtained with the ReedsSheppCarPlanner).")
public class nRobotsDeadlock {
	
	private static void initStat(String fileName, String stat) {
        try {
        	//Append to file
            PrintWriter writer = new PrintWriter(new FileOutputStream(new File(fileName), false)); 
            writer.println(stat);
            writer.close();
        }
        catch (Exception e) { e.printStackTrace(); }
	}
	
	static public void writeStat(String fileName, String stat) {
        try {
        	//Append to file
            PrintWriter writer = new PrintWriter(new FileOutputStream(new File(fileName), true)); 
            writer.println(stat);
            writer.close();
        }
        catch (Exception e) { e.printStackTrace(); }
	}
		
	protected static final int NUMBER_ROBOTS = 40;

	public static void main(String[] args) throws InterruptedException {
		
		double MAX_ACCEL = 1.0;
		double MAX_VEL = 4.0;
		double radius = 20;
		
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(3000,1000,MAX_VEL,MAX_ACCEL);
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				int robot1ID = o1.getRobotReport().getRobotID();
				int robot2ID = o2.getRobotReport().getRobotID();
				if (robot2ID == nRobotsDeadlock.NUMBER_ROBOTS && robot1ID == 1) return 1;
				if (robot1ID == nRobotsDeadlock.NUMBER_ROBOTS && robot2ID == 1) return -1;
				return robot1ID > robot2ID ? 1 : -1; 
				}
		});
		
		//You probably also want to provide a non-trivial forward model
		//(the default assumes that robots can always stop)
		int[] robotIDs =  new int[NUMBER_ROBOTS];
		for (int i = 1; i <= nRobotsDeadlock.NUMBER_ROBOTS; i++) {
			robotIDs[i-1] = i;
			tec.setForwardModel(i, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getTemporalResolution(), tec.getControlPeriod(), tec.getTrackingPeriod()));
		}
		
		//comment out following (or set to true) to make the coordinator attempt to break the deadlock
		//tec.setBreakDeadlocks(false);
		//tec.setBreakDeadlocksByReordering(true);
		tec.setAvoidDeadlocksGlobally(true);
		tec.setCheckCollisions(true);

		Coordinate footprint1 = new Coordinate(-0.25,0.25);
		Coordinate footprint2 = new Coordinate(0.25,0.25);
		Coordinate footprint3 = new Coordinate(0.25,-0.25);
		Coordinate footprint4 = new Coordinate(-0.25,-0.25);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);
		
		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		
		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		RVizVisualization viz = new RVizVisualization();
		//RVizVisualization.writeRVizConfigFile(robotIDs);
		//JTSDrawingPanelVisualization viz = new JTSDrawingPanelVisualization();
		//BrowserVisualization viz = new BrowserVisualization();
		//viz.setInitialTransform(73, 22, 16);
		tec.setVisualization(viz);
		
		//tec.setUseInternalCriticalPoints(false);
		
		//For debugging purposes.
		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		//Instantiate a simple motion planner
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setRadius(0.2);
		rsp.setFootprint(tec.getDefaultFootprint());
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.5);
		tec.setDefaultMotionPlanner(rsp);
			
		//Place robots in their initial locations (looked up in the data file that was loaded above)
		// -- creates a trajectory envelope for each location, representing the fact that the robot is parked
		// -- each trajectory envelope has a path of one pose (the pose of the location)
		// -- each trajectory envelope is the footprint of the corresponding robot in that pose
		HashMap<Integer,Pose> startPoses = new HashMap<Integer,Pose>();
		HashMap<Integer,Pose> goalPoses = new HashMap<Integer,Pose>();
		ArrayList<PoseSteering[]> paths = new ArrayList<PoseSteering[]>();
		
		double theta = 0.0;
		for (int i = 0; i < NUMBER_ROBOTS; i++) {
			double alpha = theta + i*Math.PI/NUMBER_ROBOTS;
			startPoses.put(robotIDs[i], new Pose(radius*Math.cos(alpha), radius*Math.sin(alpha), alpha));
			goalPoses.put(robotIDs[i], new Pose(radius*Math.cos(alpha+Math.PI), radius*Math.sin(alpha+Math.PI), alpha));
			tec.placeRobot(robotIDs[i], startPoses.get(robotIDs[i]));
			rsp.setStart(startPoses.get(robotIDs[i]));
			rsp.setGoals(goalPoses.get(robotIDs[i]));
			if (!rsp.plan()) throw new Error ("No path between " + startPoses.get(robotIDs[i]) + " and " + goalPoses.get(robotIDs[i]));
			Mission m = new Mission(robotIDs[i], rsp.getPath());
			Missions.enqueueMission(m);
			Mission m1 = new Mission(robotIDs[i], rsp.getPathInv());
			Missions.enqueueMission(m1);
		}
		
		final String statFilename = System.getProperty("user.home")+File.separator+"stats.txt";
		String header = "#";
		for (int robotID : robotIDs) header += (robotID + "\t");
		initStat(statFilename, header);

		
		//Start a mission dispatching thread for each robot, which will run forever
		for (final int robotID : robotIDs) {
			//For each robot, create a thread that dispatches the "next" mission when the robot is free
			
			Thread t = new Thread() {
				@Override
				public void run() {
					boolean firstTime = true;
					int sequenceNumber = 0;
					int totalIterations = 20;
					if (robotID%2 == 0) totalIterations = 19;
					long startTime = Calendar.getInstance().getTimeInMillis();
					while (true && totalIterations > 0) {
						synchronized(tec.getSolver()) {
							Mission m = Missions.getMission(robotID, sequenceNumber);
							if (tec.addMissions(m)) {
								if (!firstTime) {
									long elapsed = Calendar.getInstance().getTimeInMillis()-startTime;
									String stat = "";
									for (int i = 1; i < robotID; i++) stat += "\t";
									stat += elapsed;
									writeStat(statFilename, stat);
								}
								startTime = Calendar.getInstance().getTimeInMillis();
								firstTime = false;
								sequenceNumber = (sequenceNumber+1)%Missions.getMissions(robotID).size();
								totalIterations--;
							}
						}
						//Sleep for a little
						try { Thread.sleep(tec.getControlPeriod()); }
						catch (InterruptedException e) { e.printStackTrace(); }
					}
					System.out.println("Robot" + robotID + " is done!");
				}
			};
			//Start the thread!
			t.start();
		}
		//Sleep for a little (2 sec)
		try { Thread.sleep(tec.getControlPeriod()); }
		catch (InterruptedException e) { e.printStackTrace(); }
	}
	
}
