import lejos.nxt.*;
import lejos.util.Delay;
import lejos.robotics.navigation.*;
import lejos.robotics.mapping.*;
import lejos.robotics.pathfinding.*;
import lejos.robotics.localization.*;
import lejos.robotics.Color;
import lejos.nxt.ColorSensor;
import lejos.robotics.RegulatedMotor;
import lejos.geom.*;
import java.lang.Math;
import lejos.nxt.UltrasonicSensor;


public class Taxi {
    // Sensors
    static LightSensor lightRight = new LightSensor(SensorPort.S1, true);
    static LightSensor lightLeft = new LightSensor(SensorPort.S4, true);
    static UltrasonicSensor sonic = new UltrasonicSensor(SensorPort.S2);
    static ColorSensor colorSensor = new ColorSensor(SensorPort.S3);;

    // Brick
    static DifferentialPilot pilot = new DifferentialPilot(2.205 * 2.56, 4.527 * 2.56, Motor.C, Motor.A, false);
    static DifferentialPilot pilot1 = new DifferentialPilot(2.205 * 2.56, 10.5, Motor.C, Motor.A, false);
    static OdometryPoseProvider poseProvider = new OdometryPoseProvider(pilot1);
    static Navigator navigator = new Navigator(pilot1);

    // Vars
    static int passengerDistBigStep = 5;
    static int passengerDistSmallStep = 2;
    static int openAngle = 180;
    static int closeAngle = -2; // Cumulative angle
    static int cylinderDist = 15;
    static int rotPassengerAng = 82;
    static int rotOneEighty = 82;
    static int rotCounter = 27;
    // PID BLUE LINE
	static float KPB = 12.0f;
	static float KDB = 1.3f;
	static float ulinhaB = 250.0f;
    // PID RETURN BLUE LINE
	static float KDA = 15.0f;
	static float KPA = 7.5f;
	static float ulinhaA = 150.0f;
    // Colors
	static int colorA = 56;
    static int blackValue = 50;
	static float errAntA = (float) colorA;

    //Small cilinder
    static boolean small_cilinder=false;

    
    public static Waypoint[] getLocationByID(int id) {
        Waypoint[][] mapPoints = new Waypoint[6][5];
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 5; j++) {
                mapPoints[i][j] = new Waypoint(15 + 29.5 * i,  12+ 30 * j);
            }
        }

        Waypoint FinalBakery= new Waypoint(103.5,19);
        Waypoint FinalCityHall=new Waypoint(44.5,65);
        Waypoint FinalDrugstore=new Waypoint (96.5,72);
        Waypoint FinalMuseum=new Waypoint(103.5,125);
        Waypoint FinalSchool = new Waypoint(44.5,19);
        Waypoint FinalLibrary = new Waypoint(44.5,125);
        Waypoint FinalPark = new Waypoint (155.5,72);
        Waypoint[] schoolPath = {
            mapPoints[0][1],
            mapPoints[1][1],//ponto (44.5,42)
            FinalSchool,
            //mapPoints[1][0], //ponto (44.5,12)
        };

        Waypoint[] bakeryPath = {
            mapPoints[0][1],
            mapPoints[1][1],
            mapPoints[2][1],
            mapPoints[3][1], //ponto (103.5,42)
            FinalBakery, //novo ponto (103.5,19)
            //mapPoints[3][0], //ponto(103.5,12)
        };

        Waypoint[] cityHallPath = {
            mapPoints[0][1],
            mapPoints[1][1],//ponto (44.5,42)
            FinalCityHall,
            //mapPoints[1][2], //ponto (44.5,72)
        };

        Waypoint[] drugstorePath = {
            mapPoints[0][1],
            mapPoints[1][1],
            mapPoints[2][1],
            mapPoints[2][2],//ponto(74,72)
            FinalDrugstore,
            //mapPoints[3][2], //ponto (103.5,72)
        };

        Waypoint[] libraryPath = {
            mapPoints[0][1],
            mapPoints[0][2],
            mapPoints[0][3],
            mapPoints[1][3], //ponto (44.5,102)
            FinalLibrary,
            //mapPoints[1][4], //ponto (44.5, 132)
        };

        Waypoint[] museumPath = {
            mapPoints[0][3],
            mapPoints[3][3], //ponto (103.5,102)
            FinalMuseum,
            //mapPoints[3][4], //ponto (103.5,132)
        };
    
        Waypoint[] parkPath = {
            mapPoints[0][1],
            mapPoints[1][1],
            mapPoints[2][1],
            mapPoints[3][1],
            mapPoints[4][1],
            mapPoints[4][2], // ponto (133, 72)
            FinalPark,
            //mapPoints[5][2],//ponto (162.5 , 72)
        };

        Waypoint[] realPath = parkPath;
        if (id == 0)
            realPath = schoolPath;
        else if (id == 1)
            realPath = bakeryPath;
        else if (id == 2)
            realPath = cityHallPath;
        else if (id == 3)
            realPath = drugstorePath;
        else if (id == 4)
            realPath = libraryPath;
        else if (id == 5)
            realPath = museumPath;
        return realPath;
    }

    public static int getCounterDir(int id) {
        if (id == 4||id == 2 || id == 5)
            return -1;
        return 1;
    }

    public static void ReturnBlueLine(){
        while(lightRight.readValue()>55){
            pilot.forward();
            
        }
        pilot.stop();
        pilot.rotate(90,false);

    }
    public static int getLocationByColor() {
        int locationID;
        Color color = colorSensor.getColor();
        // Green
        if (color.getGreen() > color.getRed() && color.getGreen() > color.getBlue()){
            if (!small_cilinder)
                locationID = 2;
            else
                locationID=6;
        }
        // Blue
        else if (color.getBlue() > color.getRed() && color.getBlue() > color.getGreen()){
            if (!small_cilinder)
                locationID = 5;
            else
                locationID= 0;
        }
        // Yellow
        else if (color.getRed() > color.getGreen() && color.getGreen() > color.getBlue()){
            if (!small_cilinder)
                locationID = 1;
            else 
                locationID=4;
        }
        // Red
        else
            locationID = 3;
        return locationID;
    }

    public static void openClaw() {
        Motor.B.rotateTo(-openAngle, false);
    }

    public static void closeClaw() {
        Motor.B.rotateTo(closeAngle, false);
    }

    public static void rotateZeroZero() {
        pilot.rotate(rotOneEighty, false);
        poseProvider.setPose(new Pose(0, 0, 0));
        navigator.setPoseProvider(poseProvider);
    }

    public static void deliverPassenger(int rotDir) {
        // Motor.B.rotateTo(40, false);
        pilot.rotate(rotDir * rotCounter, false);
        openClaw();
        pilot.rotate(-rotDir * rotCounter, false);
    }

    public static int grabPassenger() {
        pilot.stop();
        openClaw(); // prepare
        pilot.stop();
        pilot.rotate(rotPassengerAng, false); // look at passenger
        pilot.stop();
        pilot.travel(passengerDistBigStep, false); // get closer
        pilot.stop();
        int locationID = getLocationByColor(); // read color
        pilot.travel(passengerDistSmallStep, false); // get even closer
        pilot.stop();
        closeClaw(); // grab
        pilot.stop();
        pilot.travel(-(passengerDistBigStep + passengerDistSmallStep), false); // get closer
        pilot.stop();
        pilot.rotate(rotPassengerAng, false); // look at 0,0
        pilot.stop();
        return locationID;
    }

    public static void hardcodeFollowPath(Waypoint[] locationPath) {
        navigator.clearPath();
        for (int i = 0; i < locationPath.length; ++i)
            navigator.addWaypoint(locationPath[i]);
        navigator.followPath();
        while (navigator.isMoving());
    }

    public static void hardcodeTraceBackPath(Waypoint[] locationPath) {
        navigator.clearPath();
        pilot.travel(-23, false); // go backwards
        for (int i = locationPath.length - 3; i >= 0; --i)
            navigator.addWaypoint(locationPath[i]);
        navigator.followPath();
        while (navigator.isMoving());
    }

    public static void goToZero() {
        navigator.clearPath();
        navigator.goTo(0, 0);
        while (navigator.isMoving());
    }

    public static boolean nearCylinder() {
        int dist = sonic.getDistance();
        return cylinderDist <= dist;
    }

    // PID BLUE LINE --------------------------------------------------------
    public static float errPIDBlue(int val, float kp, float kd) {
		int diff = val - colorA;
		float prop = kp * diff;
		float diferential = kd * (errAntA - diff);
		errAntA = diff;
		return prop + diferential;
	}

    public static int pidBlueLine() {
        // Follow Blue Line
        while (true) {
            if (!nearCylinder())
                return 1;
            else if (lightRight.readValue()<40)
                return 0;
			float er = errPIDBlue(lightLeft.readValue(), KPB, KDB);
			int esq = (int) (ulinhaB - er);
			int dir = (int) (ulinhaB + er);
			Motor.C.setSpeed(esq);
			Motor.A.setSpeed(dir);
			Motor.C.forward();
            Motor.A.forward();
		}
    }

    // PID RETURN LINE --------------------------------------------------------
    public static void pidReturnLine() {
        errAntA = (float) colorA;
        while (true) {
            if (lightLeft.readValue() < blackValue) {
                Motor.A.stop();
                Motor.C.stop();
                break;
            }
			float er = errPIDBlue(lightRight.readValue(), KPA, KDA);
			int esq = (int) (ulinhaA + er);
			int dir = (int) (ulinhaA - er);
			Motor.C.setSpeed(esq);
			Motor.A.setSpeed(dir);
			Motor.C.forward();
            Motor.A.forward();
		}
    }


  public static void main(String[] args) {
    try {
        // ------------------
        System.out.println("Running!");  
        Button.waitForAnyPress();
        Delay.msDelay(500);

        // ------------------
        pilot.setTravelSpeed(15);
        pilot.setRotateSpeed(30);
        
        pilot1.setTravelSpeed(20);
        pilot1.setRotateSpeed(25);
        Motor.B.setSpeed(100);

        // ------------------
        while(true){
            int Find_Cilinder = pidBlueLine();
            if (Find_Cilinder==1){
                int locationID = grabPassenger();
                int rotDir = getCounterDir(locationID);
                Waypoint[] realPath = getLocationByID(locationID);
                System.out.println(locationID);
                pidReturnLine();
                rotateZeroZero();
                hardcodeFollowPath(realPath);
                deliverPassenger(rotDir);
                hardcodeTraceBackPath(realPath);
                ReturnBlueLine();
                pidReturnLine();
                closeClaw();
                pilot.rotate(2*rotOneEighty, false);
            }
            else if (!small_cilinder){
                small_cilinder=true;
                cylinderDist = 30; //precisa ajustar essa variável
                closeAngle = 5; //precisa ajustar essa variável
                pilot.rotate(2*rotOneEighty, false);
                pidReturnLine();
                pilot.rotate(2*rotOneEighty, false);
               
            }
            else
                break;
        }
        
    } catch (Exception e) {
        System.out.println(e);  
        Button.waitForAnyPress();
    }
  }
}   