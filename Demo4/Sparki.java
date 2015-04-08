import java.util.Arrays;
import java.util.Scanner;

import jssc.SerialPort;
import jssc.SerialPortException;
import jssc.SerialPortTimeoutException;

public class Sparki {

	public static final int SERVO_CENTER = 0;
	public static final int SERVO_LEFT = 90;
	public static final int SERVO_RIGHT = -90;
	
	private static final byte STATUS_OK        = 0;
	private static final byte MOVE_FORWARD     = 1;
	private static final byte MOVE_BACKWARD    = 2;
	private static final byte MOVE_LEFT        = 3;
	private static final byte MOVE_RIGHT       = 4;
	private static final byte SERVO            = 5;
	private static final byte REQ_PING         = 6;
	private static final byte REQ_WHEELS       = 7;
	private static final byte MOVE_STOP        = 8;

	private static final int WIDTH = 500;
	private static final int HEIGHT = 500;
	private static final int MAX_DISTANCE = 40;
	
	private String portName;
	private SerialPort serialPort;

	public double curX;
	public double curY;
	public double curTheta;
	public double servoTheta;
	
	Sparki(String comPort) {
		portName = comPort;
		serialPort = new SerialPort(portName);
		curTheta = 45;
		servoTheta = 0;
		curX = HEIGHT/2;
		curY = WIDTH/2;
	}
	
	public boolean connect() {
		System.out.println("Connecting to " + portName);
		try {
			serialPort.openPort();
			serialPort.setParams(9600, 8, 1, 0);
			System.out.println("Connected");
			this.servo(SERVO_CENTER);
		} catch (SerialPortException e) {
			e.printStackTrace();
			return false;
		}
		return true;
	}

	public void disconnect() {
		System.out.println("Disconnecting " + portName);
		try {
			serialPort.closePort();
		} catch (SerialPortException e) {
			e.printStackTrace();
		}
	}

	/**
	 * @param distance should be between 0 - 255
	 */
    public void moveForward() {
        try {
            serialPort.writeByte(MOVE_FORWARD);
        } catch (SerialPortException e) {
            e.printStackTrace();
        }
    }

    public void moveBackward() {
        try {
            serialPort.writeByte(MOVE_BACKWARD);
        } catch (SerialPortException e) {
            e.printStackTrace();
        }
    }

    public void moveLeft() {
        try {
            serialPort.writeByte(MOVE_LEFT);
        } catch (SerialPortException e) {
            e.printStackTrace();
        }
    }

    public void moveRight() {
        try {
            serialPort.writeByte(MOVE_RIGHT);
        } catch (SerialPortException e) {
            e.printStackTrace();
        }
    }

    public void moveStop() {
        try {
            serialPort.writeByte(MOVE_STOP);
        } catch (SerialPortException e) {
            e.printStackTrace();
        }
    }
	

    /**
     * 
     * @param angle should be between -90 to +90
     */
    public void servo(int angle) {
        if(angle < -90 || angle > 90) {
            throw new IllegalArgumentException("Invalid servo angle: " + angle);
        }
        angle *= -1;
        angle += 90;
	
        try {
            // Send OPCODE and angle
            serialPort.writeByte(SERVO);
            serialPort.writeByte((byte)angle);
        } catch (SerialPortException e) {
            e.printStackTrace();
        }
    }
	
    public int ping() {
        int distance = -1;
        try {
            // Send OPCODE and angle
            serialPort.writeByte(REQ_PING);
            
            // Check response
            String input = readString();
            //System.out.println("ping " + input);
            return Integer.valueOf(input);
        } catch (SerialPortException e) {
            e.printStackTrace();
        }
	return -1;
    }
    
    public int[] totalTravel() {
        int[] values = new int[2];
        try {
            // Send OPCODE and angle
            serialPort.writeByte(REQ_WHEELS);
            String input = readString();
            String[] svalues = input.split(" ");
            values[0] = Integer.valueOf(svalues[0]);
            values[1] = Integer.valueOf(svalues[1]);
        } catch (SerialPortException e) {
            e.printStackTrace();
        }
	return values;
    }
    
    public double getX(){
    	return curX;
    }
    
    public double getY(){
    	return curY;
    }

    public double getCurTheta()
    {
	return curTheta;
    }

    public double getServoTheta()
    {
	return servoTheta;
    }
    
    /**
     * All responses from Sparki are *-terminated, to ensure complete
     * reading regardless of message length.  This function reads up
     * to the * and returns the string read.
     */
    private String readString() {
        String input = ""; String last = "";
        try {
            while (true) {
                last = serialPort.readString(1,1000); 
                if (last.equals("*")) break;
                input += last;
            }
        } catch (SerialPortException e) {
            e.printStackTrace();
        } catch (SerialPortTimeoutException e) {
            System.err.print(e);
        }
        return input;
    }

    /**
     * 
     * 
     * @param time Delay in milliseconds. Should be between 0 - 25500
     */
    public void delay(int time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public static void getPrintTravel(Sparki s) {
        int[] wheels = s.totalTravel();
        System.out.println("Left: " + wheels[0] + " Right: " + wheels[1]);
    }
        
    public static void main(String[] args) {
        Sparki sparki = new Sparki("/dev/cu.ArcBotics-DevB");
        boolean connected = sparki.connect();

        if(connected) {
        	MapWindow mw = new MapWindow(WIDTH, HEIGHT, 0.01);

            	while(true) 
		{
            		int ping = sparki.ping();
            		System.out.println(ping);
			
			double side = ping * Math.tan(7*Math.PI/180);
			double p1x = sparki.getX();
			double p0x = sparki.getX() - side;
			double p2x = sparki.getX() + side;
			double p1y = sparki.getY();
			double p0y = sparki.getY() + ping;
			double p2y = sparki.getY() + ping;

			System.out.println("p0x: " + p0x + " p0y: " + p0y + " p1x: " + p1x + " p1y: " + p1y + " p2x: " + p2x + " p2y: " + p2y);
			double area = (.5)*(-p1y*p2x + p0y*(-p1x + p2x) + p0x*(p1y - p2y) + p1x*p2y);
			System.out.println(area);
			
			p0x -= p1x;
			p2x -= p1x;
			p0y -= p1y;
			p2y -= p1y;

			double curTheta = sparki.getCurTheta();
			double servoTheta = sparki.getServoTheta();
			double newp0x = p0x * Math.cos((curTheta + servoTheta) * Math.PI/180) - p0y * Math.sin((curTheta + servoTheta) * Math.PI/180);
			double newp0y = p0y * Math.cos((curTheta + servoTheta) * Math.PI/180) - p0x * Math.sin((curTheta + servoTheta) * Math.PI/180);
			
			double newp2x = p2x * Math.cos((curTheta + servoTheta) * Math.PI/180) - p2y * Math.sin((curTheta + servoTheta) * Math.PI/180);
			double newp2y = p2y * Math.cos((curTheta + servoTheta) * Math.PI/180) - p2x * Math.sin((curTheta + servoTheta) * Math.PI/180);
			
			p0x = newp0x;
			p2x = newp2x;
			p0y = newp0y;
			p2y = newp2y;

			p0x += p1x;
			p2x += p1x;
			p0y += p1y;
			p2y += p1y;
			
			System.out.println("p0x: " + p0x + " p0y: " + p0y + " p1x: " + p1x + " p1y: " + p1y + " p2x: " + p2x + " p2y: " + p2y);
			
			for(int x = 0; x < WIDTH; x++)
			{
				for(int y = 0; y < HEIGHT; y++)
				{

					double s = 1/(2*area)*(p0y*p2x - p0x*p2y + (p2y - p0y)*x + (p0x - p2x)*y);
					double t = 1/(2*area)*(p0x*p1y - p0y*p1x + (p0y - p1y)*x + (p1x - p0x)*y);
					
					//System.out.println("S: " + s + " T:" + t + " Area: " + area);
					if( s<0 || t<0 || (1-s-t) < 0)
						continue;

					double ydiff = y - sparki.getY();
					double xdiff = x - sparki.getX();
					double theta = Math.atan2(ydiff, xdiff) * 180 / Math.PI;
					double distance = Math.sqrt(Math.pow(ydiff, 2) + Math.pow(xdiff, 2));
				//	System.out.println("XCurr: " + sparki.getX() + " YCurr: " + sparki.getY());
				//	System.out.println("X: " + x + " Y: " + y);
				//	System.out.println("YDiff: " + ydiff + " Theta: " + theta);
					
					if(theta <= 2) 
						mw.newEvidence(y, x, 0.8);
					else if(theta > 2 && theta <= 4) 
						mw.newEvidence(y, x, 0.9);
					else if (theta > 4 && theta <= 7) 
						mw.newEvidence(y, x, 0.95);	
						
					if(distance < 10)
						mw.newEvidence(y, x, 0.7);	
					else if(distance <= 20)
						mw.newEvidence(y, x, 0.8);
					else if(distance <= 30)
						mw.newEvidence(y, x, 0.85);
					else if(distance <= MAX_DISTANCE)
						mw.newEvidence(y, x, 0.9);
					
					//sparki.delay(1000);
				}
			}
			mw.reColor();
			sparki.delay(1000);
            	}
    	    }
	}
}	 
