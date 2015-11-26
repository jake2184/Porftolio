
import java.io.*;
import java.net.*;
import java.util.*;

class PFDProcess extends Process {

	protected IFailureDetector detector;

	public PFDProcess (String name, int id, int size){
		super(name, id, size);
		detector = new PerfectFailureDetector(this);
	}

	public void begin() {
		detector.begin();
	}

	public synchronized void receive (Message m) {
		String type = m.getType();
		if (type.equals("heartbeat")){
			detector.receive(m);
		}
	}

	public static void main(String [] args){
		PFDProcess p = new PFDProcess (args[0], Integer.parseInt(args[1]),Integer.parseInt(args[2]));
		p.registeR();
		p.begin();	
	}

}
