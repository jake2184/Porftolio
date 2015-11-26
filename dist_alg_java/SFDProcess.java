
import java.io.*;
import java.net.*;
import java.util.*;

class SFDProcess extends PFDProcess {

	private int leader;

	private int messageReceived = 0;
	Message storedMessage;

	public SFDProcess (String name, int id, int size){
		super(name, id, size);
		detector = new StrongFailureDetector(this);
	}
	
	@Override
	public void begin() {
		super.begin();
		consensus();
	}

	@Override
	public synchronized void receive (Message m) {
		String type = m.getType();
		if (type.equals("heartbeat")){
			detector.receive(m);
		} else if (type.equals("consensus")){
			// We have received new leader, notify collect
			storedMessage = m;
			messageReceived = m.getSource();
			notifyAll();
		}
	}

	public synchronized void notifySuspect () {
		// If FD suspects a process, notify this object as collect could be waiting
		notifyAll();
	}

	private synchronized void consensus(){
		int leader = pid;
		// n rounds in algorithm. Should result with leader being highest non-faulty pid
		for(int i = 1; i <= this.n; i++){
			if( i == pid ){
				// Coordinator of the round. Broadcast current leader
				broadcast("consensus", leader + "," + Integer.toString(i) );
			} else {
				// Collect value from coordinator
				if( collect("consensus", i) ){ 
					MessageInfo messInfo = new MessageInfo(storedMessage);
					leader = messInfo.data.get(0);
				}
			}
		}
		Utils.out(pid, String.format("Decided on %s", leader));
	}

	// Specification suggests type is passed to collect, and is included for completeness.
	// It is not needed, unless the message type is checked. We have not included this check
	// as it will slow down execution and the algorithm functions without this check
	private synchronized boolean collect (String type, int i){
		// Wait whilst process isn't suspect and we haven't received a message
		while(!detector.isSuspect(i) && messageReceived != i){
			try {
				wait();
			} catch (InterruptedException e) {
				//System.out.println("ERROR"); 
			}
		}			

		// Return bool depending on if we received a message or not
		if(messageReceived == i){ 
			messageReceived = 0;
			return true;
		} else {
			return false;
		}
	}

 	
	public static void main(String [] args){
		SFDProcess p = new SFDProcess (args[0], Integer.parseInt(args[1]), Integer.parseInt(args[2]));
		p.registeR();
		p.begin();	
	}

}
