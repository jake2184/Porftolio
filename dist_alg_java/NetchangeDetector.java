

import java.io.*;
import java.net.*;
import java.util.*;

class NetchangeDetector implements IFailureDetector {

	NetchangeProcess p;
	long [] timeouts;
	// Separate timers so separate threads for timeouts and heartbeats
	Timer t, timeoutTimer;


	// How regularly we update and check timeout. 
	// Lower number makes it more complete, but less accurate
	static final int timeoutSchedule = 500; // milliseconds
	static final int Delta = Utils.Delta;
	static final long messageDelay = (long)(1.1 * Utils.DELAY); // a priori knowledge, with 10% error margin due to overhead

	class PeriodicTask extends TimerTask {
		public void run(){
			p.broadcast("heartbeat", String.format("%d", System.currentTimeMillis()));
		}
	}

	// Regularly update and check timeouts
	class CheckTimeouts extends TimerTask {
		public void run(){
			for(int i=1; i<=p.n; i++){
				if(i != p.pid){
					// Reduce timeout by schedule time
					timeouts[i] -= timeoutSchedule;
					// If too late, suspect. Set timeout to zero to prevent 'underflow'
					if(timeouts[i] < 0){
						if(p.neighbours.contains(i)){
							isSuspected(i);
						}
						timeouts[i] = 0;
					}
				}
			}
		}
	}


	public NetchangeDetector(Process p) {
		this.p = (NetchangeProcess) p;
		timeouts = new long [p.n + 1];
		t = new Timer();
		timeoutTimer = new Timer();
	}

	public void begin() {
		t.schedule(new PeriodicTask(), 0, Delta);
		for(int i=0; i<=p.n; i++){
			// Set initial timeouts
			timeouts[i] = 2*Delta + messageDelay;		
		}
		timeoutTimer.schedule(new CheckTimeouts(), 0, timeoutSchedule);
	}

	public synchronized void receive(Message m){
		//Utils.out(p.pid, m.toString());

		if(!p.neighbours.contains(m.getSource())){
			newNeighbour(m.getSource());
		}

		timeouts[m.getSource()] = 2*Delta + messageDelay; 

	}
	
	public boolean isSuspect(Integer pid) {
		return !p.neighbours.contains(pid);
	}

	public synchronized void isSuspected(Integer process) {
		// Send 'closed' message to process

		Message m = new Message();
		m.setSource(p.pid);
		m.setDestination(p.pid);
		m.setType("closed");
		m.setPayload(Integer.toString(process));

		// unicast(m);
		p.receive(m);
	
		Utils.out(p.pid, 
			String.format("P%d has been removed as neighbour at %s",process,
			 Utils.timeMillisToDateString(System.currentTimeMillis())));
		
		return;
	}

	public synchronized void newNeighbour(Integer process){
		// Send 'open' message to process

		Message m = new Message();
		m.setSource(p.pid);
		m.setDestination(p.pid);
		m.setType("open");
		m.setPayload(Integer.toString(process));
		
		// unicast(m);
		p.receive(m);
	
		Utils.out(p.pid, 
			String.format("P%d has been added as neighbour at %s",process,
			 Utils.timeMillisToDateString(System.currentTimeMillis())));
	}
}
