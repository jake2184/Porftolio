

import java.io.*;
import java.net.*;
import java.util.*;

class PerfectFailureDetector implements IFailureDetector {

	// Although Process works, explicit PFDProcess is preferred
	PFDProcess p;
	long [] timeouts;
	LinkedList<Integer> suspects;
	// Separate timers so separate threads for timeouts and heartbeats
	Timer t, timeoutTimer;


	// How regularly we update and check timeout. 
	// Lower number makes it more complete, but less accurate
	static final int timeoutSchedule = 100; // milliseconds
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
						isSuspected(i);
						timeouts[i] = 0;
					}
				}
			}
		}
	}


	public PerfectFailureDetector(Process p) {
		this.p = (PFDProcess) p;
		timeouts = new long [p.n + 1];
		t = new Timer();
		timeoutTimer = new Timer();
		suspects = new LinkedList<Integer>();
	}

	public void begin() {
		t.schedule(new PeriodicTask(), 0, Delta);
		for(int i=0; i<=p.n; i++){
			// Set initial timeouts
			timeouts[i] = Delta + messageDelay;		
		}
		timeoutTimer.schedule(new CheckTimeouts(), 0, timeoutSchedule);
	}

	public void receive(Message m){
		//Utils.out(p.pid, m.toString());

		timeouts[m.getSource()] = Delta + messageDelay; 

	}
	
	public boolean isSuspect(Integer pid) {
		return suspects.contains(pid);
	}

	public void isSuspected(Integer process) {
		// If not already on suspect list, add
		if(!suspects.contains(process)){
			suspects.add(process);
			Utils.out(p.pid, 
				String.format("P%d has been suspected at %s",process,
				 Utils.timeMillisToDateString(System.currentTimeMillis())));
		}
		return;
	}
}
