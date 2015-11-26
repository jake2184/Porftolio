
class EventuallyPerfectFailureDetector extends PerfectFailureDetector implements IFailureDetector {

	EPFDProcess p;

	// Use the maxDelay for each process as a way of detecting failure
	long [] maxDelay;

	static final int delayAverage = Utils.DELAY;

	public EventuallyPerfectFailureDetector(Process p) {
		super(p);
		this.p = (EPFDProcess) p;
		maxDelay = new long[p.n + 1];
	}

	@Override
	public void begin(){
		super.begin();
		for(int i=1; i<=p.n; i++){
			// Set initial delay. 
			maxDelay[i] = 0;		
		}
	}

	@Override
	public void receive(Message m){
		//Utils.out(p.pid, m.toString()); 

		int source = m.getSource();

		// Unsuspect
		if(suspects.contains(source)){
			isUnsuspected(source);
		}

		long delay = System.currentTimeMillis() - Long.parseLong(m.getPayload());

		if(delay > maxDelay[source]){
			maxDelay[source] = delay;
		}
		// Update timeout as heartbeat periodicity and maxDelay for that process. 
		timeouts[source] = Delta + maxDelay[source];	

	}
	
	public void isUnsuspected(Integer process){
		// Unsuspect process, remove from suspect list
		suspects.remove(suspects.indexOf(process));
		Utils.out(p.pid, 
			String.format("P%d has been unsuspected at %s",process,
			 Utils.timeMillisToDateString(System.currentTimeMillis())));
		return;
	}
}
