
class EventuallyStrongFailureDetector extends EventuallyPerfectFailureDetector implements IFailureDetector {

	ESFDProcess p;

	static final int delayAverage = Utils.DELAY;

	public EventuallyStrongFailureDetector(ESFDProcess p) {
		super((Process)p);
		this.p = (ESFDProcess) p;
	}

	@Override
	public void isSuspected(Integer process) {
		// If not already on suspect list, add
		if(!suspects.contains(process)){
			suspects.add(process);	
			Utils.out(p.pid, 
				String.format("P%d has been suspected at %s",process,
				 Utils.timeMillisToDateString(System.currentTimeMillis())));
			// Notify process as it may be waiting	
			p.notifySuspect();
		}
		return;
	}

}
