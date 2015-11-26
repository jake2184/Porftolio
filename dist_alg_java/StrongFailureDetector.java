
class StrongFailureDetector extends PerfectFailureDetector implements IFailureDetector {

	SFDProcess p;

	public StrongFailureDetector(SFDProcess p) {		
		super(p);
		this.p = p;
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
