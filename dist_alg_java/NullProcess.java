/* Copyright (c) 2013-2015, Imperial College London
 * All rights reserved.
 *
 * Distributed Algorithms, CO347
 */

class NullProcess extends Process {
	
	public NullProcess (String name, int pid, int n) {
		super(name, pid, n);
	}
	
	public void begin () {
		return ;
	}
	
	public synchronized void receive (Message m) {
		return ;
	}
	
	public static void main (String [] args) {
		String name = args[0];
		int id = Integer.parseInt(args[1]);
		int  n = Integer.parseInt(args[2]);
		NullProcess p = new NullProcess(name, id, n);
		p.registeR ();
		Utils.out(p.pid, String.format("P%d has begun at %s", p.pid, 
			Utils.timeMillisToDateString(System.currentTimeMillis())));
		p.begin ();
	}
}
