

import java.io.*;
import java.net.*;
import java.util.*;

class NetchangeProcess extends Process {

	private IFailureDetector detector;


	Integer [] du;
	Integer [] nbu;
	Integer [] [] ndis;
	public ArrayList<Integer> neighbours;

	// Variables to deal with StableState checks and printing of routing information
	// When the table is printed, no mydist messages will have been sent for 1 to 2 times StateCheckPeriodicity
	boolean stableState = false;
	int stateCheckPeriodicity = 2000; // milliseconds. 
	boolean printed = false;

	Timer t;

	public NetchangeProcess (String name, int id, int size){
		super(name, id, size);
		detector = new NetchangeDetector(this);

		// Declare data structures
		du = new Integer [size+1];
		nbu = new Integer [size+1];
		ndis = new Integer [size+1] [size+1];

		neighbours = new ArrayList<Integer> ();

		// Initialise values
		for(int i=1; i<=size; i++){
			du[i] = size;
			nbu[i] = -1; // -1 = undefined
			for(int j=1; j<=size; j++){
				ndis[i][j]=size;
			}
		}
		du[pid] = 0; // dis to self is 0
		nbu[pid] = 0; // 0 = local
		ndis[pid][pid] = 0; // dis to self from self is 0
		
	}

	class CheckStableState extends TimerTask{
		public void run(){
			if(stableState){
				if(!printed){
					printRoutingTable();
					printed = true;
				}
			}
			else {
				printed = false;
			}
			stableState = true;
		}
	}

	public void begin() {
		detector.begin();	
		broadcast("mydist", pid + ",0");
		t = new Timer();
		t.schedule(new CheckStableState(),0, stateCheckPeriodicity);
	}

	public synchronized void receive (Message m) {
		String type = m.getType();
		if (type.equals("heartbeat")){
			detector.receive(m);
			return;
		} 
		MessageInfo mi = new MessageInfo(m);
		int v = mi.data.get(0);
		if (type.equals("mydist")){

			// Update distance info, and recompute own distance to v
			stableState = false;
			ndis[m.getSource()][v] = mi.data.get(1);
			recompute(v);
		}		
		else if (type.equals("closed")){
			stableState = false;
			neighbours.remove(neighbours.indexOf(v));
			// Recompute shortest distance to all processes
			for(int i=1; i<=n; i++){
				recompute(i);
			}
		}
		else if (type.equals("open")){
			neighbours.add(v);

			// Send distance info to new neighbour
			Message message = new Message();
			message.setSource(pid);
			message.setDestination(v);
			message.setType("mydist");
			for(int i=1; i<=n; i++){
				ndis[v][i] = n;
				message.setPayload(i + "," + du[i]);
				unicast(message);
			}
		} else {
			System.err.println("Unknown Message Type");
		}
	}

	private synchronized void recompute(int v){
		int oldDU = du[v];
		if(v == pid){
			du[v] = 0;
			nbu[v] = 0; // local
		} else {
			// Find shortest route (if one exists)
			int minD = n; int indexStore = -1;
			for(int i=0; i<neighbours.size(); i++){
				int index = neighbours.get(i);
				if(minD > ndis[index][v]){
					minD = ndis[index][v];
					indexStore = index;
				}
			}

			int d = 1 + minD;

			// If we have a viable route, update routing tables
			if (d < n){
				du[v] = d;
				nbu[v] = indexStore;
				ndis[pid][v] = d;
			} 
			else {
				du[v] = n;
				nbu[v] = -1; // Undefined
				ndis[pid][v] = n;
			}
	
		}
		// If our du[] has changed, notify neighbours
		if(oldDU != du[v]){
			broadcast("mydist", v + "," + du[v]);
		}
	}

	// Print requested routing table   Dest:FwdTo
	private void printRoutingTable(){
		try{
			PrintWriter output = new PrintWriter("P" + pid + "-rt-1.out");
			Utils.out(pid, "Printing Routing Table");
			for(int i=1; i<=n; i++){
				int nextHop = nbu[i];
				String hopTo;
				if(nextHop == -1){
					hopTo = "undefined";
				} 
				else if(nextHop == 0){
					hopTo = "local";
				} 
				else {
					hopTo = String.valueOf(nextHop);
				}
				output.println(i + ":" + hopTo);
			}
			output.close();
		} catch (Exception ignored){}

	}

	// An alternative printing function. Prints all routing information. Useful for debugging
	private void printRoutingTable2(){ 
		try{
			PrintWriter output = new PrintWriter("p" + pid + "rt.txt");
			Utils.out(pid, "Printing Routing Table");

			for(int i=1; i<=n; i++){
				for(int j=1; j<=n; j++){
					output.print(ndis[i][j] + ",");
				}
				output.print("\n");
			}

			output.print("\n\nDest:  Dist  :  FwdTo\n\n");
			for(int i=1; i<=n; i++){
				output.println(i + "\t:\t" + du[i] + "\t:\t" + nbu[i]);
			}
			output.close();

		} catch (Exception ignored){}
	}

	public static void main(String [] args){
		NetchangeProcess p = new NetchangeProcess (args[0], Integer.parseInt(args[1]),Integer.parseInt(args[2]));
		p.registeR();
		p.begin();	
	}
		

}
