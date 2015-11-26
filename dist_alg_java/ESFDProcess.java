
import java.io.*;
import java.net.*;
import java.util.*;

class ESFDProcess extends EPFDProcess {

	private boolean decided = false;
	private int messageReceived = 0;

	private Vector<MessageInfo> messageStash;
	private Vector<MessageInfo> messageBuffer;

	private int ackNo = 0, nackNo = 0;
	private int estimate = 0, leader = 0; 

	private Message decisionMessage;
	private Message outcomeMessage;

	public ESFDProcess (String name, int id, int size){
		super(name, id, size);
		detector = new EventuallyStrongFailureDetector(this);
		messageStash = new Vector<MessageInfo>();
		messageBuffer = new Vector<MessageInfo>();
	}

	public synchronized void begin() {
		detector.begin();
		consensus();
	}

	public synchronized void receive (Message m) {
		String type = m.getType();
		if (type.equals("heartbeat")){
			detector.receive(m);
		} else if (type.equals("VAL")){
			//MessageInfo messageInfo = new MessageInfo(m);			
			//estimate = messageInfo.data.get(0);
			messageReceived = m.getSource();
			messageBuffer.add(new MessageInfo(m));
			notifyAll();

		} else if (type.equals("outcome")){
			//MessageInfo messageInfo = new MessageInfo(m);
			//estimate = messageInfo.data.get(0);
			outcomeMessage = m;
			messageReceived = m.getSource();
			notifyAll();

		} else if (type.equals("decision")){
			if(decided == false){	

				/* Here, or in consensus loop? */
				MessageInfo decMessInfo = new MessageInfo(m);
				leader = decMessInfo.data.get(0);
				decided = true;
				Utils.out(pid, String.format("Decided on %s", leader));
				/*			****		*/

				decisionMessage = m;
				messageReceived = m.getSource();
				notifyAll();				
			}

		} else if (type.equals("ack")){
			ackNo++;
			notifyAll();
		} else if (type.equals("nack")){
			nackNo++;
			notifyAll();
		}
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
				//System.err.println("ERROR"); 
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

	private synchronized void consensus (){
		// Number of processes responses required
		int majorityN = (int) Math.ceil( (float) n / 2);

		int estimate = pid;
		// r is round number, k is round number where val was sent
		// c is coordinator pid
		int r = 0, k = 0, c;	
		decided = false;	
		while(!decided){
			messageBuffer.clear();
			r++;
			c = (r % n) + 1;

			// Send message to coordinator with current estimate
			Message m = new Message();
			m.setSource(pid);
			m.setDestination(c);
			m.setType("VAL");
			m.setPayload(estimate + "," + r + "," + k);
			unicast(m);

			if(c == pid){
				// Wait till coordinator receives majority of estimates
				while(messageBuffer.size() < majorityN){
					try {
						wait();
					} catch (Exception e) {
						System.out.println("ERROR");
					}
				}
				
				// Stash current buffer, ignore any extra messages
				messageStash = messageBuffer;

				// t = max value of k. 
				int t = 0, index = 0; 
				
				for(int i=0; i<messageStash.size(); i++){
					// Extract k from each message in bugger
					int Kj = messageStash.get(i).data.get(2); 
					if(Kj > t){
						t = Kj;
						index = i;
					}
				}

				// set estimate 
				estimate = messageStash.get(index).data.get(0);

				// broadcast coordinator's estimate
				broadcast("outcome", estimate+ "," + r);

			} else {
				// Collect the outcome from the coordinator
				if(collect("outcome", c)){					
					// Extract estimate from outcome message of coordinator				
					MessageInfo messInfo = new MessageInfo(outcomeMessage);
					estimate = messInfo.data.get(0); 

					// Set k (last round to receive an outcome) to current round
					// Send ack to coordinator 
					k = r;
					m.setType("ack");
					m.setPayload(String.valueOf(r));
					unicast(m);

				} else {
					// Assume coordinator failed, send nack
					m.setType("nack");
					m.setPayload(String.valueOf(r));
					unicast(m);
				}
			}
			if(c == pid){
				// Wait for majority of processes to respond
				while(ackNo + nackNo < majorityN){
					try {
						wait();
					} catch (InterruptedException e) {
						//System.err.println("ERROR");
					}
				}
				// If we have majority ack, decide and broadcast this value
				if(ackNo >= majorityN){
					broadcast("decision", estimate + "," + r);
					leader = estimate;
					decided = true;
					Utils.out(pid, String.format("Decided on %s", leader));					
				}
				ackNo = 0; 
				nackNo = 0;
			} /*else {		// Have here, or in decision receive?
				// Collect the decision from coordinator
				if(collect("decision", c)){
					// Extract value from decision message
					MessageInfo decMessInfo = new MessageInfo(decisionMessage);
					leader = decMessInfo.data.get(0);
					decided = true;
					Utils.out(pid, String.format("Decided on %s", leader));
				} // else not enough acks/failure of coordinator, rotate
			}*/
		}

	}


	public synchronized void notifySuspect () {
		// If FD suspects a process, notify this as collect could be waiting on it
		notifyAll();
	}

	public static void main(String [] args){
		ESFDProcess p = new ESFDProcess (args[0], Integer.parseInt(args[1]),Integer.parseInt(args[2]));
		p.registeR();
		p.begin();	
	}

}
