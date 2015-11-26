
import java.util.*;

class MessageInfo {
	
	public int source;
	public int destination;
	public String type;
	public String payload;
	Vector <Integer> data;

	MessageInfo(Message m){
		data = new Vector<Integer>();
		this.source = m.getSource();
		this.destination = m.getDestination();
		this.type = m.getType();
		this.payload = m.getPayload();
		String temp[] = payload.split(",");
		for(int i=0; i<temp.length; i++){
			data.add(Integer.parseInt(temp[i]));
		}
	}

}