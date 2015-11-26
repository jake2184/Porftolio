/* Copyright (c) 2013-2015, Imperial College London
 * All rights reserved.
 *
 * Distributed Algorithms, CO347
 */

public class MessageEntry implements Comparable<MessageEntry> {
	
	private String message;
	private int priority;
	private long timestamp;
	
	public MessageEntry (String message, int priority, long timestamp) {
		this.message   =   message;
		this.priority  =  priority;
		this.timestamp = timestamp;
	}
	
	public int getPriority() {
		return priority;
	}
	
	public String getMessage() {
		return message;
	}
	
	public int compareTo (MessageEntry m) {
		int result;
		result = this.priority - m.priority;
		if (result == 0)
			return (int) (this.timestamp - m.timestamp);
		return result;
	}
}
