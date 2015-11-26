
import java.io.*;
import java.net.*;
import java.util.*;

class EPFDProcess extends PFDProcess {

	public EPFDProcess (String name, int id, int size){
		super(name, id, size);
		detector = new EventuallyPerfectFailureDetector(this);
	}

	public static void main(String [] args){
		EPFDProcess p = new EPFDProcess (args[0], Integer.parseInt(args[1]),Integer.parseInt(args[2]));
		p.registeR();
		p.begin();	
	}

}
