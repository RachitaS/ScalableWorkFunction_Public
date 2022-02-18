import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.Scanner;


public class MainClass {
	static String FILE = "KServer_Project/testing"; // Input file: First line: serverCount requentCount. Next requentCount lines: x y x y. Where x and y and coordinates of request locations.
	static int serverCount;
	static int reqCount;
	static Graph G;
	static double onlineCost;
	
	//Parameter servers and users are in the format: {{x1,y1},{x2,y2},...}
	//eucl(hs=0), haversone(hs=1), approx_haversine(hs=2) 
	public static double[][] kServer_API(int numServer, int numUser, double[][] servers, double[][] users, int algoFlag, double greedyWt, int hs) throws Exception{
		if(greedyWt==1 && (algoFlag==0 || algoFlag==3)) algoFlag=2;
		int userSize = users.length;
		double[] onlineCostList = new double[userSize];
		double[] serverIds = new double[userSize];
		serverCount = numServer;
		reqCount = numUser;
		G = new Graph(serverCount+reqCount, serverCount);
		int count=1;
		for(double[] s : servers) {
			Node v = new Node(s[0], s[1], s[0], s[1]);
			
			v.index = count++;
			G.addServer(v);
		}
		G.addAnchors();
		int i=0;
		for(double[] r : users) {
			Node v = new Node(r[0], r[1], r[2], r[3]);
			v.index = count++;	
			double[] temp = G.addRequest(v,algoFlag, greedyWt, hs);
			onlineCostList[i] = temp[0];
			serverIds[i++] = temp[1];
			if(count==serverCount+reqCount+1) count=1+ serverCount + (reqCount/2);
		}
		double[][] d = {onlineCostList, serverIds};
		return d;
	}

	static void parse(int algoFlag, double greedyWt,int hs) throws Exception{
		try {
			if(greedyWt==1 && (algoFlag==0 || algoFlag==3)) algoFlag=2; 
			Scanner scanner = new Scanner(new File(FILE));
			if(scanner.hasNextLine()) {
				String[] arr = scanner.nextLine().split("\\s+");
				serverCount = Integer.parseInt(arr[0]);
				reqCount = Integer.parseInt(arr[1]);
				G = new Graph(serverCount+reqCount, serverCount);
			}
			int count=0;
			while (scanner.hasNextLine()) {
				String[] arr = scanner.nextLine().split("\\s+");
				double x1 = Double.parseDouble(arr[0]);
				double y1 = Double.parseDouble(arr[1]);
				
				double x2 = Double.parseDouble(arr[2]);
				double y2 = Double.parseDouble(arr[3]);
				//double x1=x2;
				//double y1=y2;
				//System.out.println(x1 +", "+y1);
				count++;
				if(count <= serverCount) {
					Node v = new Node(x1, y1, x1, y1);
					v.index = count;
					G.addServer(v);
				}
				else {
					if(count==serverCount+1) {
						G.addAnchors();
					}
					Node v = new Node(x1, y1, x2, y2);
					//Node v = new Node(x1, y1, x1, y1);
					v.index = count;
					
					double[] temp = G.addRequest(v, algoFlag, greedyWt,hs);
					System.out.println(temp[0]);
					//System.out.println(temp[1]);
					
				}
				if(count==serverCount+reqCount) count=serverCount + (reqCount/2);
			}
			scanner.close();
		} catch (FileNotFoundException e) {
			System.out.println("File not found");
			e.printStackTrace();
		}
	}
	

	public static void main(String[] args) throws Exception {
		onlineCost=0;
		/* Call with parameter 0, 1, 2, 3 or 4
		 * 0: Work function
		 * 1: Retrospective
		 * 2: Greedy
		 * 3: Efficient WF
		 * 4: Optimal
		 */
		//call with algoflag, greedyWt, Havesine
		parse(3,0.5,1);
		System.out.println("Total Cost = "+onlineCost);
	}
}
