
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.lang.ProcessBuilder.Redirect;
import java.util.ArrayList;
import java.util.Stack;

public class Graph {
	int N;
	int K;
	int[] cmapping;
	int[] rmapping;
	double[][] greenMatrix;
	Node[] redNodeArray;
	ArrayList<Node> serverList;
	Node[] nodeArray;
	int dualupdates;
	int lru;
	Graph(int n, int k) {
		N=n;
		K=k;
		lru=k+1;
		cmapping = new int[N+K+1];
		rmapping = new int[N+K+1];
		greenMatrix = new double[N+K+1][N+K+1]; //First column is dedicated to the prev vertex in the path (red edge)
		int j=0;
		for (double[] row: greenMatrix) {
			for(int i=0;i<N+K+1;i++)
			{
				row[i] = Double.MAX_VALUE;
			}
			cmapping[j]=j;
			rmapping[j]=j++;
		}
		//Arrays.fill(redCostArray,-1);
		serverList = new ArrayList<Node>();
		nodeArray = new Node[N+K+1];
		redNodeArray = new Node[N+K+1];
	}
	//First all the servers location will be revealed one by one
	void addServer(Node v) {
		//System.out.println("Adding server: "+v.index);
		serverList.add(v);
		nodeArray[v.index] = v;
	}
	//First time adding all anchors after servers are placed.
	void addAnchors() {
		//System.out.println("Adding Anchers");
		for(int j=0;j<K;j++) {
			greenMatrix[rmapping[N+1+j]][0]=j+1;
			//redCostArray[j+1]=0.0;
			nodeArray[N+1+j] = new Node(nodeArray[j+1].x2,nodeArray[j+1].y2, nodeArray[j+1].x2,nodeArray[j+1].y2);
			nodeArray[N+1+j].index = N+1+j;

		}
	}
	double[] addRequest(Node u, int algo_flag, double greedyWt, int hs) throws Exception {
		double ret,ret2;
		// algo_flag = 0 for regular WF, 1 for retro, 2 for greedy, and 4 for special WF with recent requests
		
		/*
		//-------------------------------------------------
		//For writing outut to file
		FileOutputStream f = null;
		try {
			//output file generated based on the algo
			if(algo_flag==0) f = new FileOutputStream("output_WF", true);
			else if (algo_flag==1) f = new FileOutputStream("output_retro", true);
			else if (algo_flag==2) f = new FileOutputStream("output_greedy", true);
			else if (algo_flag==3) f = new FileOutputStream("output_WF_efficientnew", true);
			else if (algo_flag==4) f = new FileOutputStream("output_optimal", true);
			else System.out.println("algo flag not valid");
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		System.setOut(new PrintStream(f));
		//--------------------------------------------------
		*/
		Stack<Integer> pathStack = new Stack<Integer>(); 
		//System.out.println("Adding request: "+u.index);
		nodeArray[u.index] = u;
		if(algo_flag==2) { //GREEDY
			double min = Double.MAX_VALUE;
			int minNodeIndex = -1;
			for(int i=N+1;i<N+K+1;i++) {
				//1:RED, 2: BLUE
				double temp = euclidian(nodeArray[i],u, hs); //The greedy value
				if(temp<min) {
					min = temp;
					minNodeIndex = i;
				}
			}
			
			//MainClass.onlineCost += euclidian(nodeArray[minNodeIndex],u,hs);
			ret = euclidian(nodeArray[minNodeIndex],u,hs);
			ret2 = minNodeIndex - N;
			nodeArray[minNodeIndex].x1 = u.x1;
			nodeArray[minNodeIndex].y1 = u.y1;
			nodeArray[minNodeIndex].x2 = u.x2;
			nodeArray[minNodeIndex].y2 = u.y2;
			
		}
		else {
					//Add new green edges
					//--From u to all previous requests and anchor nodes
					
					addNewGreenEdges(u,lru,hs);
					//Find the shortest augmenting path from u to anchor nodes
					ShortestPath t = new ShortestPath(this);
					
					
					double[][] distParent;
					distParent = t.dijkstra(u.index, lru, algo_flag, greedyWt, hs); //Now we have dist as well as parent array
					
					double min = Double.MAX_VALUE;
					int minNodeIndex = -1;
					for(int i=N+1;i<N+K+1;i++) {
						double temp;
						if(algo_flag==0 || algo_flag==3) {
							temp = (1-greedyWt)*(distParent[0][i] +(u.dual- nodeArray[i].dual)) + greedyWt*euclidian(nodeArray[i],u,hs); //The work function value
						}
						else temp = distParent[0][i] +(u.dual- nodeArray[i].dual); //The retrospective or optimum value
						if(temp<min) {
							min = temp;
							minNodeIndex = i;
						}
					}
					
					if(algo_flag==4) {
						//MainClass.onlineCost += min;
						ret = min;
					}
					else {
						//MainClass.onlineCost += euclidian(nodeArray[minNodeIndex],u,hs);
						ret = euclidian(nodeArray[minNodeIndex],u,hs);
					}
					ret2 = minNodeIndex-N;
					//Update dual weights
					
					dualupdates=0;
					
					for(int j=lru;j<=N+K;j++) {
						if(distParent[0][j] != Double.MAX_VALUE && distParent[0][minNodeIndex] > distParent[0][j])	{
							nodeArray[j].dual = nodeArray[j].dual + (distParent[0][minNodeIndex] - distParent[0][j]);
							dualupdates++;
						}
					}
					
					///We need to have path edges in forward sequence to be able to augment
					//Fill path nodes in stack starting from the last node in the path and moving backwards
					int parentIndex = minNodeIndex;
					while(parentIndex != u.index) {
						pathStack.push(parentIndex);
						parentIndex = (int)distParent[1][parentIndex];
					} 
					//Process path edges one by one
					int a_index = u.index, b_index = pathStack.pop();
					while(!pathStack.empty()) {
						// Do the operation for regular green edge a to b
						//Remove green edge a to b
						double aTobGreen = greenMatrix[rmapping[a_index]][cmapping[b_index]];
						greenMatrix[rmapping[a_index]][cmapping[b_index]] = Double.MAX_VALUE;
						//-- All incoming green edge of b become's a's incoming green but with different cost
						int temp = cmapping[a_index];
						cmapping[a_index] = cmapping[b_index];
						cmapping[b_index] = temp;
						int c_index = (int) greenMatrix[rmapping[b_index]][0]; 
						Node c;
						if(c_index==-1) c = redNodeArray[b_index]; 
						else c = nodeArray[(int)greenMatrix[rmapping[b_index]][0]];
						
						//-- (a,b) has the intermediate vertex, say c, such that (c,b) is red
						//-- Remove current red edge (c,b) and add a new red edge (c,a)
						greenMatrix[rmapping[b_index]][0] = Double.MAX_VALUE;
						if(c_index==-1) {
							greenMatrix[rmapping[a_index]][0]=-1;
							redNodeArray[a_index] = c;
						}
						else greenMatrix[rmapping[a_index]][0]=c_index;
						//cost of earlier red edge is c to b, cost of new red edge is c to a
						//double red_old_cost = euclidian(nodeArray[c_index], nodeArray[b_index]); 
						//double red_new_cost = euclidian(nodeArray[c_index], nodeArray[a_index]); 
						//change the cost of all the incoming green to a
						for(int j=1;j<=N;j++) {
							if(greenMatrix[rmapping[j]][cmapping[a_index]] != Double.MAX_VALUE) {
									//greenMatrix[j][mapping[a_index]] = cost(j to c) - cost(c to a)'
									//greenMatrix[rmapping[j]][cmapping[a_index]] = euclidian(c,nodeArray[j],hs) - euclidian(c, nodeArray[a_index],hs);
									greenMatrix[rmapping[j]][cmapping[a_index]] = greenMatrix[rmapping[j]][cmapping[a_index]] - aTobGreen;
									
								}
						}
						//-- Flip the direction with a new cost of green edge (a,b)
						greenMatrix[rmapping[b_index]][cmapping[a_index]] = -1*aTobGreen;
						
						a_index = b_index;
						b_index = pathStack.pop();
					}
					//System.out.println();
					//Now (a,b) is the last edge of the path where b is an anchor node
					double aTobGreen = greenMatrix[rmapping[a_index]][cmapping[b_index]];
					//Remove green edge a to b
					greenMatrix[rmapping[a_index]][cmapping[b_index]] = Double.MAX_VALUE;
					//-- All incoming green edge of b become's a's incoming green
					int temp = cmapping[a_index];
					cmapping[a_index] = cmapping[b_index];
					cmapping[b_index] = temp;
					//-- (a,b) has the intermediate vertex, say c, such that (c,b) is red
					int c_index =(int) greenMatrix[rmapping[b_index]][0];
					Node c;
					if(c_index==-1) c = redNodeArray[b_index]; 
					else c = nodeArray[(int)greenMatrix[rmapping[b_index]][0]];
					//-- Remove edge (c,b) and add a new red edge (c,a) and add a new red edge u to b
					greenMatrix[rmapping[b_index]][0] = u.index;
					if(c_index == -1) {
						greenMatrix[rmapping[a_index]][0] = -1;
						redNodeArray[a_index] = c;
					}
					else greenMatrix[rmapping[a_index]][0] = c_index;
					//Change b location to u's as b is the u's anchor node now.
					nodeArray[b_index].x1 = u.x2;
					nodeArray[b_index].y1 = u.y2;
					nodeArray[b_index].x2 = u.x2;
					nodeArray[b_index].y2 = u.y2;
					nodeArray[b_index].dual=0.0;
					ret2 = b_index-N;
					//change the cost of all the incoming green to a
					for(int j=1;j<=N;j++) {
						if(greenMatrix[rmapping[j]][cmapping[a_index]] != Double.MAX_VALUE) {
							//greenMatrix[j][mapping[a_index]] = cost(j to c) - cost(c to a)
							//greenMatrix[rmapping[j]][cmapping[a_index]] = euclidian(c,nodeArray[j],hs) - euclidian(c, nodeArray[a_index],hs);
							greenMatrix[rmapping[j]][cmapping[a_index]] = greenMatrix[rmapping[j]][cmapping[a_index]] - aTobGreen;

						}
					}
					
					//update lru if needed
					if(algo_flag==3) {
						int l =Integer.MAX_VALUE;
						for(int i=N+1;i<N+K+1;i++) {
							if (greenMatrix[rmapping[i]][0]>K && greenMatrix[rmapping[i]][0] < l) l = (int) greenMatrix[rmapping[i]][0];
							else if(greenMatrix[rmapping[i]][0]<=K) {
								l = K+1;
								break;
							}
						}
						lru = l;
					}
			
		//Slide if needed
		
		if(u.index==N) {
			int mid = K+(N-K)/2;
			for(int i=K+1;i<=mid;i++) {
				
				int temp1 = cmapping[i];
				int temp2 = rmapping[i];
				cmapping[i] = cmapping[1+mid+(i-(K+1))];
				rmapping[i] = rmapping[1+mid+(i-(K+1))];
				cmapping[1+mid+(i-(K+1))] = temp1;
				rmapping[1+mid+(i-(K+1))] = temp2;
				if(greenMatrix[rmapping[i]][0]>N) {
					//do nothing
				}
				else if(greenMatrix[rmapping[i]][0]>mid) {
					greenMatrix[rmapping[i]][0] = K + greenMatrix[rmapping[i]][0]-mid;
					
				}
				else {
					if(greenMatrix[rmapping[i]][0]  != -1) redNodeArray[i] = nodeArray[(int)greenMatrix[rmapping[i]][0]];		
					else redNodeArray[i] = redNodeArray[1+mid+(i-(K+1))];
					
					greenMatrix[rmapping[i]][0]  = -1;
				}	
			}
			
			//red to anchor nodes
			for(int i=N+1;i<=N+K;i++) {
				if(greenMatrix[rmapping[i]][0]>N) {
					//do nothing
				}
				else if(greenMatrix[rmapping[i]][0]>mid) {
					greenMatrix[rmapping[i]][0] = K + greenMatrix[rmapping[i]][0]-mid;
				}
				else{
					if(greenMatrix[rmapping[i]][0]  != -1) redNodeArray[i] = nodeArray[(int)greenMatrix[rmapping[i]][0]];
					//else redNodeArray[i] = redNodeArray[1+mid+(i-(K+1))];
					
					greenMatrix[rmapping[i]][0]  = -1;
				}
			}
			for(int i=K+1;i<=mid;i++) {
				nodeArray[i] = nodeArray[1+mid+(i-(K+1))];
				nodeArray[i].index = i;
			}
			
			for(int i=mid+1;i<=N;i++) {
				nodeArray[i] = null;
				for(int j=0;j<=N+K;j++) {
					greenMatrix[rmapping[i]][j] = Double.MAX_VALUE;
					greenMatrix[j][cmapping[i]] = Double.MAX_VALUE;
				}
			}
			if(lru>mid) lru = lru - (mid-K);
			else lru=K+1;
			}
		}
		double[] d = {ret,ret2};
		return d;
			
	}

	private void addNewGreenEdges(Node a, int lru,int hs) {
		//For every red edge add the corresponding green edge
		//First add green edge from u to all anchor nodes
		for(int j=1;j<=K;j++) {
			greenMatrix[rmapping[a.index]][cmapping[N+j]] = euclidian(nodeArray[N+j],a,hs);
			
		}
		//Then add green edge from u to all previous request nodes
		//Let the red edge to a previous request be (c,b), and out request is a. We need to add (a,b) where cost(a,b)=eucl(a,c)-eucl(c,b) 
		for(int j=K+1;j<a.index;j++) {
			Node b = nodeArray[j];
			Node c;
			if(greenMatrix[rmapping[j]][0]==-1) c = redNodeArray[j]; 
			else c = nodeArray[(int)greenMatrix[rmapping[j]][0]];
				
			greenMatrix[rmapping[a.index]][cmapping[j]] = euclidian(c,a,hs) - euclidian(c,b,hs);
			
		}
	}

	

	double euclidian(Node u, Node v, int hs) {
		if(hs==0)	return Math.sqrt((v.x1-u.x2)*(v.x1-u.x2) + (v.y1-u.y2)*(v.y1-u.y2));
		else if(hs==1){
		//Calculating Haversine distance
			double lon1 = u.x2;
			double lat1 = u.y2;
			double lon2 = v.x1;
			double lat2 = v.y1;
			int R = 6371; // Radious of the earth
			double latDistance = toRad(lat2-lat1);
			double lonDistance = toRad(lon2-lon1);
			double a = Math.sin(latDistance / 2) * Math.sin(latDistance / 2) + 
			Math.cos(toRad(lat1)) * Math.cos(toRad(lat2)) * 
			Math.sin(lonDistance / 2) * Math.sin(lonDistance / 2);
			double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
			double distance = R * c;
			return distance;
		}
		else if(hs==2){
				double lon1 = u.x2;
				double lat1 = u.y2;
				double lon2 = v.x1;
				double lat2 = v.y1;
				double deglen = 110.25;
				double a = lat2 - lat1;
				double b = (lon2 - lon1)*Math.cos(lat1);
				return deglen*Math.sqrt(a*a + b*b);
			}
		else return -1;
	}
			 
		private static double toRad(Double value) {
			 return value * Math.PI / 180;
		}
}
