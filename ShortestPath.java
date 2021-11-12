import java.util.Arrays;

//Applying direct distance trick to reduce DJK visits
public class ShortestPath { 
	// A utility function to find the vertex with minimum distance value, 
	// from the set of vertices not yet included in shortest path tree 
	static int c=0;
	Graph G;
	int V;
	double distParent[][];
	double[] directDistance;
	ShortestPath(Graph Gr) {
		G = Gr;
		V = G.N+G.K+1;
	}
	int minDistance(double[] distParent, Boolean sptSet[],int lru) 
	{ 
		// Initialize min value 
		double min = Double.MAX_VALUE;
		int min_index = -1; 
		for (int v = lru; v < V; v++) 
			if (sptSet[v] == false && distParent[v]<min) { 
				min = distParent[v]; 
				min_index = v; 
			} 
		
		return min_index; 
	} 

	// Function that implements Dijkstra's single source shortest path 
	// algorithm for a graph represented using adjacency matrix 
	// representation 
	double[][] dijkstra(int src, int lru, int algo_flag, double greedyWt, int hs) throws Exception 
	{ 
		double maxDouble = Double.MAX_VALUE;
		distParent = new double[2][V]; // The output array. dist[i] will hold  // The output array. dist[i] will hold 
		// the shortest distance from src to i 

		// sptSet[i] will true if vertex i is included in shortest 
		// path tree or shortest distance from src to i is finalized 
		Boolean sptSet[] = new Boolean[V]; 

		// Initialize all distances as INFINITE and stpSet[] as false 
		for (int i = lru; i < V; i++) { 
			distParent[0][i] = maxDouble; 
			sptSet[i] = false; 
			distParent[1][i]=-1;
		} 

		// Distance of source vertex from itself is always 0 
		distParent[0][src] = 0.0; 
		
		int countAnchor=0;
		
		int[] anchorFlag = new int[G.K+1];
		Arrays.fill(anchorFlag,0);
		 
		 
		for (int count = lru; count < V ; count++) { 
			// Pick the minimum distance vertex from the set of vertices 
			// not yet processed. u is always equal to src in first 
			// iteration. 
			int u = minDistance(distParent[0], sptSet, lru);
			//if(u==1917) System.out.println(" u is 1917 is "+distParent[0][u]+" but dist of 1584 is "+distParent[0][1584]+" req is "+src);
			//if(u==1798) System.out.println("here is 1798!");
			//System.out.println("okay u = "+u+ "with dist "+distParent[0][u]);
			if(u==-1) break;
			
			// Mark the picked vertex as processed 
			sptSet[u] = true;
			if(algo_flag==0 || algo_flag==3) {
				if(u>G.N && anchorFlag[u-G.N]==0) {
					anchorFlag[u-G.N]=1;
					countAnchor++;
				}
				if( countAnchor>=G.K) break;
			
				directDistance = directDist(src,hs);
				double bestWFYet = bestWFYet(src, sptSet, greedyWt,hs);
			
				for(int i=G.N+1;i<=G.N+G.K;i++) {
					if (sptSet[i]==false && u!=src && ((1-greedyWt)*(distParent[0][u] - G.nodeArray[i].dual) + greedyWt*directDistance[i-G.N])> bestWFYet )	{
						if(anchorFlag[i-G.N]==0) {
							anchorFlag[i-G.N]=1;
							countAnchor++;
						}
					}
				}
				if(countAnchor>=G.K) break;
			}
		
			// Update dist value of the adjacent vertices of the 
			// picked vertex. 
			for (int v = lru; v < V; v++) { 
				// Update dist[v] only if is not in sptSet, there is an 
				// edge from u to v, and total weight of path from src to 
				// v through u is smaller than current value of dist[v]
				//if(v==1584 && u==1798) System.out.println("HERE_0 req is "+src+". dist 1798 is "+distParent[0][u]+" and dist to 1584 is "+distParent[0][v]);
				if (!sptSet[v] && G.greenMatrix[G.rmapping[u]][G.cmapping[v]]!=maxDouble && distParent[0][u]!=maxDouble) {
					double slack = G.greenMatrix[G.rmapping[u]][G.cmapping[v]] - (G.nodeArray[u].dual-G.nodeArray[v].dual);
					//if(v==1584 && u==1798) System.out.println("HERE req is "+src+". dist 1798 is "+distParent[0][u]+" and dist to 1584 is "+distParent[0][v]);
					if (slack <0 && slack + 0.000001 >=0) slack = 0;
						if (distParent[0][u]+slack < distParent[0][v]) { //SLACK
						if(slack<0) {
							System.out.println("STOP!!!  "+slack); //SLACK
							System.out.println("from "+u+" to "+v+" via "+G.greenMatrix[G.rmapping[v]][0]+" is "+G.greenMatrix[G.rmapping[u]][G.cmapping[v]] +"-("+ G.nodeArray[u].dual+"-"+G.nodeArray[v].dual+")");
							System.out.println("req is "+src);
							throw new Exception("Exception: Negative Slack!");
						}
						//if(v==1584 || v==1798) System.out.println("updating dist of "+v+" by "+(distParent[0][u] + slack)+" from "+u+ " when earlier it was "+distParent[0][v]);
						
						distParent[0][v] = distParent[0][u] + slack; //SLACK
						distParent[1][v]=u;
					}
				}
			}	
		} 
		return distParent;
	}
	
	
	private double bestWFYet(int src, Boolean[] sptSet, double greedyWt,int hs) {
		double min = Double.MAX_VALUE;
		for(int j=G.N+1;j<=G.N+G.K;j++) {
			if(sptSet[j]==true && greedyWt*directDistance[j-G.N] + (1-greedyWt)*(distParent[0][j] - G.nodeArray[j].dual) < min) min = greedyWt*directDistance[j-G.N] + (1-greedyWt)*(distParent[0][j] - G.nodeArray[j].dual) ;
		}
		return min;
	}
	private double[] directDist(int src,int hs) {
		double dirDist[] = new double[G.K+1];
		for(int j=G.N+1;j<=G.N+G.K;j++) {
			dirDist[j-G.N] = G.euclidian(G.nodeArray[j],G.nodeArray[src],hs);
		}
		return dirDist;
	}

} 
