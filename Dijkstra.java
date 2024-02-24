/**
 * Anna Rees
 * Algorithm Design and Analysis - Byung Lee
 * February 2024
 *
 * Write a program code to implement the Dijkstra’s single-source shortest paths finding algorithm discussed in class
 * and show the trace of the algorithm against the directed graph
 */

import java.util.*;
import java.util.PriorityQueue;

public class Dijkstra{

    //    initializing Dijkstra"s alg w/ Edge class
    static class Edge {
        String destination;
        int weight;

        public Edge(String destination, int weight){
            this.destination = destination; //destination node
            this.weight = weight;
        }
    }

    static class Node implements Comparable<Node>{
        String ID; //node ID
        int shortestDist; //shortest distance from node
        String previous; //previous node
        List<Edge> edges; //list of edges

        public Node(String ID) {
            this.ID = ID;
            this.shortestDist = Integer.MAX_VALUE; //initialize to infinity (kinda)
            this.previous = null; //no previous node initially
            this.edges = new ArrayList<>();
        }

        @Override
        public int compareTo(Node otherNode) {
            return Integer.compare(this.shortestDist, otherNode.shortestDist);
        }
    }

    static class Graph {
        List<Node> nodes;

        //      graph constructor
        public Graph(int numNodes) {
            //            creating a list of all the nodes
            nodes = new ArrayList<>(numNodes);
        }

        public void addNode(String nodeID) {
            nodes.add(new Node(nodeID));
        }

        public void addEdge(String source, String destinationNode, int weight) {
            //adding edge to list of edges
            Node sourceNode = getNode(source);
            sourceNode.edges.add(new Edge(destinationNode, weight));
        }

        private Node getNode(String ID) {
            for (Node node : nodes) {
                if (node.ID.equals(ID)) {
                    return node;
                }
            }
            return null;
        }

    }

    /**
     * creating PriorityQueueNode objects so the nodes can be compared and placed
     * in the queue correctly
     */

    static void Dijkstra(Graph graph, String source) {
        /**
         * Dijkstra(G,s) {
         *  *     Initialize Q of V with pi(v) <- infinity for each node v in V
         *  *     S <- {}, pi(s)<-0
         *  *     while(Q is not empty){
         *  *         v = ExtractMin(Q)
         *  *         add v to S
         *  *         for each edge e=(v,w) such that w is not in set S {
         *  *             if(pi(v) + l(e) < pi(w)){
         *  *                 pi(w) = pi(w) + l(e)
         *  *                 ChangeKey(Q, w, pi(w))
         *  *             }
         *  *         }
         *  *         endwhile
         *  *     }
         *  * }
         */
        // int numNodes = graph.nodes.size();
        PriorityQueue<Node> priorityQueue = new PriorityQueue<>();
        //Initialize Q of V with pi(v) <- infinity for each node v in V
        for (Node node : graph.nodes) {
            //initializing the shortest distance and previous nodes
            node.shortestDist = Integer.MAX_VALUE;
            node.previous = null;
        }
        //first node in the list (starting node) is distance 0 from self
        //S <- {}, pi(s)<-0
        graph.getNode(source).shortestDist = 0;
        priorityQueue.add(graph.getNode(source));

        //while(Q is not empty){
        while(!priorityQueue.isEmpty()) {
            //v = ExtractMin(Q)
            Node current = priorityQueue.poll();

            //add v to S
            System.out.print("Node " + current.ID + " included in S with the shortest path length " + current.shortestDist + " on the path s");
            String predecessor = current.previous;
            while(predecessor != null) {
                System.out.print(" - " + predecessor);
                predecessor = graph.getNode(predecessor).previous;
            }
            System.out.println(" - " + current.ID + ".");

            //for each edge e=(v,w) such that w is not in set S {
            //  if(pi(v) + l(e) < pi(w)){
            //  pi(w) = pi(w) + l(e)
            //  ChangeKey(Q, w, pi(w))
            //  }
            // }
            for (Edge edge : current.edges) {
                Node next = graph.getNode(edge.destination);
                if (current.shortestDist + edge.weight < next.shortestDist) {
                    next.shortestDist = current.shortestDist + edge.weight;
                    next.previous = current.ID;
                    priorityQueue.remove(next);
                    priorityQueue.add(next);
                }
            }
        }
    }

    public static void main(String[] args) {
        int numNodes = 8;
        Graph graph = new Graph(numNodes);
        //graph.addEdge(u ID, v ID, weight)
        graph.addNode("s");
        graph.addNode("2");
        graph.addNode("3");
        graph.addNode("4");
        graph.addNode("5");
        graph.addNode("6");
        graph.addNode("7");
        graph.addNode("t");

        graph.addEdge("s","2", 9);
        graph.addEdge("s","6", 14);
        graph.addEdge("s","7", 15);

        graph.addEdge("2","3", 23);

        graph.addEdge("6","3", 18);
        graph.addEdge("6","5", 30);
        graph.addEdge("6","7", 5);

        graph.addEdge("7","5", 20);
        graph.addEdge("7","t", 44);

        graph.addEdge("3","t", 19);
        graph.addEdge("3","5", 2);

        graph.addEdge("5","4", 11);
        graph.addEdge("5","t", 16);

        graph.addEdge("4","3", 6);
        graph.addEdge("4","t", 6);

        //Run algorithm
        System.out.println("Graph is represented using an adjacency list.");
        Dijkstra(graph, "s"); // Source node is "s"

    }
}