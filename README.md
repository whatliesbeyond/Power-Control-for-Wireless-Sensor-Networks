# Power-Control-for-Wireless-Sensor-Networks

A centralized adaptive power control and routing algorithm for rechargeable Wireless Sensor Network with limited power source


Abstract—This scheme proposes a Centralized Power Control and Routing (CPCR) protocol for large scale rechargeable Wireless Sensor Networks (WSNs). The scheme is based on transmission power control and energy aware routing for the nodes with minimum remaining lifetime. Our focus is to reduce the overhearing of those nodes to eventually increase their lifetime, hence contributing towards overall increment of network lifetime. The WSN considered in this paper is deployed in an area of 1000m X 400m and is characterized with multi hop, asynchronous communication. The outcome of the simulations adds to the cogency of this research.
Keywords— Wireless Sensor Network; Centralized Power Control and Routing; Transmission range adaptation; Cost adaptation; Network Lifetime; Overhearing

I. INTRODUCTION
A Wireless Sensor Network (WSN) comprises of a large number of wireless sensor nodes, each of which is equipped with one or more sensors (eg. temperature, pressure, light etc.) along with an embedded processor, memory and a wireless interface. Wireless Sensor Networks can be used in a wide variety of applications like environmental monitoring, structural monitoring, industrial automation etc. Power management is a critical issue for such kind of networks as the wireless sensor nodes have limited power resources that rely on small batteries and it is not practical to replace them. As an alternative, rechargeable batteries may be used that may be charged by renewable energy sources like solar energy. The energy resources in rechargeable sensor nodes depend on the availability of harvested energy and its storage limitations which can be highly variable. The lifetime of a rechargeable WSN may be ideally infinite, but it faces the problem of wide spatial and temporal variations due to natural and location specific factors that are difficult to predict before deployment. Consequently, the lifetime of a rechargeable WSN may be maximized if each node adapts its operations so that its energy consumption is controlled corresponding to its energy resources.
In this work, we consider transmission power control along with route adaptations to achieve adaptations of the power consumption of the sensor nodes that are characterized by variable energy resources to maximize the network lifetime. Both Power control and routing affect the power consumption of the nodes; however these must be performed cooperatively in the entire network since their effects are global and not node specific. Our objective is to focus on the mechanisms for adaptations rather than how the cooperation is implemented which is beyond the scope if this work. Hence, we consider that the power control and route adaptations are performed in a centralized way under the assumption that all network information is available.
We consider WSNs that comprise of a large number of rechargeable sensor nodes that are typical of deployments in industries/ forests/ structures for monitoring applications. The nodes are located close to each other and they often overhear data packet or route update transmissions from neighboring nodes which are not meant for them. These overheard data/route update packets are major contributors for power consumption. Therefore reducing the overhearing of nodes decreases their power consumption and results in an increase of network lifetime. We develop a centralized power control and routing protocol (CPCR) to achieve energy conservation by reducing overhearing of the nodes with minimum lifetime remaining. CPCR is a combination of two schemes:
1. Transmission Range Adaptation (TRA):
The transmission power of the neighbors of the nodes with low lifetime is decreased to a minimum level that is required to maintain the connectivity of the neighbors with their respective parents. This reduces the overhearing of these nodes thereby reducing its power consumption and increasing its lifetime.
2. Cost Adaptation (CA):
The cost between neighbors of nodes with low lifetime and their respective parents(also vice versa) are increased to divert the forwarded traffic from their vicinity that further contributes to reduce the overhearing of these nodes.
A detailed explanation of CPCR and its schemes is provided in Section IV. Section II gives a brief description of related work on Power Control techniques of WSNs.
Section III describes the system model and problem statement for the adaptation schemes. Section V is
dedicated for performance evaluation of CPCR and its sub-schemes based on simulation results. Section VI concludes the report.

II. RELATED WORK
There is a huge amount of research going on for Power control and energy aware routing of WSNs and researchers keep coming up with new ideas, supporting simulations and best possible results. One such paper [2] proposes a method in which the minimum transmission power of one node with the neighboring nodes is considered as “distance”. This distance is input to a shortest path algorithm, the outcome of which results in minimum transmission power levels for all nodes and optimized routing information for the whole network. In paper [3] they targets at Home/building automation application and propose a routing strategy which utilizes an adaptive energy-slope control method to keep every node alive for a fixed lifetime period (maintenance period). This scheme claims to optimize the use of various energy types within a network based on maintenance schedule for each sub-network.
Another approach as proposed in [4] is a practical Transmission Power Control (P-TPC) protocol which combines a practical system design and control theoretic approach. It is based on a feedback control algorithm, stability analysis and an online link model identification scheme. This protocol is receiver oriented i.e. receiver selects the transmission power for all incoming links according to Packet Received Ratio (PRR) which is considered as a direct metric of the quality of the link.

III. SYSTEM MODEL AND PROBLEM STATEMENT
Figure 1: WSN considered for this project
We assume a WSN located in an area of 1000*400m2 [1] as shown in figure 1. The nodes are deployed randomly and they perform periodic sampling of a sensor signal that needs to be transmitted to the base station. Communication is maintained over multi-hop asynchronous transmissions. Two nodes are considered to be connected to each other if their transmission range is more than or equal to the distance between them. We assume that the network uses a routing framework similar to the Collection Tree Protocol (CTP) [5], where nodes transmit periodic route update (beacon) packets for exchanging link information. The network is assumed to apply periodic Low-Power-Listen (LPL) for energy conservation where each node remains in sleep mode most of the time, waking up for brief periods (8 times per second) to check for transmissions [1]. If any activity is detected, the node stays awake to receive the packet, else it continues its periodic duty-cycling. All packets are transmitted with an extended preamble so that they are not missed.
The parameters required for simulation are shown in the table below:

TABLE 1:
MEASUREMENTS OF CURRENT DRAWN BY MICAZ MOTE UNDER VARIOUS EVENTS
Event                                       Current(mA)                     Duration(ms)
RU Transmit/Receive (Rt/Rr)                   20                                140  
Data Transmit/Receive (Dt/Dr)                 20                                140
Processing(P)                                 8                                  3
Sensing(S):                                   9.5                               7000
-Vibration/sound                                

The current consumption of each node is calculated based on a formula developed in [1]:
I= IRtTRt/TRUI + Lo(IDt TDt / TD) + N(IRrTRr / TRUI + O(IDrTDr / TD)) + ISTS / TD + 8 IPTP
Here, Ix and Tx represent the current drawn and the duration, respectively, of the event x; and x can be transmission of route update or data packets (Rt/Dt), reception of route update or data packets (Rr/Dr) and Sensing(S) and Processing(P). TRUI and TD denote Route Update Interval and data interval respectively. While Lo, O and N denote the Load, Overheard data packets and number of neighbors of each node respectively. We assume initially each node has only one data packet. The nodes were programmed with route update interval of 900s and data sampling interval of 900s.
We assume that the health/remaining lifetime (l) of each node is directly proportional to the remaining battery capacity (C) and inversely proportional to the current (I) drawn at the node. The lifetime formula used is as follows-
l = C/I
where we assume the value of C at any point of time varies randomly from 50-100% for each node.

IV. PROPOSED CENTRALIZED POWER CONTROL AND ROUTING SCHEME
CPCR is a combination of TRA and CA schemes. For simplicity we will discuss these schemes separately and then decide in what order they must be implemented to achieve best results.
To begin with, the simulation of our scheme starts with an input of X number of nodes, with a transmission range of Y (common to every node initially) and the number of runs for
which we want to test the energy efficiency of our network. As these inputs are fed to the simulator it first selects random locations for each wireless sensor node that are chosen uniformly over the deployment region. Then it finds the distance between each pair of nodes. A node k is considered to be connected to node i if the distance between i and k is less than or equal to range of k. The cost of link between i and k is considered to be 1if they are connected and set to infinity otherwise. After which it runs shortest path algorithm to get the initial least cost path for data packets transmissions from each node to the sink. With this network setup and initial routes, we calculate the power consumption and lifetime of each node with the help of the formulae explained in Section III. We then apply our adaptation algorithms TRA and CA individually to evaluate their effects. Each algorithm is applied sequentially to nodes starting with the one that has the lowest remaining lifetime. We consider both cases, the first where TRA is applied before CA and vice versa.

A. Transmission Range Adaptation:
TRA aims at reducing the overhearing of the node with minimum remaining lifetime (NLmin) by controlling the transmission power of its neighbors such that it is reduced to a minimum level while maintaining the communication with their respective parents. The scheme adapts to the new/controlled transmission power of the nodes and performs shortest path algorithm to give least cost path of all nodes to the sink. Results (Section V) show reduced overhearing and increased lifetime of the node. The process is repeated with the nodes that has the lowest remaining lifetime in steps. The following piece of code provides a basic procedure of this adaptation-
If(i== NLmin)
{
If(NA(NLmin!=0))
{
If(D(k,parent[k])<=range[k])
range[k]=D(k,parent[k]);
}
}
This algorithm selects the neighbors of the node with minimum lifetime with the help of a neighbor array [NA(NLmin) ] checks the distance between the neighbor and its parent (D(k,parent[k]) if it comes out to be less than or equal to the range of neighbor (range[k]), the range[k] is equalized to (D(k,parent[k]). After implementing TRA we rerun the simulation starting step 3 until number of loops are half of its value, looking at the results from simulations we observe an increase in the network lifetime(See Section V).
B. Cost Adaptation:
This scheme performs route load reduction at the node with minimum remaining lifetime(NLmin) by increasing its link cost between its neighbors and their respective parents, after which it executes shortest path algorithm. Implementing CA decreases route traffic at the node and increases its lifetime. Following piece of code explains the procedure-
If(i== NLmin)
{
If(NA(NLmin !=0))
{
Cost(k,parent[k])= j;
Cost(parent[k],k)= Cost(k,parent[k]);
}
}
This algorithm selects the neighbors of the node with minimum lifetime with the help of a neighbor array [NA(NLmin) ] and assigns a cost equal to j which is greater than 1 (say 2/3/4) to the link between the neighbor and its parent also vice versa. To get the best results we suggest using the value of j equal to 2 (See Section V).

C. Integrating Both Schemes:
Now, the question lies why to integrate both the schemes when implementing either of them gives us an increase in network lifetime. Our goal is to increase the network lifetime as much as possible. Using both schemes one after the other will serve our purpose but only if we use it in an efficient way. How to implement it ? CA if compared to TRA gives less increase in network lifetime, therefore we suggest using CA scheme after TRA to get the best possible results. Plots (from the results of simulation) in Section V should suffice to comprehend our theory. The corresponding flow chart of CPCR is depicted in Figure 2. Since the communication between nodes in a WSN is a continuous process, which might be hindered with the change in topology of the WSN according to the remaining lifetime of nodes, predicting the energy efficiency for less number of runs would be inappropriate. CPCR selects the nodes with minimum remaining lifetime to increase it, consequently increasing the overall lifetime of WSN and thus stabilizing after few runs. We suggest running CPCR for at least 40 times to get the best results.



VI. CONCLUSION
In this paper we propose a centralized scheme for adapting transmission power control and routing of rechargeable WSN to achieve maximum network lifetime. The effectiveness of this scheme is demonstrated through simulations.

REFERENCES
[1] A. Nasipuri, R. Cox, J. Conrad, L. V. der Zel, B. Rodriguez, and R. McKosky, "Design considerations for a large-scale wireless sensor network for substation monitoring,"Proc. IEEE LCN, pages 866–873, 2010.
[2] Liqiang Zhengy, Wensi Wang, Alan Mathewson, Brendan O'Flynn, Michael Hayes, “An Adaptive Transmission Power Control Method for Wireless Sensor Networks” ISSC, UCC, Cork, June 23-24, pages 261 – 265, 2010.
[3] Hanh-Phuc Le, Mervin John, Kris Pister, “Energy-Aware Routing in Wireless Sensor Networks with Adaptive Energy-Slope Control”, University of California, Berkeley, EE290Q-2 Spring 2009.
[4] Y. Fu, M. Sha, G. Hackmann, and C. Lu, “Practical control of transmission power for wireless sensor networks,” in ICNP, 2012.
[5] O. Gnawali, R. Fonseca, K. Jamieson, D. Moss, and P. Levis, “Collection tree protocol,” in SenSys, 2009, pp. 1–14.
