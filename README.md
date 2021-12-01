# Swarm-Control
Decentralised Swarm Control with MQTT

The purpose of this is to allow a swarm of drones to communicate and send data through mqtt to each other. This code uses python however mqtt is available in different languages and the algorithms presented in this repo serve as examples that may be extended with other software for various use. Additionally dronekit libraries were used to perform some functionality for simplified integration.

To make use of this repo, setup instances of Ardupilot vehicles with SITL (possibly natively in an OS). These vehicles will have TCP/IP ports corresponding between ports 5770-5800. (for a 4 drone mission . Ports 5760 are skipped in the code developed because of issues in my native development environment.).These vehicles are also meant to be started with the locations being UCT to simulate the mission running over UCT campus. To do this when setting up a SITL instance add a -L tag with UCT1-6 as the different locations. (it goes up to six in case of a 6 drone mission, which is currently unsupported by this repo but duplicating the code files and some connection changes can easily allow for this.).An example of this would be to use the command "sim_vehicle.py -L UCT2 --instance 2 --no-mavproxy".This would start the second vehicle at position 2 of the UCT locations. For the drone code the v2 files respectively describe the first leader-followeer confguration developed. The v3 code files desribe the performance for the second configuration of the leader-follower method developed.The naming of these files arised from labelling the number of the type ( e.g follower)  of drone in a swarm followed by the total number of drones in the swarm. Therefore for a 4 drone mission , the second follower is follower 2-4.The IP docker.txt file makes use of some common docker commands and other useful commands.The locations.txt file can be used to replace the default file in Ardupilot's folders to allow for the vehichles to start in familiar locations, which in my case consisted of flying over UCT's Engineering Faculty.

To run a 4 drone mission with any leader-follower configuration (i.e v2 files must be run together to simulate performance of configuration 1 and v3 files correspond to the configuration 2) the last follower must be started first and the order continues until the leader is started , as the MQTT publish-subscribe method requires clients that subscribe to a topic to be connected before data  is published to that topic for it to work correctly.
Therefore e.g for config 1, run follower3-4_v2,run follower2-4_v2,run follower1-4_v2,run leader1-4_v2.
Config 2 would be similar to above excpet the file versions will be v3.

The rest of the code in this repo (not including the v2 and v3 files) was used to improve and develop the latest v2 and v3 files as these were developed later.

Currently running this code will allow a swarm of drones to map a square boundary around the UCT Menzies building.Drone failure will allow for the continued mappinng of the area in a simiilar square boundary by remaining drones.

Improvements to this software is being made in a separate branch.
 
