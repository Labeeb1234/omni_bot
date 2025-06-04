- Software: NVIDIA IsaacSim (4.5.0)
  
## Human Dynamic Obstacles (IsaacSim)

- For human simulation there are many ways to do this, one is the painful way of creating an articulation rig with all the joints and physics colliders for each links and stuff but the motion planning for this will then be similar to what we do for standard humanoid based robots. Another way is the easier way (no pain) using the standard IsaacSim framework for agent simulation (IsaacSim.Replicator.Agent.Core).
- The agent core framework can be used to simulate and animate many common actions on both human assets as well as rigged robots too, but commonly this is used for people simulation; the people assets used in this framework core is rigged using skeleton frame for proper human actions (like sit, walk, lookaround, even custom one which we can setup ourselves). The base actions like sit, walk and lookaround is programmed for simulation using the behaviour script in IsaacSim. The entire thing is made modular because of behaviour script usage. The framework was then made into an extension which is optionally enabled based on the requirements of the user.
- For enabling the extension and checking it out follow this [doc](https://docs.isaacsim.omniverse.nvidia.com/latest/replicator_tutorials/tutorial_replicator_agent.html#enable-isaacsim-replicator-agent) from the official website of IsaacSim.
- For [getting started](https://docs.isaacsim.omniverse.nvidia.com/latest/replicator_tutorials/tutorial_replicator_agent.html#getting-started). Now here you can copy paste your custom scene as well as control the number of people you need on the scene as well as the cameras required in the scene; note after specifying all these things on the AgentSDG UI, press **save as** and save the config file (which you can use later on use to open you custom setup with that saved yaml instead of doing it all over again with your custom setup). Format of usual [config file](https://docs.isaacsim.omniverse.nvidia.com/latest/replicator_tutorials/tutorial_replicator_agent.html#configuration-file).
- Before giving cmd to move the loaded human assets we need to add a navmesh volume to your custom world which can be easily done by 


## Other Dynamic Obstalces (IsaacSim
