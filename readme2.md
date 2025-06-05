- Software: NVIDIA IsaacSim (4.5.0)
  
## Human Dynamic Obstacles (IsaacSim)

- For human simulation there are many ways to do this, one is the painful way of creating an articulation rig with all the joints and physics colliders for each links and stuff but the motion planning for this will then be similar to what we do for standard humanoid based robots. Another way is the easier way (no pain) using the standard IsaacSim framework for agent simulation (IsaacSim.Replicator.Agent.Core).
- The agent core framework can be used to simulate and animate many common actions on both human assets as well as rigged robots too, but commonly this is used for people simulation; the people assets used in this framework core is rigged using skeleton frame for proper human actions (like sit, walk, lookaround, even custom one which we can setup ourselves). The base actions like sit, walk and lookaround is programmed for simulation using the behaviour script in IsaacSim. The entire thing is made modular because of behaviour script usage. The framework was then made into an extension which is optionally enabled based on the requirements of the user.
- For enabling the extension and checking it out follow this [doc](https://docs.isaacsim.omniverse.nvidia.com/latest/replicator_tutorials/tutorial_replicator_agent.html#enable-isaacsim-replicator-agent) from the official website of IsaacSim.
- For [getting started](https://docs.isaacsim.omniverse.nvidia.com/latest/replicator_tutorials/tutorial_replicator_agent.html#getting-started). Now here you can copy paste your custom scene as well as control the number of people you need on the scene as well as the cameras required in the scene; note after specifying all these things on the AgentSDG UI, press **save as** and save the config file (which you can use later on use to open you custom setup with that saved yaml instead of doing it all over again with your custom setup). Format of usual [config file](https://docs.isaacsim.omniverse.nvidia.com/latest/replicator_tutorials/tutorial_replicator_agent.html#configuration-file).
- Before giving cmd to move the loaded human assets we need to add a navmesh volume to your custom world for this first enable the Agent SGD extensions (both core and UI extensions) and then right click on the world prim->create->create navmesh volume. The navmesh volume can be edited to exclude an include assets; first of the navmesh is required to enable agent based animations of assets as well as navigation and obstacle avoidance behaviours of the considered assets. To know more about how to enble navmesh check [here](https://docs.isaacsim.omniverse.nvidia.com/latest/replicator_tutorials/ext_replicator-agent/customization.html#building-the-navmesh) and to know more about what navmesh is in detail check [this](https://docs.omniverse.nvidia.com/extensions/latest/ext_navigation-mesh.html).
- After adding the navmesh volume and excluding the omni bot asset from it to prevent the human assets to avoid it as it moves to the specified waypoints and loading the agent env with the custom setup (load the saved config if the setup is already created by you). The framework also helps in recording datasets for agent (human/robot/etc) behaviours which can be used for Foundational ML models for training and stuff (optional up to you to record but saving and loading any changes to the agent env that you created is necessary for it to reflect changes).
- Giving cmd to the agent(people in this case):
  - For the main character (the character with no number suffix) we can give the cmds via the "Agent SDG" UI as shown below. The commands give in this way executes on sim play.
      <div>
        <img src="https://github.com/user-attachments/assets/83745c9b-c092-449d-8188-4213d0850ff4" alt="initial command"/>
      </div>
  - To modify the commands during sim play use the command injection widget (show below), over here we can specify which character to give the command to by adding its prim name as prefix to the given command as shown in the pic below circled in yellow :|
      <div>
        <img src="https://github.com/user-attachments/assets/824d3ff4-6689-4b3f-b610-10f316b9df37" alt="command injection widget"/>
      </div>
  - To open the "command injection" widget follow [this](https://docs.isaacsim.omniverse.nvidia.com/latest/replicator_tutorials/ext_replicator-agent/agent_control.html#command-injection).
  - PS: The commands are executed one after the other just like a standard program execution in both cases.
- Now after all the setup is done one issue that can happen is that the loaded people assets are just meshes with a skeleton (which gives a more human behaviour to its motion) so no physics based collision will be present for this all we have to do is go to the skelroot of the Character (anyone) right click on that prim and add rigid body attribute to to the prim (or you can add rigid body with collider presets your wish), then go to the properties tab of the skelroot and go to the rigid body section and enable kinematics, rigid body is already enabled anyways after adding rigid body. The required sections are marked below in the pic for reference.
      <div>
        <img src="https://github.com/user-attachments/assets/924a369a-db91-410e-b081-79abee9d0c76" alt="reference markers"/>
      </div>
- Finally things will look like this (if the collider view is enabled)
      <div>
        <img src="https://github.com/user-attachments/assets/35a2681a-3a38-4b1a-9be8-2e57b7d18b91" alt="setup"/>
      </div>
  - Here in the gif above you can **see the Lidar projection points around the legs** of the moving character asset, this won't happen in the default setting of the character asset as they are just meshes without any physics based attributes (this need to be manually added by us {as of 2025}).




## Other Dynamic Obstalces (IsaacSim)
