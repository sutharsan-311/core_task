# **Omokai \- Robotics Engineering Take-Home Task (Simulation is sufficient)**

## **1\. Core Task (Mandatory \- Simulation Sufficient)**

You may choose **a ground robot** whichever you can demonstrate most convincingly. You can use a simulator of your choice.

1. Build a working pipeline, in a simulated environment, where  you give a command using a prompt. Then AI understands your intent and converts that into action for a robot. This is how it flow looks like:

   Prompt  \-\>  LLM  \-\>  validated mission JSON  \-\>  deterministic executor  \-\>  simulator 

2. And the robot must **follow a pre-determined path or loop**.  
3. You need to document your approach on how you will solve the problems mentioned in section 2\. 

**Use of any simulator of your choice is sufficient. If you can demonstrate using an actual robot then that is even better.**

**What each stage means, so we're aligned:**

* **Prompt** — A natural-language instruction from the operator, e.g. *"Patrol the perimeter loop twice at 15 metres,"* or *"Drive the inspection route and return to start."*  
* **LLM** — Interprets the intent and emits a structured plan. The LLM should *propose*, not *fly*.  
* **Validated mission JSON** — The plan is expressed as structured JSON and checked against a schema and basic safety/sanity rules (valid waypoints, allowed speed, known commands) before anything executes. This is your guardrail layer.  
* **Deterministic executor** — Reads the validated JSON and issues concrete commands to the vehicle. This layer is deterministic and auditable: the same JSON always produces the same behaviour. The LLM is never directly in the control loop.  
* **Simulator** — a ROS 2 \+ Gazebo / Nav2 setup for a ground robot.

## **2\. Senior/Mid-level Challenges (Pick at least one)**

For **senior or mid level** roles, solve **at least one** of the challenges below. **The challenge should work along with the core task, consider the challenge as an extension of the core task.** You still need to provide an overview of your approach on how you will solve the remaining. The more problems you solve, the stronger the signal and the higher seniority you achieve

You do not have to fully finish a challenge to get credit. A clear, working partial solution plus a sound explanation of how you'd complete it counts.

1. **Multi-agent formations.** Control 2– 3 robots autonomously in a formation, and command them to perform tasks (e.g. *"You three sweep this area in a wedge,"* split a route between them, regroup). This is the hardest challenge — it requires the LLM to issue squad-level intent and a coordination/formation layer to keep the agents aligned.  
2. **SLAM or autonomous navigation.** Integrate SLAM (online mapping \+ localization) or full autonomous navigation, so the agent can navigate an unknown or partially known environment rather than just following fixed waypoints.  
3. **Vision AI target detection \+ follow.** Give the agent a vision capability so that, when it sees a user-defined target, it (a) sends a picture to the human operator, and (b) automatically follows the target. The target type should be configurable by the user.

---

## **3\. Direct Offer Challenge**

**The first candidate to solve all three senior challenges (plus the core task) within 3 weeks of getting the task receives a direct offer:**

* **₹80,000 / month**, fully remote.  
* **₹4,00,000 bonus as part of the contract**  
* **ESOPS Negotiable**  
* **Senior or Staff Title**

A direct offer means a single conversation, not a full interview loop. To qualify, your submission must satisfy every condition in Section 4 — in particular, it must run on our examiner's machine and you must be able to defend and modify your code live.

---

## **4\. Conditions (All required)**

1. **Portability.** The code must run on a Linux environment, on the examiner's laptop — not only on yours. Provide reproducible setup (a Docker image / Dockerfile is strongly preferred; otherwise a clean, tested install script and exact dependency/version list).  
2. **Cite your sources.** Using open-source repos or your past work is fine and expected, but cite every source you build on — repo URL, license, and what you took from it.  
3. **You must own your codebase.** Using AI assistants is fine, but you must understand your code, answer questions about your implementation, and should be able to make changes to your own codebase.  
4. **Documentation.** Provide clean demo documentation: how to install, how to run, what commands to issue, and what to expect.

---

## **5\. What to Submit**

* A repository (or archive) containing your code, Dockerfile/setup, and documentation.  
* A **demo video** recording the simulation doing what you built. Show the prompt going in and the agent executing.  
* A short **write-up** explaining:  
  * Your approach and architecture (how prompt → LLM → JSON → executor → sim fits together).  
  * Which challenges you attempted and how you solved them.  
  * **How you would scale this to harder, real-world problems**   
  * A clear list of cited sources (repos, licenses, past work).

---

## **6\. Open-Source Repos & Tools**

You are encouraged to use or refer to open source repos. We compiled a list of repos ourselves which you can look into. You should also either find work which you are comfortable with or use your own past work. **A note of caution:** these projects target different ROS 2 distributions (Humble / Jazzy / older Noetic), different Gazebo versions (Classic vs. Harmonic), and different flight stacks (PX4 / ArduPilot / Tello). 

### **Simulation & autopilot base**

| Tool | Why it's useful |
| ----- | ----- |
| **PX4 Autopilot \+ PX4 SITL / Gazebo** — github.com/PX4/PX4-Autopilot | Strong default for drone simulation; PX4 SITL runs the real flight stack against a simulated vehicle. |
| **ArduPilot \+ SITL \+ Gazebo** — github.com/ArduPilot/ardupilot | Good for Copter/Rover/Plane; supports Gazebo and multi-vehicle/swarm setups. |
| **MAVSDK-Python** — github.com/mavlink/MAVSDK-Python | Clean Python API for mission upload, telemetry, and offboard control, with examples. |
| **PX4-ROS2-Gazebo Drone Template** — github.com/SathanBERNARD/PX4-ROS2-Gazebo-Drone-Simulation-Template | Clean starting point: quadcopter \+ camera on PX4 \+ Gazebo Harmonic \+ ROS 2 Humble. |
| **px4-ros2-gazebo-simulation** — github.com/nhma20/px4-ros2-gazebo-simulation | Guide for manual \+ autonomous multirotor flight, PX4-in-the-loop. |

### **Prompt / LLM → drone control (core pipeline references)**

| Repo | Why it's useful |
| ----- | ----- |
| **ChatDrones** — github.com/Gaurang-1402/ChatDrones | NL → drone on ROS 2 Humble \+ Gazebo; ROSGPT-style node emits structured JSON commands. Close to the core pipeline. |
| **LLM-controlled-drone** — github.com/pratikPhadte/LLM-controlled-drone | NL commands ("fly a 50 m square pattern", "circle the area") on ROS 2 Jazzy. |
| **ros2-agent-ws** — github.com/limshoonkit/ros2-agent-ws | PX4 \+ ROS 2 \+ locally hosted LLMs/VLMs via Ollama; also covers scene understanding (relevant to the vision challenge). |
| **MAVLink-AI-Agent** — github.com/SuperMK15/MAVLink-AI-Agent | LLM (with RAG) → MAVLink commands; includes a voice path if you want it. |
| **ROS-LLM** — github.com/Auromix/ROS-LLM | General NL-control framework for ROS; useful as an architecture reference. |

### **Ground robot, navigation & SLAM (Challenge 2\)**

| Tool | Why it's useful |
| ----- | ----- |
| **ROS 2 Navigation2 (Nav2)** — github.com/ros-navigation/navigation2 | Standard ROS 2 navigation stack; docs cover navigating while mapping with SLAM. |
| **TurtleBot3 \+ turtlebot3\_simulations** — github.com/ROBOTIS-GIT/turtlebot3\_simulations | Easiest way to test robot navigation, SLAM, Gazebo, and Nav2. |
| **SLAM Toolbox** — github.com/SteveMacenski/slam\_toolbox | Common choice for 2D SLAM in ROS 2 navigation. |
| **RTAB-Map ROS** — github.com/introlab/rtabmap\_ros | 3D/visual SLAM with ROS 2 support and TurtleBot/Nav2 examples. |

### **Multi-agent / swarm (Challenge 1\)**

| Repo | Why it's useful |
| ----- | ----- |
| **PX4\_Swarm\_Controller** — github.com/artastier/PX4\_Swarm\_Controller | Leader-follower formation control on PX4 \+ Gazebo \+ ROS 2 Humble; configurable formation geometry. |
| **px4\_multi\_drone\_sim** — github.com/AntonSHBK/px4\_multi\_drone\_sim | Modular multi-drone PX4 sim with per-drone trajectory commands and Dockerized setup. |
| **gym-pybullet-drones** — github.com/utiasDSL/gym-pybullet-drones | Lightweight multi-agent quadcopter sim with single- and multi-agent examples. |
| **Crazyswarm2** — github.com/IMRCLab/crazyswarm2 | ROS 2 stack for Crazyflie teams; good inspiration for formations. |
| **mavsdk\_drone\_show** — github.com/alireza787b/mavsdk\_drone\_show | MAVLink/PX4 fleet operations, SITL, drone shows, and cooperative autonomy. |

### **Vision AI (Challenge 3\)**

| Tool | Why it's useful |
| ----- | ----- |
| **Ultralytics YOLO** — github.com/ultralytics/ultralytics | Practical detection/segmentation/tracking foundation; exports to ONNX and other deployment formats. |
| **PX4-ROS2-Gazebo-YOLOv8** — github.com/monemati/PX4-ROS2-Gazebo-YOLOv8 | Very relevant: drone sim \+ ROS 2 \+ Gazebo \+ YOLOv8 with a moving target to track. The closest single match to Challenge 3\. |
| **Autonomous-Drone-Navigation-and-Human-Search** — github.com/mirzaxbilal/Autonomous-Drone-Navigation-and-Human-Search-Algorithim | Waypoint following \+ YOLO detection \+ image capture on detection (older ROS Noetic; may need porting). |

* Some of our evaluation principles.  
  * **Core pipeline works end-to-end** (prompt in → agent follows path) — the baseline bar.  
  * **Architecture quality** — Is the LLM kept out of the control loop? Is the JSON validated? Is the executor deterministic and auditable?  
  * **Reproducibility** — Does it run on the examiner's Linux machine from your instructions?  
  * **Depth of challenges attempted** and how well they work.  
  * **Scaling story** — How clearly you reason about taking this from a demo to a real system.

Good luck. We're excited to see what you build.

