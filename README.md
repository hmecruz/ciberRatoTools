
# CiberRato Robot Simulation Environment <br/> Universidade de Aveiro / IEETA 

## Information

CiberRato Robot Simulation Environment simulates the movement
of robots inside a labyrinth.  Robots objective is to go from their
starting position to beacon area and then return to their start position.

The MicroRato competition
[http://microrato.ua.pt/], held annually at Aveiro University, 
uses these these tools for its Explorer league.

## Contents

* simulator -           The simulator source code
* Viewer -              The Visualizer source code
* logplayer -           The logplayer source code
* GUISample -           Graphical robot agent (C++) source code
* robsample -           robot agent (C) source code
* jClient -             robot agent (Java) source code
* pClient -             robot agent (Python) source code
* Labs -                examples of labyrinths used in previous competitions
* startAll -            script that runs the simulator, the visualizer and 5 GUISamples
* startSimViewer -      script that runs the simulator and the Viewer


Additionally, this repository contains an **agent developed for four competition challenges** (see [Challenges](#challenges)).  

---

## Installation  

The source code was compiled with **gcc/g++ (v9.3.0)** using **Qt libraries (v5.12.8)** on **Ubuntu 20.04**.  

### 1. Install dependencies  

```bash
sudo apt-get install build-essential cmake qtmultimedia5-dev
```

### 2. Build the simulation environment

```bash
mkdir build
cd build
cmake ..
make
```

### Start the Challenge-Specific Viewer
```bash
./startC1
./startC2
./startC3
./startC4
```

## Running the Developed Agent
My developed agent can be found in the agent/ folder. It was used in four challenges of the MicroRato competition, where I achieved:

- **Challenge 1** – 🥈 2nd place  
- **Challenge 2** – 🥇 1st place  
- **Challenge 3** – 🥈 2nd place  
- **Challenge 4** – 🥈 2nd place

### Setup
#### 1. Create and activate a Python virtual environment:
```bash
python3 -m venv venv
source venv/bin/activate   # Linux / macOS
venv\\Scripts\\activate      # Windows
```

#### 2. Install Python requirements:
```bash
pip install -r requirements.txt
```

#### Run a Challenge:
Navigate to the corresponding challenge directory (C1, C2, C3, or C4) and run:
```bash
python main.py
```

⚠️ Important: You must be inside the specific challenge directory to start it. The challenges can be found at agent/C#/.


## Challenges

### Challenge 1 – Control
**Objective:** Control the robot’s movement through an unknown closed circuit as fast as possible, avoiding wall collisions.

### Challenge 2 – Mapping
**Objective:** Explore an unknown maze and extract its map.

### Challenge 3 – Planning
**Objective:** Explore an unknown maze, locate multiple target spots, and compute the shortest closed path visiting all spots, starting and ending at the starting position.

### Challenge 4 – Localization, Mapping & Planning

#### Localization
- Navigate and localize the robot in an unknown maze using only noisy sensors (motors, compass, obstacle, beacon).  
- No GPS available; collisions are penalized.

#### Mapping
- Explore the maze, extract the map, and identify all target spots.  
- The agent must return to the starting spot after mapping.

#### Planning
- Compute a minimal-cost closed path visiting all target spots, starting and ending at the starting position.  
- Noise parameters are specified in `C4-config.xml`.

---

## Additional Resources
**Presentations & Challenge Descriptions:**

- `challenges/presentation.pdf`  
- `challenges/challgenge_c1_c2_c3.pdf`  
- `challenges/challgenge_c4.pdf`

## Authors

* Nuno Lau,
  University of Aveiro,
  nunolau@ua.pt

* Artur C. Pereira,
  University of Aveiro,
  artur@ua.pt

* Andreia Melo,
  University of Aveiro,
  abmelo@criticalsoftware.com

* Antonio Neves,
  University of Aveiro,
  an@ua.pt

* Joao Figueiredo,
  University of Aveiro
  joao.figueiredo@ieeta.pt

* Miguel Rodrigues,
  University of Aveiro,
  miguel.rodrigues@ua.pt

* Eurico Pedrosa,
  University of Aveiro,
  efp@ua.pt

 Copyright (C) 2001-2024 Universidade de Aveiro


