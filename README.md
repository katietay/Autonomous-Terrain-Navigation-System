# Autonomous Terrain Navigation System (Forest Rover)
This project was developed for the **2025 NASA/USDA/CSU Hackathon**, this project tackles the challenge of designing an autonomous system capable of navigating dynamic, unmarked environments—such as lunar surfaces or forested terrain—to reach incapacitated individuals or areas of interest for rescue or observation.

The system simulates intelligent, real-time pathfinding using elevation-based terrain analysis, mimicking real-world search-and-rescue missions where optimal paths are unknown, change rapidly, and must be discovered autonomously.

---

## Hackathon Prompt
> **Design an intelligent system to traverse a terrain and provide aid to an incapacitated person in a dynamic, unmarked environment.**  
> The system must work in both **forest** and **lunar** settings, adapt to real-time changes like fallen trees or shifting terrain, and operate independently—without relying on external control or premarked routes.

---

## Features
- **Custom A\*** Pathfinding: Enhanced A* algorithm with Euclidean distance and terrain resistance based on elevation.
- **Elevation-Aware Navigation**: Interprets grayscale elevation maps where darker pixels represent higher terrain resistance.
- **Real-Time Adaptation**: Supports dynamic obstacle interaction (e.g., adding or removing trees/rocks) during simulation.
- **Interactive Simulation**: Built with Pygame for visual feedback and manual interaction (e.g., setting start/end points).
- **Environment-Agnostic Design**: Framework applicable to both lunar and terrestrial terrains without environmental dependencies.
- **Configurable Missions**: Loads start/end locations via JSON files to simulate different rescue scenarios.

---

## Installation

### Requirements

- Python 3.7 or later  
- Jupyter Notebook or Google Colab  
- Python libraries:
 - `pygame`
  - `numpy`
  - `pillow`
  - `json`

### Setup

Clone the repository and install dependencies:

```bash
git clone https://github.com/your-username/Autonomous-Terrain-Navigation-System.git
cd Autonomous-Terrain-Navigation-System
pip install pygame numpy pillow
```

## Usage
To run the simulation, execute the main Python script:

```bash
python main.py
```

### Controls
- Left-click: Place the start and end points on the map (first click = start, second click = end)
- Spacebar: Run the A* pathfinding algorithm and display the optimal route
- R: Regenerate a new randomized terrain layout
- T: Toggle terrain overlays (trees, rocks, cliffs)
- C: Clear the current path and reset start/end points

### Configuration
You can preload specific coordinates by editing config.json:

```json
{
  "start": [x1, y1],
  "end": [x2, y2]
}
```
The simulation will automatically use these coordinates if present and valid.

### Elevation Input
The system uses shade.tiff, a grayscale elevation map where lighter pixels represent low elevation (easier to traverse) and darker pixels represent high elevation (more resistant). This data directly influences the A* pathfinding cost function.

## Project Structure

````markdown
Autonomous-Terrain-Navigation-System/
├── main.py
├── config.json
├── shade.tiff
├── utils/
│   └── elevation_utils.py
├── assets/
│   ├── tree.png
│   ├── rock.png
│   └── cliff.png
├── README.md
└── LICENSE
````

## Contributing
Contributions are welcome. To contribute:
1. Fork the repository  
2. Create a new branch:  
   `git checkout -b feature/your-feature-name`  
3. Commit your changes:  
   `git commit -m "Add feature"`  
4. Push to your branch:  
   `git push origin feature/your-feature-name`  
5. Open a pull request

## License
This project is licensed under the [MIT License](LICENSE). You may use, modify, and distribute it under the terms of this license.

## Contact
For questions, collaboration, or feedback:
- **Name**: Katie Taylor  
- **Email**: katietaylorcruz@gmail.com  
- **GitHub**: [https://github.com/katietay](https://github.com/katietay)
