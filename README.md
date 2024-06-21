# Zeppelins Battle Simulation

## What is Zeppelins Battle Simulation?
This is a simulation of 4 Zeppelins: a player zeppelin and 3 enemy zeppelins in OpenGl on a Macbook system. The player zeppelin has to destroy all zeppelin enemies and enter the ancient temple. Each time the player zeppelin hits any of the zeppelin enemies, the zeppelin enemy will get destroyed and respawn again. In order to permanently destroy all zeppelin enemies, the zeppelin player has to destroy the sun which is located in the middle of the ancient temple.  

Watch demo [here](https://drive.google.com/file/d/1WfwtiW9E4qKwkQAC6iDxzPj_ouPKKfDN/view?usp=drive_link)   
<img width="1439" alt="Screenshot 2024-02-06 at 5 50 38 AM" src="https://github.com/CoboAr/Battle-of-Zeppelins/assets/144629565/879c6d20-6484-466a-afbc-eb27d186113d">



## Requirements
Set up OpenGl on Macbook. Follow guide [here](file:///Users/arnoldcobo/Downloads/setup-guide-Macbook.pdf)

## Game functionality
<ul>
  <li>The body of all Zeppelins is texture mapped.</li>
  <li>Zeppelins are created by using glu, glut shape primitives but also with writeOBJ() and readOBJ() functions which export the current surface of revolution mesh into a 3D text file called “mesh.obj”</li>
  <li>FPV (first person view) camera for the player zeppelin you are controlling. Whenever the "f" key is pressed, the camera view will switch between the FPV camera and the normal world camera view.</li>
  <li>Capability of firing missiles from the zeppelins. Player zeppelin fires missiles each time the "h" key is pressed. Enemy zeppelins, fire missiles automatically whenever they are close to the player Zeppelin.</li>
  <li>Collision handling logic. When the missile hits either the player zeppelin or the enemy zeppelin, they should get destroyed and reappear on a new random position.</li>
  <li>Special effects when zeppelins are hit by a missile and get destroyed.</li>
  <li> Enemy zeppelins move automatically based on three states: 1)Circle movement or patrol 2)chase_player (when player zeppelin gets close to the enemy zeppelin, enemy zeppelin enters a new state called chase-player) 3) fire_missiles (when the enemy zeppelin gets too close, it should turn to face the player and starts firing missiles. )</li>
  <li>Surrounding environemnt.</li>
  <ul>
    <li> A texture mapped ground mash which represent a moving ocean.</li>
    <li>Ancient temple built with columns</li>
    <li>A sun</li>
    <li>A moon</li>
    <li>Snow effect</li>
    <li>Icebergs into the ocean, which currently are commented. In order to see them you just need to uncomment drawiceberg() code section.</li>
  </ul>
  <li>Collision detection: To determine if a missile has collided with a zeppelin, a bounding box has been used to surround all zeppelins zeppelins which is also visualized. It can be hidden by commenting out the draw bounding box code sections accordingly. </li>
  <li>Music incorporated into the game. As soon as the game starts, "Worakls - By the brook" song will start playing.</li>
</ul>

## Zeppelin functionality
These are the functionalities associated with each key press:
<ul>
  <li>Press "w" to make the Zeppelin move forward.</li>
  <li>Press "s" to make the Zeppelin move backward.</li>
  <li>Press "a" to make the Zeppelin turn to the left.</li>
  <li>Press "d" to make the Zeppelin turn to the right.</li>
  <li>Press "i" to make the Zeppelin move upward.</li>
  <li>Press "k" to make the Zeppelin move downward.</li>
  <li>Press "h" to fire missiles from the Zeppelin.</li>
  <li>Press "f" to switch the camera between FPV (first-person camera view) and the normal world camera view.</li>
  <li>Press "q" to stop the program and exit the game.</li>
</ul>

## Demo

Click the link [here](https://drive.google.com/file/d/1yIeIS9aPVgDQNMAk1fDLltxBCzBFo9c0/view?usp=drive_link) to access the file on Google Drive. I uploaded the video directly to Google Drive so it can be watched. Due to its large size, direct uploading was necessary.


Enjoy! And please do let me know if you have any comments, constructive criticism, and/or bug reports.
## Author
## Arnold Cobo

