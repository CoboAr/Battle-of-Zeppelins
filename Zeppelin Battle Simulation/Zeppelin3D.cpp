/********************************************************************
           Hierarchical Multi-Part Model Example ARNOLD COBO
 **********************************************************************************************************/
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#define GL_SILENCE_DEPRECATION

#define _USE_MATH_DEFINES
#include <math.h>

#include <iostream>
#include <string>
#include <sstream>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <utility>
#include <vector>
#include <array>
#include "VECTOR3D.h"
#include "QuadMesh.h"
#define M_PI 3.14159265358979323846

#include <cstdlib>  // Include for rand() function
#include <ctime>    // Include for srand() function
#include <chrono>
#include <thread>
#include <random>


#include "SimplexNoise.h" // library used for terrain

// Texture mapping declarion of include
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"


#import <OpenAL/OpenAL.h>
#include <OpenAL/alc.h>
#include "sndfile.h"


// Playing music on the background while game is going on
ALCdevice* device;
ALCcontext* context;

// Global variables for music
ALuint buffer;
ALuint source;

void initializeAudio() {
    device = alcOpenDevice(NULL);
    if (!device) {
        std::cerr << "Failed to initialize audio device" << std::endl;
        return;
    }

    context = alcCreateContext(device, NULL);
    alcMakeContextCurrent(context);
}

void loadAudio(const char* filePath, ALuint* buffer) {
    SNDFILE* sndfile;
    SF_INFO info;

    // Open audio file
    sndfile = sf_open(filePath, SFM_READ, &info);
    if (!sndfile) {
        std::cerr << "Failed to open audio file: " << filePath << std::endl;
        return;
    }

    // Read audio data
    short* data = new short[info.frames * info.channels];
    sf_read_short(sndfile, data, info.frames * info.channels);

    // Close audio file
    sf_close(sndfile);

    // Generate OpenAL buffer
    alGenBuffers(1, buffer);

    // Upload audio data to buffer
    alBufferData(*buffer, info.channels == 2 ? AL_FORMAT_STEREO16 : AL_FORMAT_MONO16, data, info.frames * info.channels * sizeof(short), info.samplerate);

    delete[] data; // Free allocated memory
}

void cleanupAudio() {
    alcDestroyContext(context);
    alcCloseDevice(device);
}


// Function to play music asynchronously
void playMusicAsync(const char* filePath) {
    initializeAudio();
    loadAudio(filePath, &buffer);
    
    alGenSources(1, &source);
    alSourcei(source, AL_BUFFER, buffer);
    alSourcePlay(source);
}

// Enemy Zeppelins initial position
float objectposition1[]={0.0,0.0,0.0}; // position of zeppelin player
float objectposition2[]={-50.0,20.0,-150.0}; // position of zeppelin1 enemy
float objectposition3[]={20.0,40.0,-190.0}; // position of zeppelin2 enemy
float objectposition4[]={-30.0,0.0, -130.0}; // position of zeppelin3 enemy
float sunPosition[3] = { 0.0f, 10.0f, -10.0f }; // sun's initial position before transformations
float sunFinalPosition[3] = { 0.0f, 85.0f, -255.0f }; // sun's final position


const int vWidth  = 1450;    // Viewport width in pixels
const int vHeight = 800;    // Viewport height in pixels

// Note how everything depends on Zeppelin body dimensions so that can scale entire robot proportionately
// just by changing robot body scale
float zeppelinBodyWidth = 15.0;
float zeppelinBodyLength = 8.0;
float zeppelinBodyDepth = 6.0;
float cabinWidth = 0.4*zeppelinBodyWidth;
float cabinLength = 0.4*zeppelinBodyLength;
float cabinDepth = 0.4*zeppelinBodyDepth;
float stanchionLength = zeppelinBodyLength;
float stanchionRadius = 0.1*zeppelinBodyDepth;


// Control Zeppelin body rotation on base
float zeppelinUserAngle = 0.0;

float zeppelinAngleUp = -90;
float zeppelinAngleDown = 90;
//Zoom in/Zoom out Zeppelin model
float x = 0.0;
float y = 0.0;
float z = 0.0;

static GLfloat spin  = 0.0;
static GLfloat theta = 0.0;
// When flag is true the blades of the propellor keep spinning, otherwise they stop.
bool start=true;
bool stop = false;
bool forward = true;
bool upward = true;
bool downward = true;

/* If moveUpDown > 0.0, Zeppelin will move vertically up.
If moveUpDown < 0.0, Zeppelin will move vertically down.
If moveUpDown = 0, Zeppelin moves only horizontalyy. */
float moveUpDown = 0.0;

// variables used for texture mapping
GLuint body_texture1; // Declare the texture ID variable for user body Zeppelin
GLuint body_texture2; // Declare the texture ID variable for enemy body Zeppelin
GLuint body_texture3; // Declare the texture ID variable for enemy body Zeppelin
GLuint body_texture4; // Declare the texture ID variable for enemy body Zeppelin
GLuint smokeEffect;
GLuint fireEffect;
GLuint explosionEffect;
GLuint UpperFin_texture; // Declare the texture ID variable
GLuint BottomFin_texture; // Declare the texture ID variable
GLuint LeftFin_texture; // Declare the texture ID variable
GLuint RightFin_texture; // Declare the texture ID variable
GLuint currentEffectTexture;
GLuint sunTexture;
GLuint groundTexture;
GLuint mountainTexture;
GLuint iceberg_texture;
GLuint shark_texture;
GLuint column_texture;
GLuint fire_texture;
GLuint moon_texture;
GLuint snow_texture;
GLuint MissileEnemyTexture;

/* Angle to rotate propellor up if Zeppelin if moving up,
 down if Zeppelin is moving down, horizontal position if Zeppelin is moving horizontally. */
float angleleftpropeller = 0.0;
/* Coordinate points to translate left and right propellor up or down based on the moving direction. */
float xx=0.0;
float yy=0.0;
float zz=0.0;

// Declare Missile Variables
bool missileFiredPlayer = false;
bool deleteEnemyZeppelin = false;

// Define a global variable to track the deletion time of sun
std::chrono::high_resolution_clock::time_point deletionStartTime;

// control timing for changing fire and explosion texture mapping after missile hit
std::chrono::time_point<std::chrono::high_resolution_clock> effectStartTime;
bool effectInProgress = false;


typedef struct Vector3D
{
    GLfloat x, y, z;
} Vector3D;

// Quads and Vertices of the surface of revolution
typedef struct Vertex
{
    GLfloat x, y, z;
    Vector3D normal; // normal vector for this vertex
    int numQuads;
    int quadIndex[4]; // index into quad array - tells us which quads share this vertex
} Vertex;

// structure for snow flakes
struct ParticleSnow {
    float x, y, z;  // Position
    float velocityY;  // Vertical velocity
};

// initialization of vector to save snow flakes instances
std::vector<ParticleSnow> particlesSnow;

// structure for particles created when zeppelin is hit by a missile
struct ParticleFire {
    float x, y, z;
    float speedX, speedY, speedZ;
    float lifespan;
};

std::vector<ParticleFire> particlesFire;


// ReadOBJ() variables
unsigned int numTris = 0;
unsigned int numVertices = 0;
unsigned int numIndices = 0;

// Define a struct to represent a missile of Player Zeppelin
struct MissilePlayer {
    bool active;
    float positionPlayer[3];
    float initialDirectionPlayer[3];
    float speedPlayer;
};

// Declare a vector to store multiple missiles
std::vector<MissilePlayer> missilesPlayer;

void initMissilesPlayer() {
    // Initialize the missiles vector
    missilesPlayer.clear();
}

// Define a struct to represent a missile of Enemy Zeppelin
struct MissileEnemy {
    bool active;
    float positionEnemy[3];
    float initialDirectionEnemy[3];
    float speedEnemy;
};
// Vector to store missiles
std::vector<MissileEnemy> missilesEnemy;

void initMissilesEnemy() {
    // Initialize the missiles vector
    missilesEnemy.clear();
   }

// Requirement 1.c declaration of readOBJ() function variables
// third bonus
Vertex   *varray3;     // array of vertices
Vector3D *positions3;  // vertex positions - used for VBO draw
Vector3D *normals3;    // normal vectors for each vertex - used for VBO draw
GLuint *indices3;

bool fpvMode = false; // Flag to track FPV mode (first person view mode)

// Initial object position
float objectPositionPlayer[3];
// Forward movement vector
float forwardMovingVector[] = { -1.0f, 0.0f, 0.0f };

// Moving distance, it can be adjusted so the moving distance after translation can be bigger or smaller accordingly.
float movingDistance = 0.5f;


void updateZeppelinDirection() {
    // Update the forward vector based on the rotation angle
    float angleInRadians = zeppelinUserAngle * M_PI / 180.0f;
    // Calculate the new forward vector
    forwardMovingVector[0] = -cos(angleInRadians);
    forwardMovingVector[2] = sin(angleInRadians);
}

void updateBoundingBox(float objectPositionPlayer[3]);
//Zeppelin moves forward
void updateZeppelinPlayerPositionf() {
    // Update the zeppelin's position based on the forward vector and moving distance
    for (int i = 0; i < 3; i++) {
        objectPositionPlayer[i] += forwardMovingVector[i] * movingDistance;
    }
    updateBoundingBox(objectPositionPlayer);
}

//Zeppelin moves backward
void updateZeppelinPlayerPositionb() {
    // Update the zeppelin's position based on the forward vector and moving distance
    for (int i = 0; i < 3; i++) {
        objectPositionPlayer[i] -= forwardMovingVector[i] * movingDistance;
        updateBoundingBox(objectPositionPlayer);
    }
}

// Lighting/shading and material properties for Zeppelin
// Zeppelin RGBA material properties

//zeppelin body color
GLfloat zeppelinBody_mat_ambient[] = /*{ 0.0F, 0.0F, 0.0F }*/ { 0.25F, 0.25F, 0.25F };
GLfloat zeppelinBody_mat_specular[] = /*{ 0.60F, 0.60F, 0.50F }*/ { 0.774597F, 0.774597F, 0.774597F };
GLfloat zeppelinBody_mat_diffuse[] = /*{ 0.5F, 0.5F, 0.0F }*/ { 0.4F, 0.4F, 0.4F };
GLfloat zeppelinBody_mat_shininess[] = { 35.0F};

//cabin color
GLfloat zeppelinCabin_mat_ambient[] = { 0.25F, 0.25F, 0.25F };
GLfloat zeppelinCabin_mat_specular[] = { 0.774597F, 0.774597F, 0.774597F };
GLfloat zeppelinCabin_mat_diffuse[] = { 0.4F, 0.4F, 0.4F };
GLfloat zeppelinCabin_mat_shininess[] = { 32.0F };

//propellor color
GLfloat zeppelinPropellor_mat_ambient[] = { 0.1745F, 0.01175F, 0.01175F };
GLfloat zeppelinPropellor_mat_specular[] = { 0.727811F, 0.626959F, 0.626959F };
GLfloat zeppelinPropellor_mat_diffuse[] = { 0.61424F, 0.04136F, 0.04136F };
GLfloat zeppelinPropellor_mat_shininess[] = { 32.0F };

//stanchion color
GLfloat zeppelinStanchion_mat_ambient[] = { 0.02F, 0.02F, 0.02F };
GLfloat zeppelinStanchion_mat_specular[] = { 0.4F, 0.4F, 0.4F };
GLfloat zeppelinStanchion_mat_diffuse[] = { 0.01F, 0.01F, 0.01F };
GLfloat zeppelinStanchion_mat_shininess[] = { 32.0F };

// Light properties
GLfloat light_position0[] = { -4.0F, 8.0F, 8.0F, 1.0F };
GLfloat light_position1[] = { 4.0F, 8.0F, 8.0F, 1.0F };
GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat light_ambient[] = { 0.2F, 0.2F, 0.2F, 1.0F };


// A flat open mesh
QuadMesh *groundMesh = NULL;
// Default Mesh Size
int meshSize = 1;

// Prototypes for functions in this module
void initOpenGL(int w, int h);
void display(void);
void reshape(int w, int h);
void keyboard(unsigned char key, int x, int y);
void functionKeys(int key, int x, int y);
//void drawZeppelin(GLuint texture);
void drawZeppelin(GLuint texture, float objectPosition[3]);
//void drawZeppelinEnemy(GLuint texture, float objectposition[3]);
void drawZeppelinEnemy(GLuint texture);

//void drawBody();
void drawBody(GLuint texture);
void drawBodyWithEffects(GLuint bodyTexture, GLuint smokeTexture, GLuint fireTexture, GLuint explosionTexture);
void drawCabin();
void drawFins();
void drawUpFin();
void drawBottomFin();
void drawLeftFin();
void drawRightFin();
void drawPropellorRear();
void drawPropellorLeft();
void drawPropellorRight();
void drawBlades();
void spinDisplay(int);
void RotateVectorY(float x, float y, float z, float angle);
void updateZeppelinPlayerPositionf();
void updateZeppelinPlayerPositionb();
void drawStanchion1();
void drawStanchion2();
void animationHandlerUp(int param);
void animationHandlerDown(int param);
void animationHandlerForward(int param);
void readOBJ(const std::string& fileName);
void drawTris3(Vector3D *positions3, Vector3D *normals3, GLuint *indices3, GLuint numIndices);
void setupWorldCamera();
void setupFPVCamera(float eyeX, float eyeY, float eyeZ);
void drawMissilePlayer();
void fireMissilePlayer();
void drawMissileEnemy();
void initMissilesPlayer();
void updateMissilesPlayer();
void updateMissilesEnemy();
void updateFireMissilesEnemy();
void drawMoon();
void drawIceberg();
void drawColumn(GLuint texture);
void drawColumns();
void drawSnow();
void drawSun();
void drawBoundingBox(const float sunPosition[3]);

// Class and functions to control enemy Zeppelins
class EnemyZeppelin {
    
public:
    enum ZeppelinMovementState {
        CIRCLE_MOVEMENT,
        CHASE_PLAYER,
        FIRE_MISSILES
    };
    ZeppelinMovementState state = CIRCLE_MOVEMENT;
    float chaseRange = 80.0f;  // Set your desired chase range
    float fireRange = 55.0f;    // Set your desired fire range
    float baseSpeed = 2.5f;     // Base speed of the zeppelinEnemy
    float chaseSpeed = 5.5f;    // Speed when chasing the zeppelinPlayer
    float currentSpeed = baseSpeed;
    
    bool missileFiredEnemy = false;
    float zeppelinMinBounds[3];
    float zeppelinMaxBounds[3];
    float offsetFromZeppelin = -5.0f;
    
public:
    bool disappearing = false;
    std::chrono::high_resolution_clock::time_point disappearanceStartTime;
    
public:
    float position[3];
    float zeppelinEnemyAngle;
    float forwardMovingVectorEnemy[3] = {-1.0f, 0.0f, 0.0f};
    float movingDistance = 0.5;
    bool collisionOccurred = false; // Flag to indicate if collision has occurred
    
public:
    EnemyZeppelin(float initialAngle, float initialPosition[3], float distance)
            : zeppelinEnemyAngle(initialAngle), movingDistance(distance) {
            for (int i = 0; i < 3; ++i) {
                position[i] = initialPosition[i];
                forwardMovingVectorEnemy[i] = 0.0f;
            }
                updateZeppelinEnemyDirection();
                initMissilesEnemy();  // Initialize missiles

        }
    void updateZeppelinEnemyDirection() {
        // Update the forward vector based on the rotation angle
        float angleInRadians = zeppelinEnemyAngle * M_PI / 180.0f;
        // Calculate the new forward vector
        forwardMovingVectorEnemy[0] = -cos(angleInRadians);
        forwardMovingVectorEnemy[2] = sin(angleInRadians);
        // Update the angle based on the movement direction
        zeppelinEnemyAngle = atan2(forwardMovingVectorEnemy[2], -forwardMovingVectorEnemy[0]) * 180.0f / M_PI;
    }
    //Zeppelin moves forward
    void updateZeppelinEnemyPositionf() {
        // Update the zeppelin's position based on the forward vector and moving distance
        for (int i = 0; i < 3; i++) {
            position[i] += forwardMovingVectorEnemy[i] * movingDistance * currentSpeed;
        }
    }
    // Function to update the automatic movement of the enemy zeppelin
        void updateAutomaticMovement() {
            switch (state) {
                case CIRCLE_MOVEMENT:
                    updateCircleMovement();
                    break;
                case CHASE_PLAYER:
                    updateChasePlayer();
                    break;
                case FIRE_MISSILES:
                    updateFireMissilesEnemy();
                    break;
            }
        }
    void drawZeppelinEnemy(GLuint bodyTexture,GLuint smokeTexture, GLuint fireTexture, GLuint explosionTexture ) {
        glPushMatrix();
        // Translate to the updated position
        glTranslatef(position[0], position[1], position[2]);
        // Rotate based on the enemyZeppelin's angle
        glRotatef(zeppelinEnemyAngle, 0.0, 1.0, 0.0);
        if (collisionOccurred== true)
        {
            glMatrixMode(GL_TEXTURE);
            glLoadIdentity();
            glTranslatef(0.0, 4.8, 0.0);  // Translate the texture along the y-axis
            // Switch back to the modelview matrix
            glMatrixMode(GL_MODELVIEW);
            drawBodyWithEffects(bodyTexture, smokeTexture, fireTexture, explosionTexture);
        }
        else{
            glMatrixMode(GL_TEXTURE);
            glLoadIdentity();
            glTranslatef(0.9, 0.9, 10.0);  // Translate the texture along the y-axis
            // Switch back to the modelview matrix
            glMatrixMode(GL_MODELVIEW);
            drawBody(bodyTexture);
        }
        // Draw the zeppelin components (cabin, fins, propellers, etc.)
        drawCabin();
        drawFins();
        drawPropellorRear();
    //  drawPropellorLeft();
    //  drawPropellorRight();
        glPopMatrix();
    }
public:
    void updateCircleMovement() {
        // Incremental rotation value
        float rotationStep = 0.9f;
        // Move forward
        updateZeppelinEnemyPositionf();
        // Rotate the zeppelin
        zeppelinEnemyAngle += rotationStep;
        updateZeppelinEnemyDirection();
        updateZeppelinEnemyPositionf();
        // Check if it's time to transition to CHASE_PLAYER state
        float distanceToPlayer = calculateDistance(position, objectPositionPlayer);
        if (distanceToPlayer < chaseRange) {
            state = CHASE_PLAYER;
        }
    }
    void updateChasePlayer() {
        // Calculate the vector pointing from the enemy to the player using player's position
        float vectorToPlayer[3];
        for (int i = 0; i < 3; i++) {
            vectorToPlayer[i] = objectPositionPlayer[i] - position[i];
        }
        // Calculate the distance to the player
        float distanceToPlayer = calculateDistance(position, objectPositionPlayer);
        // Normalize the vector
        for (int i = 0; i < 3; i++) {
            vectorToPlayer[i] /= distanceToPlayer;
        }
        // Update the zeppelin's angle to face the player's position directly
        zeppelinEnemyAngle = atan2(vectorToPlayer[2], -vectorToPlayer[0]) * 180.0f / M_PI;
        // Correct the angle to ensure it faces the player regardless of its direction
        if (zeppelinEnemyAngle < 0) {
            zeppelinEnemyAngle += 360.0f;
        } else if (zeppelinEnemyAngle > 360.0f) {
            zeppelinEnemyAngle -= 360.0f;
        }
        // Update the zeppelin's direction
        updateZeppelinEnemyDirection();
        // Update the position based on the normalized vector and moving distance
        for (int i = 0; i < 3; i++) {
            position[i] += vectorToPlayer[i] * chaseSpeed * movingDistance;
        }
        // Check if it's time to transition to FIRE_MISSILES state
        if (distanceToPlayer < fireRange) {
            state = FIRE_MISSILES;
            fireMissileEnemy();
        } else if (distanceToPlayer > chaseRange) {
            // If player goes far enough, return to CIRCLE_MOVEMENT state
            state = CIRCLE_MOVEMENT;
        }
    }
    float calculateDistance(float point1[3], float point2[3]) {
        float dx = point2[0] - point1[0];
        float dy = point2[1] - point1[1];
        float dz = point2[2] - point1[2];
        return sqrt(dx * dx + dy * dy + dz * dz);
    }
    void fireMissileEnemy() {
        // Create a new missile and add it to the vector
        MissileEnemy newMissile;
        newMissile.active = true;
        missileFiredEnemy = false;
        if (!missileFiredEnemy) {
            // Set missile position to the Zeppelin's position (or adjust as needed)
            for (int i = 0; i < 3; i++) {
                newMissile.positionEnemy[i] = position[i];
            }
            // Set missile direction based on Zeppelin's orientation
            newMissile.initialDirectionEnemy[0] = -cos(zeppelinEnemyAngle * M_PI / 180.0f);
            newMissile.initialDirectionEnemy[2] = sin(zeppelinEnemyAngle * M_PI / 180.0f);
            newMissile.speedEnemy = 1.0f;
            // assign vertical position of the missile along the y axis when the Zeppelin moves upward/downward
            missilesEnemy.push_back(newMissile);
            missileFiredEnemy = true;
        }
    }
    
    void drawMissileEnemy() {
        if (missileFiredEnemy) {
            // Draw all active missiles
            for (auto& missile : missilesEnemy) {
                if (missile.active) {
                    
                    glPushMatrix();
                    glTranslatef(missile.positionEnemy[0], missile.positionEnemy[1], missile.positionEnemy[2]);

                    // Apply rotation to align the missile with its initial direction
                    float missileOrientation = atan2(missile.initialDirectionEnemy[2], -missile.initialDirectionEnemy[0]) * 180.0f / M_PI;
                    glRotatef(missileOrientation, 0, 1, 0);
                    // Enable texture mapping and set texture parameters
                    glEnable(GL_TEXTURE_2D);
                    glBindTexture(GL_TEXTURE_2D, MissileEnemyTexture);
                    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); // Use MODULATE mode
                    
                    glScalef(10, 1.5, 1);
                    // Use gluSphere to draw the missile
                    GLUquadric* quadric = gluNewQuadric();
                    gluQuadricTexture(quadric, GL_TRUE);
                    gluSphere(quadric, 0.2, 10, 10); // Adjust the radius and resolution as needed
                    gluDeleteQuadric(quadric);

                    glPopMatrix();
                }
            }
        }
        glDisable(GL_TEXTURE_2D);
    }
    
    void updateFireMissilesEnemy() {
        // Update the position of active missiles
        for (auto& missile : missilesEnemy) {
            if (missile.active) {
                // Update missile position based on its direction and speed
                for (int i = 0; i < 3; i++) {
                    missile.positionEnemy[i] += missile.initialDirectionEnemy[i] * missile.speedEnemy;
                }
            }
        }
        state = CIRCLE_MOVEMENT;
        }
};



// Structure to hold instances of EnemyZeppelin
struct EnemyZeppelinContainer {
    std::vector<EnemyZeppelin> zeppelins; // Vector to hold instances of EnemyZeppelin

    // Constructor to initialize instances of EnemyZeppelin and add them to the container
    EnemyZeppelinContainer() {
        // Create instances of EnemyZeppelin and add them to the vector
        EnemyZeppelin zeppelin1(0.0f, objectposition2, 0.1f);
        EnemyZeppelin zeppelin2(-65.0f, objectposition3, 0.1f);
        EnemyZeppelin zeppelin3(90.0f, objectposition4, 0.1f);

        // Add instances to the vector
        zeppelins.push_back(zeppelin1);
        zeppelins.push_back(zeppelin2);
        zeppelins.push_back(zeppelin3);
    }
};
void destroyEnemyZeppelins(EnemyZeppelinContainer& enemyZeppelinsContainer);


// Create an instance of EnemyZeppelinContainer
EnemyZeppelinContainer container;

// Access the first instance of EnemyZeppelin
EnemyZeppelin& zeppelin1 = container.zeppelins[0];
// Access the second instance of EnemyZeppelin
EnemyZeppelin& zeppelin2 = container.zeppelins[1];
// Access the third instance of EnemyZeppelin
EnemyZeppelin& zeppelin3 = container.zeppelins[2];

// collision functions
void handleCollisionEnemy(float missilePosition[3], float zeppelinPosition[3], float zeppelinBodyWidth, float zeppelinBodyLength, float zeppelinBodyDepth,  EnemyZeppelin& zeppelin, MissilePlayer& missilePlayer);

void handleCollisionPlayer(float missilePosition[3], float objectPositionPlayer[3], float zeppelinBodyWidth, float zeppelinBodyLength, float zeppelinBodyDepth, MissileEnemy& missileEnemy);

void handleCollisionSun(const float missilePosition[3], const float sunPosition[3], MissilePlayer& missilePlayer);


void deleteZeppelinEnemy(EnemyZeppelin& zeppelin) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> distribution(-50.0f, 50.0f);  // Adjust the range as needed
    
    // Delay for 0.5 seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Move the zeppelin to a new random position
    zeppelin.position[0] = 1000.0f; // X-coordinate outside the visible range
    zeppelin.position[1] = -1000.0f; // Y-coordinate outside the visible range
    zeppelin.position[2] = 1000.0f; // Z-coordinate outside the visible range

    // Reset any other necessary zeppelin state
    zeppelin.collisionOccurred = false;
    effectInProgress=false;
}

void destroyEnemyZeppelins(EnemyZeppelinContainer& enemyZeppelinsContainer) {
    // Iterate through the enemy zeppelins container
    for (auto& zeppelin : enemyZeppelinsContainer.zeppelins) {
        // Set a flag to indicate that the zeppelin is in the process of disappearing
        // Collision detected
        zeppelin.collisionOccurred = true;
        effectInProgress = true;
        effectStartTime = std::chrono::high_resolution_clock::now();
        
        // Set a variable to indicate that the zeppelin is in the process of disappearing
        zeppelin.disappearing = true;
        zeppelin.disappearanceStartTime = std::chrono::high_resolution_clock::now();
        
        // Perform any additional actions you need to handle the destruction of the zeppelin
        // For example, set them to coordinates that are far beyond the visible range
        // If the zeppelin is disappearing, check the time elapsed to determine when to actually disappear it
        if (zeppelin.disappearing) {
            auto currentTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<float> elapsedTime = currentTime - zeppelin.disappearanceStartTime;
            // Check if 3 seconds have passed
            if (elapsedTime.count() > 1.5f) {
                effectInProgress = false;
                deleteZeppelinEnemy(zeppelin);
                
                // Initialize random seed and create particles
                srand(time(nullptr));
            }
        }

    }
}

// Function to update the scene
void update() {
    // Update the automatic movement of the enemy zeppelins
    zeppelin1.updateAutomaticMovement();
    zeppelin2.updateAutomaticMovement();
    zeppelin3.updateAutomaticMovement();
    
    // Request a redraw to update the display
    glutPostRedisplay();
}

// Callback function for the main loop
void idle() {
    // Call the update function in the main loop
    update();
}

// Requirement 1
// Function to load texture using STB_IMAGE
GLuint loadTexture(const char* filename) {
    int width, height, channels;
    stbi_set_flip_vertically_on_load(true); // Flip the image vertically (OpenGL expects the bottom-left origin)

    unsigned char* image = stbi_load(filename, &width, &height, &channels, STBI_rgb);

    if (!image) {
        std::cerr << "Error loading texture: " << stbi_failure_reason() << std::endl;
        return 0;
    }

    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);
    // Set the texture parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // Specify the texture image
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
    // Generate mipmaps
    gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, width, height, GL_RGB, GL_UNSIGNED_BYTE, image);
    stbi_image_free(image);
    return textureID;
}
// Terrain created in the form of icebergs
const int width = 50;
const int height = 50;
float terrain[width][height];

// Adjust these parameters for different terrains
const float terrainScale = 0.005f;
const float terrainHeightScale = 50.0f;

void generateHeightmap() {
    SimplexNoise noise;

    // Generate heightmap using Perlin noise
    for (int i = 0; i < width; ++i) {
        for (int j = 0; j < height; ++j) {
            float x = i * terrainScale;
            float y = j * terrainScale;
            // Adjust the parameters based on your preferences
            terrain[i][j] = noise.fractal(6, x, y) * terrainHeightScale;
        }
    }
}


void drawTerrain() {
    GLuint mountainTexture = loadTexture("/Users/arnoldcobo/Library/Developer/Xcode/DerivedData/Zeppelin_Battle_Simulation-fleirgtdhybsbihcxczdgdnvmvpk/Build/Products/Debug/iceberg.jpg");
    // Enable texture mapping and set texture parameters
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, mountainTexture);
    // Requirement 1.b
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); // Use MODULATE mode

    for (int i = 0; i < height - 1; ++i) {
        glBegin(GL_TRIANGLE_STRIP);
        for (int j = 0; j < width; ++j) {
            
            // Assuming terrain coordinates are in the range [0, 1]
            float texCoordX = (float)j / (float)(width-1);
            float texCoordY1 = (float)i / (float)(height-1);
            float texCoordY2 = (float)(i + 1) / (float)(height-1);

            glTexCoord2f(texCoordX, texCoordY1);
            glVertex3f(j, terrain[j][i], i);

            glTexCoord2f(texCoordX, texCoordY2);
            glVertex3f(j, terrain[j][i + 1], i + 1);
            
        }
        glEnd();
    }
    glDisable(GL_TEXTURE_2D); // Disable texture mapping

}

//void drawTerrain() {
//    glColor3f(0.8f, 0.8f, 1.0f);  // Light blue color for the iceberg
//
//    // Draw the upper part of the iceberg (dome)
//    glutSolidSphere(2.0, 100, 100);
//
//    // Draw irregular shapes to simulate the lower part of the iceberg
//    glPushMatrix();
//    glTranslatef(0.0, -2.0, 0.0);
//
//    // Draw irregular shapes using cubes, cones, etc.
//    glutSolidCube(1.5);
//
//    // Draw cones to represent spikes or irregular formations
//    for (int i = 0; i < 360; i += 30) {
//        glPushMatrix();
//        glRotatef(static_cast<float>(i), 0.0, 1.0, 0.0);
//        glTranslatef(1.5, 0.0, 0.0);
//        glutSolidCone(0.5, 2.0, 50, 50);
//        glPopMatrix();
//    }
//
//    glPopMatrix();
//}


// These functions are called during initialization of the system, hence they need to be declared before the main function

void initializeSnowParticles() {
    // Initialize snowflakes with random positions and velocities
    for (int i = 0; i < 1000; ++i) {  // Increase the number of particles for a denser snowfall
        ParticleSnow snowflake;
        snowflake.x = static_cast<float>(rand()) / RAND_MAX * 200.0f - 100.0f;  // Random X position within the scene
        snowflake.y = static_cast<float>(rand()) / RAND_MAX * 50.0f + 50.0f;   // Random Y position within the scene
        snowflake.z = static_cast<float>(rand()) / RAND_MAX * 200.0f - 100.0f;  // Random Z position within the scene
        snowflake.velocityY = static_cast<float>(rand()) / RAND_MAX * 0.1f + 0.05f;  // Random vertical velocity
        particlesSnow.push_back(snowflake);
    }
}

// Modify the drawSnowParticles() function to skip drawing when the sun is hit
void updateSnowParticles() {
    for (auto& snowflake : particlesSnow) {
        if (!deleteEnemyZeppelin) {
            snowflake.y -= snowflake.velocityY*4;
            // Reset the snowflake to the top when it reaches the bottom
            if (snowflake.y < -50.0f) {
                snowflake.y = 50.0f;  // Reset to the top
                snowflake.x = static_cast<float>(rand()) / RAND_MAX * 200.0f - 100.0f;
                snowflake.z = static_cast<float>(rand()) / RAND_MAX * 200.0f - 100.0f;
            }
        }
    }
}

void drawSnowParticles() {
    if (!deleteEnemyZeppelin) {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, snow_texture);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
        float snowflakeSize = 0.5f;  // Adjust the size of each snowflake
        for (const auto& snowflake : particlesSnow) {
            glPushMatrix();
            glTranslatef(snowflake.x, snowflake.y, snowflake.z);
            GLUquadric* quadric = gluNewQuadric();
            gluQuadricTexture(quadric, GL_TRUE);
            gluSphere(quadric, snowflakeSize, 8, 8);  // Adjust the resolution as needed
            gluDeleteQuadric(quadric);
            glPopMatrix();
        }
        glDisable(GL_TEXTURE_2D);
    }
}

// creates the effect of snowing
void drawSnow(){
    glPushMatrix();
    glTranslatef(0, 0, -500);  // Adjust the initial position in the scene
    drawSnowParticles();
    glPopMatrix();
}

void drawCircle(float radius, int segments) {
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < segments; ++i) {
        float theta = 2.0f * 3.1415926f * static_cast<float>(i) / static_cast<float>(segments);
        float x = radius * std::cos(theta);
        float z = radius * std::sin(theta);
        glVertex3f(x, 0.0f, z);
    }
    glEnd();
}

int main(int argc, char **argv)
{
    // Initialize GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(vWidth, vHeight);
    glutInitWindowPosition(200, 30);
    glutCreateWindow("The Battle of Zeppelins!");

    // Initialize GL
    initOpenGL(vWidth, vHeight);
    // Initialize missiles
    initMissilesPlayer();
    initMissilesEnemy();
    // Set up music asynchronously
    std::thread musicThread(playMusicAsync, "/Users/arnoldcobo/Desktop/IEEQB/CPS-511 Computer Graphics/Assignment 3/Zeppelin Battle Simulation/Zeppelin Battle Simulation/Worakls - By The Brook.wav");
    
    glEnable(GL_DEPTH_TEST);
//    generateHeightmap();
        
    initializeSnowParticles();
    // Register callback functions
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(functionKeys);
    glutIdleFunc(idle);

    // Start event loop, never returns
    glutMainLoop();

    // Join the music thread to avoid termination before music finishes
    musicThread.join();
    return 0;
}

// Set up OpenGL. For viewport and projection setup see reshape().
void initOpenGL(int w, int h)
{
    // Set up and enable lighting
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);

    glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
    glLightfv(GL_LIGHT1, GL_POSITION, light_position1);
    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);   // This second light is currently off

    // Other OpenGL setup
    glEnable(GL_DEPTH_TEST);   // Remove hidded surfaces
    glShadeModel(GL_SMOOTH);   // Use smooth shading, makes boundaries between polygons harder to see
    glClearColor(0.4F, 0.4F, 0.4F, 0.0F);  // Color and depth for glClear
    glClearDepth(1.0f);
    glEnable(GL_NORMALIZE);    // Renormalize normal vectors
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);   // Nicer perspective

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Other initializatuion
    // Set up ground quad mesh
    VECTOR3D origin = VECTOR3D(-800.0f, -100.0f, 500.0f);
    VECTOR3D dir1v = VECTOR3D(1.0f, 0.0f, 0.0f);
    VECTOR3D dir2v = VECTOR3D(0.0f, 0.0f, -1.0f);
    groundMesh = new QuadMesh(meshSize, 32.0);
    groundMesh->InitMesh(meshSize, origin, 1400.0, 2400.0, dir1v, dir2v);

    VECTOR3D ambient = VECTOR3D(0.8f, 0.8f, 1.0f);    // Lighter blue ambient light
    VECTOR3D diffuse = VECTOR3D(0.8f, 0.8f, 1.0f);    // Lighter blue diffuse color
    VECTOR3D specular = VECTOR3D(0.0f, 0.0f, 0.0f);    // No specular highlight
    float shininess = 4.2;

    groundMesh->SetMaterial(ambient, diffuse, specular, shininess);
    
    // Load the texture
    body_texture1 = loadTexture("/Users/arnoldcobo/Library/Developer/Xcode/DerivedData/Zeppelin_Battle_Simulation-fleirgtdhybsbihcxczdgdnvmvpk/Build/Products/Debug/al_al.bmp");
    body_texture2 = loadTexture("/Users/arnoldcobo/Library/Developer/Xcode/DerivedData/Zeppelin_Battle_Simulation-fleirgtdhybsbihcxczdgdnvmvpk/Build/Products/Debug/dragon2.jpeg");
    body_texture3 = loadTexture("/Users/arnoldcobo/Library/Developer/Xcode/DerivedData/Zeppelin_Battle_Simulation-fleirgtdhybsbihcxczdgdnvmvpk/Build/Products/Debug/pirates.bmp");
    body_texture4 = loadTexture("/Users/arnoldcobo/Library/Developer/Xcode/DerivedData/Zeppelin_Battle_Simulation-fleirgtdhybsbihcxczdgdnvmvpk/Build/Products/Debug/tiles01.bmp");
    smokeEffect = loadTexture("/Users/arnoldcobo/Library/Developer/Xcode/DerivedData/Zeppelin_Battle_Simulation-fleirgtdhybsbihcxczdgdnvmvpk/Build/Products/Debug/smoke.bmp");
    fireEffect = loadTexture("/Users/arnoldcobo/Library/Developer/Xcode/DerivedData/Zeppelin_Battle_Simulation-fleirgtdhybsbihcxczdgdnvmvpk/Build/Products/Debug/fire.bmp");
    explosionEffect = loadTexture("/Users/arnoldcobo/Library/Developer/Xcode/DerivedData/Zeppelin_Battle_Simulation-fleirgtdhybsbihcxczdgdnvmvpk/Build/Products/Debug/explosion.bmp");
    
    groundTexture = loadTexture("/Users/arnoldcobo/Library/Developer/Xcode/DerivedData/Zeppelin_Battle_Simulation-fleirgtdhybsbihcxczdgdnvmvpk/Build/Products/Debug/ocean.bmp");
    
     shark_texture = loadTexture("/Users/arnoldcobo/Library/Developer/Xcode/DerivedData/Zeppelin_Battle_Simulation-fleirgtdhybsbihcxczdgdnvmvpk/Build/Products/Debug/pirates.bmp");
    
    column_texture = loadTexture("/Users/arnoldcobo/Library/Developer/Xcode/DerivedData/Zeppelin_Battle_Simulation-fleirgtdhybsbihcxczdgdnvmvpk/Build/Products/Debug/final_pilar.jpg");
    
    fire_texture = loadTexture("/Users/arnoldcobo/Library/Developer/Xcode/DerivedData/Zeppelin_Battle_Simulation-fleirgtdhybsbihcxczdgdnvmvpk/Build/Products/Debug/sun.bmp");
    
    moon_texture = loadTexture("/Users/arnoldcobo/Library/Developer/Xcode/DerivedData/Zeppelin_Battle_Simulation-fleirgtdhybsbihcxczdgdnvmvpk/Build/Products/Debug/moon.jpg");
    
    snow_texture = loadTexture("/Users/arnoldcobo/Library/Developer/Xcode/DerivedData/Zeppelin_Battle_Simulation-fleirgtdhybsbihcxczdgdnvmvpk/Build/Products/Debug/snow.jpg");
    
    MissileEnemyTexture = loadTexture("/Users/arnoldcobo/Library/Developer/Xcode/DerivedData/Zeppelin_Battle_Simulation-fleirgtdhybsbihcxczdgdnvmvpk/Build/Products/Debug/ice_missiles.jpg");

    // Set the texture ID for the ground mesh
    groundMesh->SetTextureID(groundTexture);
    // Set up the idle callback
       glutIdleFunc(idle);

}
// Functions dealing with creation, update and drawing of fire particles
void createParticlesFire(float explosionX, float explosionY, float explosionZ) {
    for (int i = 0; i < 100; ++i) {
        ParticleFire particle;
        particle.x = explosionX;
        particle.y = explosionY;
        particle.z = explosionZ;
        particle.speedX = (rand() % 100 - 50) / 100.0f;
        particle.speedY = (rand() % 100 - 50) / 100.0f;
        particle.speedZ = (rand() % 100 - 50) / 100.0f;
        particle.lifespan = 1.0f;
        particlesFire.push_back(particle);
    }
}
void updateParticlesFire(float deltaTime) {
    for (auto& particle : particlesFire) {
        particle.x += particle.speedX * deltaTime;
        particle.y += particle.speedY * deltaTime;
        particle.z += particle.speedZ * deltaTime;
        particle.lifespan -= 100.5f * deltaTime;
    }
}
void drawParticlesFire() {
    for (auto& particle : particlesFire) {
        // Enable texture mapping and set texture parameters
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, fire_texture);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); // Use MODULATE mode
        glColor3f(1.0f, particle.lifespan, 0.0f);
        
        glPushMatrix();
        glTranslatef(particle.x, particle.y, particle.z);
        GLUquadric* quadric = gluNewQuadric();
        gluQuadricTexture(quadric, GL_TRUE);
        gluSphere(quadric, 0.4, 9, 9);  // Adjust the radius and slices/stacks as needed
        glDisable(GL_TEXTURE_2D); // Disable texture mapping
        glPopMatrix();
        glDisable(GL_TEXTURE_2D); // Disable texture mapping

    }
}

// Callback, called whenever GLUT determines that the window should be redisplayed
// or glutPostRedisplay() has been called.
void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();
    // Create Viewing Matrix V
    // Set up the camera at position (0, 6, 37) looking at the origin, up along positive y axis
    //    gluLookAt(0.0, 6.0, 37.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    // check if the user is uing world camera or first person view camera
    if (fpvMode) {
            setupFPVCamera(objectPositionPlayer[0], objectPositionPlayer[1], objectPositionPlayer[2]);
        } else {
            setupWorldCamera();
        }
    
    glPushMatrix();
    zeppelin1.drawMissileEnemy();
    zeppelin2.drawMissileEnemy();
    zeppelin3.drawMissileEnemy();
    glPopMatrix();

    glPushMatrix();
    updateMissilesPlayer();
    updateMissilesEnemy();
    glPopMatrix();

    glPushMatrix();
    updateParticlesFire(1.5);
    drawParticlesFire();
    glPopMatrix();

    glPushMatrix();
    updateSnowParticles();
    drawSnowParticles();
    glPopMatrix();
    
    glPushMatrix();
    glTranslatef(10, 0, -50);
    glRotatef(90, 1, 0, 0);
    drawCircle(10, 20);
    glPopMatrix();

    // Draw Enemy Zeppelins
    // Apply modelling transformations M to move robot
    // Current transformation matrix is set to IV, where I is identity matrix
    // CTM = IV
    
    glPushMatrix();
    zeppelin3.drawZeppelinEnemy(body_texture4, smokeEffect, fireEffect, explosionEffect);
    glPopMatrix();
    
   
    glPushMatrix();
        zeppelin2.drawZeppelinEnemy(shark_texture, smokeEffect, fireEffect, explosionEffect);
    glPopMatrix();
    
    glPushMatrix();
    zeppelin1.drawZeppelinEnemy(body_texture2, smokeEffect, fireEffect, explosionEffect);
    glPopMatrix();
    
    // draw player Zeppelin
    glPushMatrix();
      // Move Zeppelin to the new position after the moving forward vector has been multiplicated with Rotation matrix and added the moving distance.
      // glTranslatef(objectPosition[0], objectPosition[1], objectPosition[2]);
    
        /* Managing 4x4 model transformation matrices.*/
        GLfloat translate2Matrix[16] = {
            1.0, 0.0f,0.0, 0.0,  // Column 1
            0.0f, 1.0f, 0.0f, 0.0,  // Column 2
            0.0, 0.0f, 1.0, 0.0,  // Column 3
            objectPositionPlayer[0], objectPositionPlayer[1], objectPositionPlayer[2], 1.0f  // Column 4
        };
    
        glMultMatrixf(translate2Matrix);
    
        
        glTranslatef(x,y,z);
        // Rotate Zeppelin left or right.
        glRotatef(zeppelinUserAngle, 0.0, 1.0, 0.0);
    
        //Draw Zeppelin (First layer of Hierarchy)
        //(First layer of Hierarchy)
        
        drawZeppelin(body_texture1, objectposition1);
    glPopMatrix();
    
    glPushMatrix();
        drawMissilePlayer();
    glPopMatrix();

    double previousTime =0;

    // Draw ground
    glPushMatrix();
        glTranslatef(0.0, -20.0, 0.0);
        // Calculate elapsed time
        double currentTime = glutGet(GLUT_ELAPSED_TIME) / 1000.0;  // Convert milliseconds to seconds
        double elapsedTime = currentTime - previousTime;
        previousTime = currentTime;

        // Update the mesh with the elapsed time
        groundMesh->UpdateMesh(elapsedTime*1.5);

        // Render the updated mesh
        groundMesh->DrawMesh(meshSize);
    glPopMatrix();
    
        // draw moon
        glPushMatrix();
        glScalef(3.5, 3.5, 3.5);
        glTranslatef(-35, 2, -20);
        drawMoon();
        glPopMatrix();
    
        // draw fire sphere
        glPushMatrix();
        glScalef(8.5, 8.5, 8.5);
        glTranslatef(0, 0, -20);
        drawSun();
        glPopMatrix();
    
        //draw icebergs
//        glPushMatrix();
//        glTranslatef(30, -15, -90);
//        drawTerrain();
//        glPopMatrix();
//    
//        glPushMatrix();
//        glRotatef(20, 0, 0, 1);
//        glTranslatef(-110, -15, -70);
//        drawTerrain();
//        glPopMatrix();
//        
//        glPushMatrix();
//        glScalef(6, 2, 1.5);
//        glTranslatef(-40, -10, -200);
//        drawTerrain();
//        glPopMatrix();
//    
//        glPushMatrix();
//        glTranslatef(-40, -20, -90);
//        drawIceberg();
//        glPopMatrix();
        
        // draw ancient columns
        glPushMatrix();
        glTranslatef(0, 0, -200);
        drawColumns();
        glPopMatrix();
        
//        // draw iceberg
//        glPushMatrix();
//
//        drawBody(iceberg_texture);
//        glPopMatrix();
    
    
        glutSwapBuffers();   // Double buffering, swap buffers
        glutPostRedisplay(); // Trigger a redraw   // Double buffering, swap buffers
}

// draw player Zeppelin
void drawZeppelin(GLuint texture, float objectPosition[3])
{
    
    glPushMatrix();
    // Adjust the texture mapping image
     glPushMatrix();
    // Apply the texture movement
    glMatrixMode(GL_TEXTURE);
    glLoadIdentity();
    glTranslatef(0.0, 1.8, 0.0);  // Translate the texture along the y-axis
        
    // Switch back to the modelview matrix
    glMatrixMode(GL_MODELVIEW);
    //Second level of hierarchy
    drawBody(texture);
    glPopMatrix();

    drawCabin();
    drawFins();
    drawPropellorRear();
    drawPropellorLeft();
    drawPropellorRight();
    glPopMatrix();
}

// draw zeppelin body with effects when hit by missile
void drawBodyWithEffects(GLuint bodyTexture, GLuint smokeTexture, GLuint fireTexture, GLuint explosionTexture){
    if (effectInProgress){
        
        // Calculate elapsed time since effect started
        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsedTime = currentTime - effectStartTime;
        
        // Check which effect to display based on elapsed time
                if (elapsedTime.count() < 0.6f) {
                    currentEffectTexture = smokeTexture;
                } else if (elapsedTime.count() < 0.8f) {
                    currentEffectTexture = explosionTexture;
                    drawParticlesFire(); // Draw particles during the smoke effect
                } else if (elapsedTime.count() < 1.5f) {
                    currentEffectTexture = explosionTexture;
                } else {
                    // Reset after the explosion effect
                    effectInProgress = false;
                    particlesFire.clear(); // Clear particles when the effect is done
                }

                // Draw the current effect
                drawBody(currentEffectTexture);
                
    }else{
        drawBody(bodyTexture);
    }
    
}

// Draw body in normal mode
void drawBody(GLuint texture)
{
    glMaterialfv(GL_FRONT, GL_AMBIENT, zeppelinBody_mat_ambient);
    glMaterialfv(GL_FRONT, GL_SPECULAR, zeppelinBody_mat_specular);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, zeppelinBody_mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SHININESS, zeppelinBody_mat_shininess);
    
    // Enable texture mapping and set texture parameters
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); // Use MODULATE mode

    glPushMatrix();
        glScalef(zeppelinBodyWidth, zeppelinBodyLength, zeppelinBodyDepth); // This will be done last
        GLUquadric *quadric = gluNewQuadric();
        gluQuadricTexture(quadric, GL_TRUE); // Enable texture coordinates

        gluSphere(quadric, 1.0, 50, 50); // gluSphere instead of glutSolidSphere
        gluDeleteQuadric(quadric);
    glPopMatrix();
    glDisable(GL_TEXTURE_2D); // Disable texture mapping

}

void drawCabin()
{
    glMaterialfv(GL_FRONT, GL_AMBIENT, zeppelinCabin_mat_ambient);
    glMaterialfv(GL_FRONT, GL_SPECULAR, zeppelinCabin_mat_specular);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, zeppelinCabin_mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SHININESS, zeppelinCabin_mat_shininess);

    glPushMatrix();
        glTranslatef(0, 0.5*zeppelinBodyLength-2*cabinWidth, 0); // This will be done last
        glScalef(cabinWidth, cabinLength, cabinDepth);
        glutSolidCube(1.0);
    glPopMatrix();
}

void drawFins()
{
    glPushMatrix();
    drawUpFin();
    drawBottomFin();
    drawLeftFin();
    drawRightFin();
    glPopMatrix();
}

void drawUpFin()
{
    // Load the texture
    UpperFin_texture = loadTexture("/Users/arnoldcobo/Library/Developer/Xcode/DerivedData/Zeppelin_Battle_Simulation-fleirgtdhybsbihcxczdgdnvmvpk/Build/Products/Debug/Fin.bmp");
    glMaterialfv(GL_FRONT, GL_AMBIENT, zeppelinCabin_mat_ambient);
    glMaterialfv(GL_FRONT, GL_SPECULAR, zeppelinCabin_mat_specular);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, zeppelinCabin_mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SHININESS, zeppelinCabin_mat_shininess);
    
    glPushMatrix();
        // Convert angle in Radians
        float angleInRadians = 75.0 * M_PI / 180.0f;

        glPushMatrix();

        //    Commented part using gl transformation functions
        //    glTranslatef(12, 14+(0.5*zeppelinBodyLength-2*cabinWidth), 0); // this will be done last
        //    glScalef(cabinWidth/2, cabinLength/2, 0.1);
        //    glRotatef(45.0,0.0, 0.0, 1.0 );

        /* Managing 4x4 model transformation matrices.*/

        //    Translate matrix
            GLfloat translate1Matrix[16] = {
                1.0, 0.0f, 0.0, 0.0,  // Column 1
                0.0f, 1.0f, 0.0f, 0.0,  // Column 2
                0.0, 0.0f, 1.0, 0.0,  // Column 3
                12.0, ((6.0)), 0.0f, 1.0f   // Column 4
            };
        //    Scaling matrix
            GLfloat Scale1Matrix[16] = {
                3.0f, 0.0f, 0.0f, 0.0f,  // Column 1
                0.0f, 1.6f, 0.0f, 0.0f,  // Column 2
                0.0f, 0.0f, 0.1f, 0.0f,  // Column 3
                0.0f, 0.0f, 0.0f, 1.0f   // Column 4
            };

        //    Rotation matrix
            GLfloat Rotate1Matrix[16] = {
                cos(angleInRadians), sin(angleInRadians) , 0.0f, 0.0f,  // Column 1
                -sin(angleInRadians), cos(angleInRadians), 0.0f, 0.0f,  // Column 2
                0.0f, 0.0f, 1.0, 0.0f,  // Column 3
                0.0f, 0.0f, 0.0f, 1.0f   // Column 4
            };
            
            
            // Enable texture mapping and set texture parameters
            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, UpperFin_texture);
            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); // Use MODULATE mode
            glPushMatrix();
                //  Multiplication of transformation matrices to transform the upper fin , rotation, scaling and translation.
                glMultMatrixf(translate1Matrix);
                glMultMatrixf(Scale1Matrix);
                glMultMatrixf(Rotate1Matrix);
        
                readOBJ("UpperFin.obj");
                drawTris3(positions3,normals3, indices3, numIndices);
//                glDisable(GL_TEXTURE_2D); // Disable texture mapping
            glPopMatrix();
    glPopMatrix();
}

void drawBottomFin()
{
    // Load the texture
    BottomFin_texture = loadTexture("/Users/arnoldcobo/Library/Developer/Xcode/DerivedData/Zeppelin_Battle_Simulation-fleirgtdhybsbihcxczdgdnvmvpk/Build/Products/Debug/skin01.bmp");
    
    glMaterialfv(GL_FRONT, GL_AMBIENT, zeppelinCabin_mat_ambient);
    glMaterialfv(GL_FRONT, GL_SPECULAR, zeppelinCabin_mat_specular);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, zeppelinCabin_mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SHININESS, zeppelinCabin_mat_shininess);
    
    // Enable texture mapping and set texture parameters
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, BottomFin_texture);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); // Use MODULATE mode
    glPushMatrix();
        glTranslatef(12, 2+(0.5*zeppelinBodyLength-2*cabinWidth), 0); // this will be done last
        glScalef(cabinWidth/2, cabinLength/2, 0.1);
        glRotatef(5.0,0.0, 0.0, 1.0 );
        readOBJ("BottomFin.obj");
        drawTris3(positions3,normals3, indices3, numIndices);
        glDisable(GL_TEXTURE_2D); // Disable texture mapping
    glPopMatrix();
}

void drawLeftFin()
{
    // Load the texture
    LeftFin_texture = loadTexture("/Users/arnoldcobo/Library/Developer/Xcode/DerivedData/Zeppelin_Battle_Simulation-fleirgtdhybsbihcxczdgdnvmvpk/Build/Products/Debug/skin01.bmp");
    glMaterialfv(GL_FRONT, GL_AMBIENT, zeppelinCabin_mat_ambient);
    glMaterialfv(GL_FRONT, GL_SPECULAR, zeppelinCabin_mat_specular);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, zeppelinCabin_mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SHININESS, zeppelinCabin_mat_shininess);
    // Enable texture mapping and set texture parameters
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, LeftFin_texture);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); // Use MODULATE mode
    
    glPushMatrix();
        glTranslatef(6, 7+(0.5*zeppelinBodyLength-2*cabinWidth), -5.5); // this will be done last
        glRotatef(40.0, 1.0, 0.0, 0.0 );
        glScalef(cabinWidth/2, cabinLength/2, 0.1);
        glRotatef(65.0,0.0, 0.0, 1.0 );
        readOBJ("BottomFin.obj");
        drawTris3(positions3,normals3, indices3, numIndices);
        glDisable(GL_TEXTURE_2D); // Disable texture mapping
    glPopMatrix();
}

void drawRightFin()
{
    // Load the texture
    RightFin_texture = loadTexture("/Users/arnoldcobo/Library/Developer/Xcode/DerivedData/Zeppelin_Battle_Simulation-fleirgtdhybsbihcxczdgdnvmvpk/Build/Products/Debug/skin01.bmp");
    glMaterialfv(GL_FRONT, GL_AMBIENT, zeppelinCabin_mat_ambient);
    glMaterialfv(GL_FRONT, GL_SPECULAR, zeppelinCabin_mat_specular);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, zeppelinCabin_mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SHININESS, zeppelinCabin_mat_shininess);
    
    // Enable texture mapping and set texture parameters
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, RightFin_texture);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); // Use MODULATE mode
    
    glPushMatrix();
        glTranslatef(6, 7+(0.5*zeppelinBodyLength-2*cabinWidth), 5.5); // this will be done last
        glRotatef(-40.0, 1.0, 0.0, 0.0 );
        glScalef(cabinWidth/2, cabinLength/2, 0.1);
        glRotatef(65.0,0.0, 0.0, 1.0 );
        readOBJ("BottomFin.obj");
        drawTris3(positions3,normals3, indices3, numIndices);
        glDisable(GL_TEXTURE_2D); // Disable texture mapping
    glPopMatrix();
}

// Second level of hierarchy
void drawPropellorRear()
{
    glMaterialfv(GL_FRONT, GL_AMBIENT, zeppelinPropellor_mat_ambient);
    glMaterialfv(GL_FRONT, GL_SPECULAR, zeppelinPropellor_mat_specular);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, zeppelinPropellor_mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SHININESS, zeppelinPropellor_mat_shininess);
    
    glPushMatrix();
    
        glScalef(0.7, 0.7, 0.7);
        glTranslatef(25.0, 0.0, 0.0); // this will be done last
        glRotatef(-90.0, 0.0, 1.0, 0.0);
        // stanchion
        glPushMatrix();
        glMaterialfv(GL_FRONT, GL_AMBIENT, zeppelinStanchion_mat_ambient);
        glMaterialfv(GL_FRONT, GL_SPECULAR, zeppelinStanchion_mat_specular);
        glMaterialfv(GL_FRONT, GL_DIFFUSE, zeppelinStanchion_mat_diffuse);
        glMaterialfv(GL_FRONT, GL_SHININESS, zeppelinStanchion_mat_shininess);
            glTranslatef(0, 0, 5.5); // this will be done last
            glRotatef(-90.0, 1.0, 0.0, 0.0);
            glScalef(stanchionRadius, stanchionLength/1.5, stanchionRadius);
            glRotatef(-90.0, 1.0, 0.0, 0.0);
            gluCylinder(gluNewQuadric(), 1.0, 1.0, 1.0, 20, 10);
        glPopMatrix();
        glPushMatrix();
            drawBlades();
    
            glPushMatrix();
                glutSolidCone(1.0, 1.0, 8.0, 10.0);
                glMaterialfv(GL_FRONT, GL_AMBIENT, zeppelinPropellor_mat_ambient);
                glMaterialfv(GL_FRONT, GL_SPECULAR, zeppelinPropellor_mat_specular);
                glMaterialfv(GL_FRONT, GL_DIFFUSE, zeppelinPropellor_mat_diffuse);
                glMaterialfv(GL_FRONT, GL_SHININESS, zeppelinPropellor_mat_shininess);
                glutSolidTorus(1.0, 5.0, 11.0, 8.0);
            glPopMatrix();
        glPopMatrix();
    
    glPopMatrix();
}

// Second level of hierarchy
void drawPropellorLeft()
{
    glMaterialfv(GL_FRONT, GL_AMBIENT, zeppelinPropellor_mat_ambient);
    glMaterialfv(GL_FRONT, GL_SPECULAR, zeppelinPropellor_mat_specular);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, zeppelinPropellor_mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SHININESS, zeppelinPropellor_mat_shininess);
    
    glPushMatrix();
        glTranslatef(0.2, 0.0, 8.0);
    
        glPushMatrix();
            glTranslatef(-0.45, 3.42, -0.52);
            glScalef(0.3, 0.3, 0.3);
            //Third level of hierarchy
            drawStanchion1(); //attached to Zeppelin
        glPopMatrix();
        glRotatef(angleleftpropeller, 0.0, 0.0, 1.0);
    
        glPushMatrix();
            glTranslatef(1.2, 0.0, 0.0);
            glScalef(0.3, 0.3, 0.3);
            glRotatef(90.0, 0.0, 1.0, 0.0);

            glPushMatrix();
                glPushMatrix();
                    glTranslatef(-1.2, +11.3, 0.0);
                    drawStanchion2(); //attached to Propeller
                glPopMatrix();
                //Fourth level of hierarchy
            drawBlades();
            glutSolidCone(1.0, 1.0, 8.0, 10.0);
            glMaterialfv(GL_FRONT, GL_AMBIENT, zeppelinPropellor_mat_ambient);
            glMaterialfv(GL_FRONT, GL_SPECULAR, zeppelinPropellor_mat_specular);
            glMaterialfv(GL_FRONT, GL_DIFFUSE, zeppelinPropellor_mat_diffuse);
            glMaterialfv(GL_FRONT, GL_SHININESS, zeppelinPropellor_mat_shininess);
            glutSolidTorus(1.0, 5.0, 11.0, 8.0);
            glPopMatrix();
        glPopMatrix();
    
    glPopMatrix();

}
// Second level of hierarchy
void drawPropellorRight()
{
    glMaterialfv(GL_FRONT, GL_AMBIENT, zeppelinPropellor_mat_ambient);
    glMaterialfv(GL_FRONT, GL_SPECULAR, zeppelinPropellor_mat_specular);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, zeppelinPropellor_mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SHININESS, zeppelinPropellor_mat_shininess);
    

    glPushMatrix();
        glTranslatef(0.2, 0.0, -4.5);

        glPushMatrix();
            glTranslatef(-0.45, 3.42, -0.52);
            glScalef(0.3, 0.3, 0.3);
            drawStanchion1(); //attached to Zeppelin
        glPopMatrix();
        glRotatef(angleleftpropeller, 0.0, 0.0, 1.0);
    
        glPushMatrix();
            glTranslatef(1.15, 0.0, -3.45);
            glScalef(0.3, 0.3, 0.3);
            glRotatef(90.0, 0.0, 1.0, 0.0);
        
            glPushMatrix();
                    glPushMatrix();
                        glTranslatef(-1.2, +11.3, 0.0);
                        //Third level of hierarchy
                        drawStanchion2(); //attached to Propeller
                    glPopMatrix();
                    //Fourth level of hierarchy
                    drawBlades();
                    glutSolidCone(1.0, 1.0, 8.0, 10.0);
                    glMaterialfv(GL_FRONT, GL_AMBIENT, zeppelinPropellor_mat_ambient);
                    glMaterialfv(GL_FRONT, GL_SPECULAR, zeppelinPropellor_mat_specular);
                    glMaterialfv(GL_FRONT, GL_DIFFUSE, zeppelinPropellor_mat_diffuse);
                    glMaterialfv(GL_FRONT, GL_SHININESS, zeppelinPropellor_mat_shininess);
                    glutSolidTorus(1.0, 5.0, 11.0, 8.0);
            glPopMatrix();
        glPopMatrix();
    glPopMatrix();
}

//connects Zeppelin with stanchion2
void drawStanchion1()
{
    glMaterialfv(GL_FRONT, GL_AMBIENT, zeppelinStanchion_mat_ambient);
    glMaterialfv(GL_FRONT, GL_SPECULAR, zeppelinStanchion_mat_specular);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, zeppelinStanchion_mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SHININESS, zeppelinStanchion_mat_shininess);
    // stanchion1
    glPushMatrix();
        glScalef(0.6, 0.6, 0.6);
        glTranslatef(2.0, -19.00, 3.5); // this will be done last
        glRotatef(-90.0, 1.0, 0.0, 0.0);
        glScalef(stanchionRadius, 2.5*stanchionLength, stanchionRadius);
        glRotatef(-90.0, 1.0, 0.0, 0.0);
        gluCylinder(gluNewQuadric(), 1.0, 1.0, 1.0, 20, 10);
    glPopMatrix();
}

//connects propellor with stanchion1
void drawStanchion2()
{
    glMaterialfv(GL_FRONT, GL_AMBIENT, zeppelinStanchion_mat_ambient);
    glMaterialfv(GL_FRONT, GL_SPECULAR, zeppelinStanchion_mat_specular);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, zeppelinStanchion_mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SHININESS, zeppelinStanchion_mat_shininess);
    // stanchion2
    glPushMatrix();
        glScalef(0.6, 0.6, 0.6);
        glTranslatef(2.0, -19.00, 0.5); // this will be done last
        glRotatef(-90.0, 1.0, 0.0, 0.0);
        glScalef(stanchionRadius, stanchionLength, stanchionRadius);
        glRotatef(-90.0, 1.0, 0.0, 0.0);
        gluCylinder(gluNewQuadric(), 1.0, 1.0, 1.0, 20, 10);
    glPopMatrix();
}

void drawBlades()
{
    glMaterialfv(GL_FRONT, GL_AMBIENT, zeppelinCabin_mat_ambient);
    glMaterialfv(GL_FRONT, GL_SPECULAR, zeppelinCabin_mat_specular);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, zeppelinCabin_mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SHININESS, zeppelinCabin_mat_shininess);
    
    glRotatef(-theta, 0.0, 0.0, 1.0);
    // Create 8 blades with a for loop
    for (int j=0; j<8; j++)
    {
        glPushMatrix();
            glRotatef(45.0 * j, 0.0, 0.0, 1.0);
            glTranslatef(0.0,3.0,0.0);
            glScalef(1.0, 5.0, 0.1);
            glutSolidCube(1.0);
        glPopMatrix();
        
    }
}


void drawIceberg()
{
    // Load the texture
    iceberg_texture = loadTexture("/Users/arnoldcobo/Library/Developer/Xcode/DerivedData/Zeppelin_Battle_Simulation-fleirgtdhybsbihcxczdgdnvmvpk/Build/Products/Debug/iceberg.jpg");
    glMaterialfv(GL_FRONT, GL_AMBIENT, zeppelinCabin_mat_ambient);
    glMaterialfv(GL_FRONT, GL_SPECULAR, zeppelinCabin_mat_specular);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, zeppelinCabin_mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SHININESS, zeppelinCabin_mat_shininess);
    
    // Enable texture mapping and set texture parameters
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, iceberg_texture);
    // Requirement 1.b
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); // Use MODULATE mode
    
    glPushMatrix();
//        glTranslatef(6, 7+(0.5*zeppelinBodyLength-2*cabinWidth), 5.5); // this will be done last
//        glRotatef(-40.0, 1.0, 0.0, 0.0 );
//        glScalef(cabinWidth/2, cabinLength/2, 0.1);
//        glRotatef(65.0,0.0, 0.0, 1.0 );
        readOBJ("iceberg.obj");
        drawTris3(positions3,normals3, indices3, numIndices);
        glDisable(GL_TEXTURE_2D); // Disable texture mapping
    glPopMatrix();
}
void drawMoon() {
    // Enable texture mapping and set texture parameters
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, moon_texture);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); // Use MODULATE mode
    
//    glColor3f(1.0, 1.0, 0.5);  // Set color to yellow
    glTranslatef(0, 10, -10);
    GLUquadric* quadric = gluNewQuadric();
    gluQuadricTexture(quadric, GL_TRUE);
    gluSphere(quadric, 1.0, 20, 20);  // Draw a sphere with radius 1.0
    gluDeleteQuadric(quadric);
    
    glDisable(GL_TEXTURE_2D); // Disable texture mapping
}
void drawSun() {
    
    glDisable(GL_LIGHTING);  // Disable lighting
    // Enable texture mapping and set texture parameters
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, fire_texture);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); // Use MODULATE mode
    
    if (deleteEnemyZeppelin) {
        // Check if it's time to delete the sun
        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsedTime = currentTime - deletionStartTime;
        if (elapsedTime.count() > 1.5f) {
            // Move the sun to a faraway location
            glTranslatef(1000, -1000, 1000);
        }
    }
    glTranslatef(sunPosition[0], sunPosition[1], sunPosition[2]);
   
//    glTranslatef(sunPosition[0], sunPosition[1], sunPosition[2]);
    GLUquadric* quadric = gluNewQuadric();
    gluQuadricTexture(quadric, GL_TRUE);
    gluSphere(quadric, 1.0, 20, 20);  // Draw a sphere with radius 1.0
    gluDeleteQuadric(quadric);
    
    glDisable(GL_TEXTURE_2D); // Disable texture mapping
    glEnable(GL_LIGHTING);  // Re-enable lighting
}

void drawColumns()
{
    const int numColumns = 15;
    const float circleRadius = 120.0f;

    for (int i = 0; i < numColumns; ++i) {
        float angle = static_cast<float>(i) * (2.0f * M_PI / numColumns);
        float xPos = circleRadius * cos(angle);
        float zPos = circleRadius * sin(angle);

        glPushMatrix();
        glTranslatef(xPos, 0.0, zPos);
        drawColumn(column_texture);
        glPopMatrix();
    }
}

void drawColumn(GLuint texture)
{
    glMaterialfv(GL_FRONT, GL_AMBIENT, zeppelinBody_mat_ambient);
    glMaterialfv(GL_FRONT, GL_SPECULAR, zeppelinBody_mat_specular);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, zeppelinBody_mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SHININESS, zeppelinBody_mat_shininess);

    // Enable texture mapping and set texture parameters
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); // Use MODULATE mode

    glScalef(1, 10, 1);
    glTranslatef(0, -2, -9);
    glRotatef(-90, 1, 0, 0);
    glPushMatrix();
    
    glScalef(4, 8, 4); // Scale the entire column
    GLUquadric *quadric = gluNewQuadric();
    gluQuadricTexture(quadric, GL_TRUE); // Enable texture coordinates

    // Draw a cylinder instead of a sphere
    gluCylinder(quadric, 1.0, 1.0, 5.0, 50, 50); // Parameters: base radius, top radius, height, slices, stacks

    gluDeleteQuadric(quadric);
    glPopMatrix();

    // Disable texture mapping after drawing the column
    glDisable(GL_TEXTURE_2D);
}

void drawMissilePlayer() {
    if (missileFiredPlayer) {
        // Draw all active missiles
        for (auto& missile : missilesPlayer) {
            if (missile.active) {
                
                glPushMatrix();
                glTranslatef(missile.positionPlayer[0], missile.positionPlayer[1], missile.positionPlayer[2]);

                // Apply rotation to align the missile with its initial direction
                float missileOrientation = atan2(missile.initialDirectionPlayer[2], -missile.initialDirectionPlayer[0]) * 180.0f / M_PI;
                glRotatef(missileOrientation, 0, 1, 0);
                
//                glDisable(GL_LIGHTING);  // Disable lighting
                // Enable texture mapping and set texture parameters
                glEnable(GL_TEXTURE_2D);
                glBindTexture(GL_TEXTURE_2D, fire_texture);
                glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); // Use MODULATE mode
                
                glScalef(10, 1.5, 1);
                // Use gluSphere to draw the missile
                GLUquadric* quadric = gluNewQuadric();
                gluQuadricTexture(quadric, GL_TRUE);
                gluSphere(quadric, 0.2, 10, 10); // Adjust the radius and resolution as needed
                gluDeleteQuadric(quadric);

                glPopMatrix();
            }
        }
    }
    glDisable(GL_TEXTURE_2D);
}

void fireMissilePlayer() {
    // Create a new missile and add it to the vector
    MissilePlayer newMissile;
    newMissile.active = true;
    missileFiredPlayer = false;

    if (!missileFiredPlayer) {
        
        // Set missile position to the Zeppelin's position (or adjust as needed)
        for (int i = 0; i < 3; i++) {
            newMissile.positionPlayer[i] = objectPositionPlayer[i];
        }
        // Set missile direction based on Zeppelin's orientation
        newMissile.initialDirectionPlayer[0] = -cos(zeppelinUserAngle * M_PI / 180.0f);
        newMissile.initialDirectionPlayer[2] = sin(zeppelinUserAngle * M_PI / 180.0f);
        newMissile.speedPlayer = 1.0f;
        missilesPlayer.push_back(newMissile);
        missileFiredPlayer = true;
        
    }
}

float zeppelinMinBounds[3];
float zeppelinMaxBounds[3];

void updateMissilesPlayer() {
    // Update the position of active missiles
    for (auto& missile : missilesPlayer) {
        if (missile.active) {
            // Update missile position based on its direction and speed
            for (int i = 0; i < 3; i++) {
                
                missile.positionPlayer[i] += missile.initialDirectionPlayer[i] * missile.speedPlayer;
                
                // handle collision when missile hits zeppelin Enemy
                handleCollisionEnemy(missile.positionPlayer, zeppelin1.position, zeppelinBodyWidth, zeppelinBodyLength, zeppelinBodyDepth, zeppelin1, missile);
                // Bounding box visualization for zeppelin Enemy1
                glColor3f(1.0f, 1.0f, 0.0f);  // Yellow color
                glBegin(GL_LINE_LOOP);
                    glVertex3f(zeppelinMinBounds[0], zeppelinMinBounds[1], zeppelinMinBounds[2]);
                    glVertex3f(zeppelinMaxBounds[0], zeppelinMinBounds[1], zeppelinMinBounds[2]);
                    glVertex3f(zeppelinMaxBounds[0], zeppelinMaxBounds[1], zeppelinMinBounds[2]);
                    glVertex3f(zeppelinMinBounds[0], zeppelinMaxBounds[1], zeppelinMinBounds[2]);
                glEnd();
                glBegin(GL_LINE_LOOP);
                    glVertex3f(zeppelinMinBounds[0], zeppelinMinBounds[1], zeppelinMaxBounds[2]);
                    glVertex3f(zeppelinMaxBounds[0], zeppelinMinBounds[1], zeppelinMaxBounds[2]);
                    glVertex3f(zeppelinMaxBounds[0], zeppelinMaxBounds[1], zeppelinMaxBounds[2]);
                    glVertex3f(zeppelinMinBounds[0], zeppelinMaxBounds[1], zeppelinMaxBounds[2]);
                glEnd();
                glBegin(GL_LINES);
                    glVertex3f(zeppelinMinBounds[0], zeppelinMinBounds[1], zeppelinMinBounds[2]);
                    glVertex3f(zeppelinMinBounds[0], zeppelinMinBounds[1], zeppelinMaxBounds[2]);
                    glVertex3f(zeppelinMaxBounds[0], zeppelinMinBounds[1], zeppelinMinBounds[2]);
                    glVertex3f(zeppelinMaxBounds[0], zeppelinMinBounds[1], zeppelinMaxBounds[2]);
                    glVertex3f(zeppelinMaxBounds[0], zeppelinMaxBounds[1], zeppelinMinBounds[2]);
                    glVertex3f(zeppelinMaxBounds[0], zeppelinMaxBounds[1], zeppelinMaxBounds[2]);
                    glVertex3f(zeppelinMinBounds[0], zeppelinMaxBounds[1], zeppelinMinBounds[2]);
                    glVertex3f(zeppelinMinBounds[0], zeppelinMaxBounds[1], zeppelinMaxBounds[2]);
                glEnd();
                
                handleCollisionEnemy(missile.positionPlayer, zeppelin2.position, zeppelinBodyWidth, zeppelinBodyLength, zeppelinBodyDepth, zeppelin2, missile);
                // Bounding box visualization for zeppelin Enemy2
                glColor3f(1.0f, 1.0f, 0.0f);  // Yellow color
                glBegin(GL_LINE_LOOP);
                    glVertex3f(zeppelinMinBounds[0], zeppelinMinBounds[1], zeppelinMinBounds[2]);
                    glVertex3f(zeppelinMaxBounds[0], zeppelinMinBounds[1], zeppelinMinBounds[2]);
                    glVertex3f(zeppelinMaxBounds[0], zeppelinMaxBounds[1], zeppelinMinBounds[2]);
                    glVertex3f(zeppelinMinBounds[0], zeppelinMaxBounds[1], zeppelinMinBounds[2]);
                glEnd();
                glBegin(GL_LINE_LOOP);
                    glVertex3f(zeppelinMinBounds[0], zeppelinMinBounds[1], zeppelinMaxBounds[2]);
                    glVertex3f(zeppelinMaxBounds[0], zeppelinMinBounds[1], zeppelinMaxBounds[2]);
                    glVertex3f(zeppelinMaxBounds[0], zeppelinMaxBounds[1], zeppelinMaxBounds[2]);
                    glVertex3f(zeppelinMinBounds[0], zeppelinMaxBounds[1], zeppelinMaxBounds[2]);
                glEnd();
                glBegin(GL_LINES);
                    glVertex3f(zeppelinMinBounds[0], zeppelinMinBounds[1], zeppelinMinBounds[2]);
                    glVertex3f(zeppelinMinBounds[0], zeppelinMinBounds[1], zeppelinMaxBounds[2]);
                    glVertex3f(zeppelinMaxBounds[0], zeppelinMinBounds[1], zeppelinMinBounds[2]);
                    glVertex3f(zeppelinMaxBounds[0], zeppelinMinBounds[1], zeppelinMaxBounds[2]);
                    glVertex3f(zeppelinMaxBounds[0], zeppelinMaxBounds[1], zeppelinMinBounds[2]);
                    glVertex3f(zeppelinMaxBounds[0], zeppelinMaxBounds[1], zeppelinMaxBounds[2]);
                    glVertex3f(zeppelinMinBounds[0], zeppelinMaxBounds[1], zeppelinMinBounds[2]);
                    glVertex3f(zeppelinMinBounds[0], zeppelinMaxBounds[1], zeppelinMaxBounds[2]);
                glEnd();
                
                handleCollisionEnemy(missile.positionPlayer, zeppelin3.position, zeppelinBodyWidth, zeppelinBodyLength, zeppelinBodyDepth, zeppelin3, missile);
                // Bounding box visualization for zeppelin Enemy3
                glColor3f(1.0f, 1.0f, 0.0f);  // Yellow color
                glBegin(GL_LINE_LOOP);
                    glVertex3f(zeppelinMinBounds[0], zeppelinMinBounds[1], zeppelinMinBounds[2]);
                    glVertex3f(zeppelinMaxBounds[0], zeppelinMinBounds[1], zeppelinMinBounds[2]);
                    glVertex3f(zeppelinMaxBounds[0], zeppelinMaxBounds[1], zeppelinMinBounds[2]);
                    glVertex3f(zeppelinMinBounds[0], zeppelinMaxBounds[1], zeppelinMinBounds[2]);
                glEnd();
                glBegin(GL_LINE_LOOP);
                    glVertex3f(zeppelinMinBounds[0], zeppelinMinBounds[1], zeppelinMaxBounds[2]);
                    glVertex3f(zeppelinMaxBounds[0], zeppelinMinBounds[1], zeppelinMaxBounds[2]);
                    glVertex3f(zeppelinMaxBounds[0], zeppelinMaxBounds[1], zeppelinMaxBounds[2]);
                    glVertex3f(zeppelinMinBounds[0], zeppelinMaxBounds[1], zeppelinMaxBounds[2]);
                glEnd();
                glBegin(GL_LINES);
                    glVertex3f(zeppelinMinBounds[0], zeppelinMinBounds[1], zeppelinMinBounds[2]);
                    glVertex3f(zeppelinMinBounds[0], zeppelinMinBounds[1], zeppelinMaxBounds[2]);
                    glVertex3f(zeppelinMaxBounds[0], zeppelinMinBounds[1], zeppelinMinBounds[2]);
                    glVertex3f(zeppelinMaxBounds[0], zeppelinMinBounds[1], zeppelinMaxBounds[2]);
                    glVertex3f(zeppelinMaxBounds[0], zeppelinMaxBounds[1], zeppelinMinBounds[2]);
                    glVertex3f(zeppelinMaxBounds[0], zeppelinMaxBounds[1], zeppelinMaxBounds[2]);
                    glVertex3f(zeppelinMinBounds[0], zeppelinMaxBounds[1], zeppelinMinBounds[2]);
                    glVertex3f(zeppelinMinBounds[0], zeppelinMaxBounds[1], zeppelinMaxBounds[2]);
                glEnd();
                
                handleCollisionSun(missile.positionPlayer, sunFinalPosition ,missile);
            }
            
        }
    }
}

// Function to check if a point is inside a bounding box
bool isInsideBoundingBox(float point[3], float minBounds[3], float maxBounds[3]) {
    return (
        point[0] >= minBounds[0] && point[0] <= maxBounds[0] &&
        point[1] >= minBounds[1] && point[1] <= maxBounds[1] &&
        point[2] >= minBounds[2] && point[2] <= maxBounds[2]
    );
}

void respawnZeppelinEnemy(EnemyZeppelin& zeppelin) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> distribution(-50.0f, 50.0f);  // Adjust the range as needed
    
    // Delay for 0.5 seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    if (deleteEnemyZeppelin){
        // Move the zeppelin to a new random position
        zeppelin.position[0] = 1000;
        zeppelin.position[1] = -1000;
        zeppelin.position[2] = 1000;
    }
    else{
        
        // Move the zeppelin to a new random position
        zeppelin.position[0] = zeppelin.position[0] + distribution(gen);
        zeppelin.position[1] = zeppelin.position[1] + distribution(gen);
        zeppelin.position[2] = zeppelin.position[2] + distribution(gen);
    }
    // Reset any other necessary zeppelin state
    zeppelin.collisionOccurred = false;
    effectInProgress=false;
}

// Function to handle missile-zeppelinEnemy collision
void handleCollisionEnemy(float missilePosition[3], float zeppelinPosition[3], float zeppelinBodyWidth, float zeppelinBodyLength, float zeppelinBodyDepth,  EnemyZeppelin& zeppelin, MissilePlayer& missilePlayer) {
    // Calculate zeppelin bounding box
    zeppelinMinBounds[0] = zeppelinPosition[0] - zeppelinBodyWidth / 2;
    zeppelinMinBounds[1] = zeppelinPosition[1] - zeppelinBodyLength / 2;
    zeppelinMinBounds[2] = zeppelinPosition[2] - zeppelinBodyDepth / 2;

    zeppelinMaxBounds[0] = zeppelinPosition[0] + zeppelinBodyWidth / 2;
    zeppelinMaxBounds[1] = zeppelinPosition[1] + zeppelinBodyLength / 2;
    zeppelinMaxBounds[2] = zeppelinPosition[2] + zeppelinBodyDepth / 2;

    // Check collision
    if (isInsideBoundingBox(missilePosition, zeppelinMinBounds, zeppelinMaxBounds)) {
        
        // Collision detected
        zeppelin.collisionOccurred = true;
        zeppelin.missileFiredEnemy = false;
        effectInProgress = true;
        effectStartTime = std::chrono::high_resolution_clock::now();
        
        // Set a variable to indicate that the zeppelin is in the process of disappearing
        zeppelin.disappearing = true;
        zeppelin.disappearanceStartTime = std::chrono::high_resolution_clock::now();
        
        missilePlayer.active = false;
        std::cout << "U got the enemy!!";
    
    }
    
    // If the zeppelin is disappearing, check the time elapsed to determine when to actually disappear it
    if (zeppelin.disappearing) {
        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsedTime = currentTime - zeppelin.disappearanceStartTime;
        // Check if 3 seconds have passed
        if (elapsedTime.count() > 1.5f) {
            // Reset the zeppelin state and move it to a new random position
            zeppelin.collisionOccurred = false;
            effectInProgress = false;
            zeppelin.disappearing = false;
            
            // Initialize random seed and create particles
            srand(time(nullptr));
            createParticlesFire(zeppelin.position[0],zeppelin.position[1],zeppelin.position[2]);
            respawnZeppelinEnemy(zeppelin);
        }
    }
}
// Draw the bounding box around the scaled and transformed sun
void drawBoundingBox(const float sunPosition[3]) {
    glDisable(GL_LIGHTING);
    glColor3f(1.0f, 0.0f, 0.0f); // Set color to red for visibility

    // Define the size of the bounding box
    float boundingBoxSize = 5.0f;

    // Calculate the coordinates of the bounding box
    float minX = sunPosition[0] - boundingBoxSize;
    float minY = sunPosition[1] - boundingBoxSize;
    float minZ = sunPosition[2] - boundingBoxSize;
    float maxX = sunPosition[0] + boundingBoxSize;
    float maxY = sunPosition[1] + boundingBoxSize;
    float maxZ = sunPosition[2] + boundingBoxSize;

    // Draw the bounding box in wireframe mode
    glBegin(GL_LINE_LOOP);
    glVertex3f(minX, minY, minZ);
    glVertex3f(maxX, minY, minZ);
    glVertex3f(maxX, maxY, minZ);
    glVertex3f(minX, maxY, minZ);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glVertex3f(minX, minY, maxZ);
    glVertex3f(maxX, minY, maxZ);
    glVertex3f(maxX, maxY, maxZ);
    glVertex3f(minX, maxY, maxZ);
    glEnd();

    glBegin(GL_LINES);
    glVertex3f(minX, minY, minZ);
    glVertex3f(minX, minY, maxZ);

    glVertex3f(maxX, minY, minZ);
    glVertex3f(maxX, minY, maxZ);

    glVertex3f(maxX, maxY, minZ);
    glVertex3f(maxX, maxY, maxZ);

    glVertex3f(minX, maxY, minZ);
    glVertex3f(minX, maxY, maxZ);
    glEnd();

    glEnable(GL_LIGHTING);
}

// Call this function when you want to start the deletion countdown
void startSunDeletionTimer() {
    deletionStartTime = std::chrono::high_resolution_clock::now();
}

// Handle collision detection with the sun's bounding box
void handleCollisionSun(const float missilePosition[3], const float sunPosition[3], MissilePlayer& missilePlayer) {
    // Define the size of the bounding box
    float boundingBoxSize = 5.0f;

    // Draw the bounding box for visualization (optional)
//    drawBoundingBox(sunPosition);
    
    // Check if the missile's position is within the bounding box around the sun
    if (missilePosition[0] >= sunPosition[0] - boundingBoxSize && missilePosition[0] <= sunPosition[0] + boundingBoxSize &&
        missilePosition[1] >= sunPosition[1] - boundingBoxSize && missilePosition[1] <= sunPosition[1] + boundingBoxSize &&
        missilePosition[2] >= sunPosition[2] - boundingBoxSize && missilePosition[2] <= sunPosition[2] + boundingBoxSize) {
        // Handle collision with the sun
        missilePlayer.active = false; // Deactivate the missile
        std::cout << "The missile hit the sun!" << std::endl;
        createParticlesFire(sunPosition[0], sunPosition[1], sunPosition[2]);
        startSunDeletionTimer();
        deleteEnemyZeppelin = true;
        destroyEnemyZeppelins(container);
        
    }
}


void updateMissilesEnemy() {
    // Update the position of active missiles
    for (auto& missile : missilesEnemy) {
        if (missile.active) {
            // Update missile position based on its direction and speed
            for (int i = 0; i < 3; i++) {
                
                missile.positionEnemy[i] += missile.initialDirectionEnemy[i] * missile.speedEnemy;
                
                // handle collision when missile hits zeppelin Player
                handleCollisionPlayer(missile.positionEnemy,objectPositionPlayer, zeppelinBodyWidth, zeppelinBodyLength, zeppelinBodyDepth, missile);
                
//                std::cout << "Missile Position: " << missile.positionEnemy[0] << ", " << missile.positionEnemy[1] << ", " << missile.positionEnemy[2] << std::endl;
//                std::cout << "Zeppelin Player Position: " << objectPositionPlayer[0] << ", " << objectPositionPlayer[1] << ", " << objectPositionPlayer[2] << std::endl;
//                std::cout << "Zeppelin Min Bounds: " << zeppelinMinBounds[0] << ", " << zeppelinMinBounds[1] << ", " << zeppelinMinBounds[2] << std::endl;
//                std::cout << "Zeppelin Max Bounds: " << zeppelinMaxBounds[0] << ", " << zeppelinMaxBounds[1] << ", " << zeppelinMaxBounds[2] << std::endl;
                
                // Bounding Box visualization for Zeppelin Player
                    glColor3f(1.0f, 1.0f, 0.0f);  // Red color
                    glBegin(GL_LINE_LOOP);
                        glVertex3f(zeppelinMinBounds[0], zeppelinMinBounds[1], zeppelinMinBounds[2]);
                        glVertex3f(zeppelinMaxBounds[0], zeppelinMinBounds[1], zeppelinMinBounds[2]);
                        glVertex3f(zeppelinMaxBounds[0], zeppelinMaxBounds[1], zeppelinMinBounds[2]);
                        glVertex3f(zeppelinMinBounds[0], zeppelinMaxBounds[1], zeppelinMinBounds[2]);
                    glEnd();
                    glBegin(GL_LINE_LOOP);
                        glVertex3f(zeppelinMinBounds[0], zeppelinMinBounds[1], zeppelinMaxBounds[2]);
                        glVertex3f(zeppelinMaxBounds[0], zeppelinMinBounds[1], zeppelinMaxBounds[2]);
                        glVertex3f(zeppelinMaxBounds[0], zeppelinMaxBounds[1], zeppelinMaxBounds[2]);
                        glVertex3f(zeppelinMinBounds[0], zeppelinMaxBounds[1], zeppelinMaxBounds[2]);
                    glEnd();
                    glBegin(GL_LINES);
                        glVertex3f(zeppelinMinBounds[0], zeppelinMinBounds[1], zeppelinMinBounds[2]);
                        glVertex3f(zeppelinMinBounds[0], zeppelinMinBounds[1], zeppelinMaxBounds[2]);
                        glVertex3f(zeppelinMaxBounds[0], zeppelinMinBounds[1], zeppelinMinBounds[2]);
                        glVertex3f(zeppelinMaxBounds[0], zeppelinMinBounds[1], zeppelinMaxBounds[2]);
                        glVertex3f(zeppelinMaxBounds[0], zeppelinMaxBounds[1], zeppelinMinBounds[2]);
                        glVertex3f(zeppelinMaxBounds[0], zeppelinMaxBounds[1], zeppelinMaxBounds[2]);
                        glVertex3f(zeppelinMinBounds[0], zeppelinMaxBounds[1], zeppelinMinBounds[2]);
                        glVertex3f(zeppelinMinBounds[0], zeppelinMaxBounds[1], zeppelinMaxBounds[2]);
                    glEnd();
            }
            
        }
    }
}

void respawnZeppelinPlayer() {
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> distribution(-20.0f, 20.0f);  // Adjust the range as needed
    
    // Delay for 0.5 seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
       
   // Move the zeppelin to a new random position within the viewport bounds
   objectPositionPlayer[0] = distribution(gen); // X position
   objectPositionPlayer[1] = distribution(gen); // Y position
   objectPositionPlayer[2] = 0.0f; // Assuming z-axis is at 0 for 2D game
   updateBoundingBox(objectPositionPlayer);
   initMissilesPlayer(); // Initialize missiles or any other necessary setup
}

// Function to handle missile-zeppelinPlayer collision
void handleCollisionPlayer(float missilePosition[3], float objectPositionPlayer[3], float zeppelinBodyWidth, float zeppelinBodyLength, float zeppelinBodyDepth, MissileEnemy& missileEnemy) {
    
    // Calculate zeppelin bounding box
    zeppelinMinBounds[0] = objectPositionPlayer[0] - zeppelinBodyWidth / 2;
    zeppelinMinBounds[1] = objectPositionPlayer[1] - zeppelinBodyLength / 2;
    zeppelinMinBounds[2] = objectPositionPlayer[2] - zeppelinBodyDepth / 2;

    zeppelinMaxBounds[0] = objectPositionPlayer[0] + zeppelinBodyWidth / 2;
    zeppelinMaxBounds[1] = objectPositionPlayer[1] + zeppelinBodyLength / 2;
    zeppelinMaxBounds[2] = objectPositionPlayer[2] + zeppelinBodyDepth / 2;
    // Check collision
    if (isInsideBoundingBox(missilePosition, zeppelinMinBounds, zeppelinMaxBounds)) {
        
        // Collision detected
        missileFiredPlayer = false;
        missileEnemy.active = false;
        createParticlesFire(objectPositionPlayer[0],objectPositionPlayer[1],objectPositionPlayer[2]);
        std::cout << "Come on Boys!!";
  
        respawnZeppelinPlayer();
        
    }
}

void updateBoundingBox(float objectPositionPlayer[3])
{
    const float boundingBoxHeightIncrease = 0.0f; // Adjust this value as needed
    
    zeppelinMinBounds[0] = (objectPositionPlayer[0] - zeppelinBodyWidth / 2);
    zeppelinMinBounds[1] = (objectPositionPlayer[1] - zeppelinBodyLength / 2)  ;
    zeppelinMinBounds[2] = (objectPositionPlayer[2] - zeppelinBodyDepth / 2);

    zeppelinMaxBounds[0] = (objectPositionPlayer[0] + zeppelinBodyWidth / 2);
    zeppelinMaxBounds[1] = (objectPositionPlayer[1] + zeppelinBodyLength / 2)  ;
    zeppelinMaxBounds[2] = (objectPositionPlayer[2] + zeppelinBodyDepth / 2);
    
//    std::cout << "Object Position: (" << objectPositionPlayer[0] << ", " << objectPositionPlayer[1] << ", " << objectPositionPlayer[2] << ")" << std::endl;
//    std::cout << "moveUpDown: " << moveUpDown << std::endl;
}

// Callback, called at initialization and whenever user resizes the window.
void reshape(int w, int h)
{
    // Set up viewport, projection, then change to modelview matrix mode -
    // display function will then set up camera and do modeling transforms.
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (GLdouble)w / h, 0.2, 440.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Set up the camera at position (0, 6, 22) looking at the origin, up along positive y axis
//    gluLookAt(0.0, 6.0, 22.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    
    // check if the user is uing world camera or first person view camera
    if (fpvMode) {
            setupFPVCamera(objectPositionPlayer[0], objectPositionPlayer[1], objectPositionPlayer[2]);
        } else {
            setupWorldCamera();
        }
}

// Callback, handles input from the keyboard, non-arrow keys
void keyboard(unsigned char key, int x, int y)
{
    switch (key)
    {
// Move Zeppelin backward and spin propeller blades at the same time
    case 's':
// Update object position to move it backward
        start = true;
        updateZeppelinDirection();
        updateZeppelinPlayerPositionb();
        glutTimerFunc(1000, spinDisplay, 0);
        glutTimerFunc(10, animationHandlerForward, 0);
        break;
            
// Move Zeppelin forward and spin propeller blades at the same time
    case 'w':
// Update object position to move it forward
    updateZeppelinDirection();
    updateZeppelinPlayerPositionf();
    glutTimerFunc(1000, spinDisplay, 0);
        
        start = true;
        upward = true;
        downward=true;
        forward = true;
            
        glutTimerFunc(10, animationHandlerForward, 0);
        break;
            
// Stop spinning propellor blades
    case 'W':
        spin=0.0;
        theta=0.0;
        start=false;
        break;
            
// Zoom in Zeppelin
    case 'p':
        start = true;
        x += 2.0;
        y+=2.0;
        z+=2.0;
        break;
            
// Zoom out Zeppelin
    case 'P':
        start = true;
        x-= 2.0;
        y-=2.0;
        z-=2.0;
        break;
          
// Move Zeppelin upwards and tilt side propellers up
    case 'i':
        glutTimerFunc(1000, spinDisplay, 0);
        moveUpDown += 1.0;
        objectPositionPlayer[1]+=1;
        updateBoundingBox(objectPositionPlayer);
        break;
         
// Move Zeppelin downwards and tilt side propellers down
    case 'k':
        glutTimerFunc(1000, spinDisplay, 0);
        moveUpDown -= 1.0;
        objectPositionPlayer[1]-=1;
        updateBoundingBox(objectPositionPlayer);
        break;
// Quit game, exit screen
    case 'q':
        exit(0);
        break;
// Simulation of first person view camera
    case 'f':
    case 'F':
// Toggle FPV mode when 'F' key is pressed
        fpvMode = !fpvMode;
        break;
// Press 'h' to fire the missile
    case 'h':
    case 'H':
        
    fireMissilePlayer();
    break;
                
        
    }

    glutPostRedisplay();   // Trigger a window redisplay
}

void spinDisplay(int)
{
  spin += 1.0;
  if (spin > 360.0)
    spin -= 360.0;

    theta += 6.0;
    if (theta > 360.0)
        theta -= 360.0;

  glutPostRedisplay();
    if (start == true)
        glutTimerFunc(1000, spinDisplay, 0);
   
}

// Side propellers will tilt up about up 90 degrees
void animationHandlerUp(int param)
{
    if (upward == true)
    {
        if (angleleftpropeller < 90.0){
            angleleftpropeller += 5.0;
            glutPostRedisplay();
            if (angleleftpropeller == 90.0)
                upward=false;
            glutTimerFunc(10, animationHandlerUp, 0);
        }
    }
}
// Side propellers will tilt down about up 90 degrees
void animationHandlerDown(int param)
{
    if (downward == true)
    {
        if (angleleftpropeller > -90.0){
            angleleftpropeller-=5.0;
            
            glutPostRedisplay();
            if (angleleftpropeller == -90.0)
                downward=false;
            glutTimerFunc(10, animationHandlerDown, 0);
            
        }
    }
}

// Side propellers will tilt back in the primary horizontal position with about 0 degree
void animationHandlerForward(int param)
{
    if (forward == true)
    {
        if (angleleftpropeller > 0){
            angleleftpropeller-=5.0;
            
            glutPostRedisplay();
            if (angleleftpropeller == 0.0)
                forward=false;
            glutTimerFunc(10, animationHandlerForward, 0);
            
        }
        else if (angleleftpropeller < 0) {
            angleleftpropeller+=5.0;
            
            glutPostRedisplay();
            if (angleleftpropeller == 0.0)
                forward=false;
            glutTimerFunc(10, animationHandlerForward, 0);
            
        }
    }
}

// Callback, handles input from the keyboard, function and arrow keys
void functionKeys(int key, int x, int y)
{
    // When left arrow key pressed, Zeppelin turns left
    if (key == GLUT_KEY_LEFT)
    {
        zeppelinUserAngle += 3;
    }
    // When right arrow key pressed, Zeppelin turns right
    if (key == GLUT_KEY_RIGHT)
    {
        zeppelinUserAngle -= 3;
    }
    // When up arrow key pressed, side propellers turn up
    if (key == GLUT_KEY_UP)
    {
        forward = true;
        upward = true;
        downward = true;
        glutTimerFunc(10, animationHandlerUp, 0);

    }
    // When down arrow key pressed, side propellers turn down
    if (key == GLUT_KEY_DOWN)
    {

        forward = true;
        upward = true;
        downward = true;
        glutTimerFunc(10, animationHandlerDown, 0);

    }

    glutPostRedisplay();   // Trigger a window redisplay
}

// Requirement 1.c
// My version of readOBJ() modified accordingly so it reads the  Zeppelin meshes parts saved as obj files accordingly
void readOBJ(const std::string& fileName)
{
    char buf[1024];
    char key[1024];
    int n;
    FILE *fin;

    int fc = 0; // face count
    int vc = 0; // vertex count
    int nc = 0; // normal count
    
    // First pass: Determine counts
        if ((fin = fopen(fileName.c_str(), "r"))) {
            while (fgets(buf, 1024, fin)) {
                if (sscanf(buf, "%s%n", key, &n) >= 1) {
                    if (!strcmp(key, "f")) {
                        fc += 3; // Assuming faces are triangles
                    } else if (!strcmp(key, "v")) {
                        vc++;
                    } else if (!strcmp(key, "vn")) {
                        nc++;
                    }
                }
            }
            fclose(fin);
            
            positions3 = (Vector3D *)malloc(vc * sizeof(Vector3D));
            normals3 = (Vector3D *)malloc(nc * sizeof(Vector3D));
            indices3 = (GLuint *)malloc(fc * sizeof(GLuint));
        }
    // Reset the variables after allocatin memory for the positions, normals, and indices arrays
    fc = 0; // face count
    vc = 0; // vertex count
    nc = 0; // normal count
    
    if ((fin = fopen(fileName.c_str(), "r")))
    {
        /* Process each line of the OBJ file, invoking the handler for each. */
        
        while (fgets(buf, 1024, fin))
            if (sscanf(buf, "%s%n", key, &n) >= 1)
            {
                const char *c = buf + n;

                if (!strcmp(key, "f"))
                {
                    sscanf(buf+1, "%d/%*d/%*d %d/%*d/%*d %d/%*d/%*d", &indices3[fc], &indices3[fc + 1], &indices3[fc + 2]);
                    fc += 3;
                }
                else if (!strcmp(key, "v"))
                {
                    sscanf(buf+1, "%f %f %f", &positions3[vc].x, &positions3[vc].y, &positions3[vc].z);
                    vc++;
                }
                else if (!strcmp(key, "vn"))
                {
                    sscanf(buf+2, "%f %f %f", &normals3[nc].x, &normals3[nc].y, &normals3[nc].z);
                    nc++;
                }
            }
        fclose(fin);

        numTris = fc / 3;
        numIndices = fc;
        numVertices = vc;
    }
}


// Use immediate mode rendering to draw the mesh as triangles rather than as quads
void drawTris3(Vector3D *positions3, Vector3D *normals3, GLuint *indices3, GLuint numIndices)
{
    glPushMatrix();

    for (GLuint i = 0; i < numIndices; i += 3)
    {
        glMaterialfv(GL_FRONT, GL_AMBIENT, zeppelinCabin_mat_ambient);
        glMaterialfv(GL_FRONT, GL_SPECULAR, zeppelinCabin_mat_specular);
        glMaterialfv(GL_FRONT, GL_DIFFUSE, zeppelinCabin_mat_diffuse);
        glMaterialfv(GL_FRONT, GL_SHININESS, zeppelinCabin_mat_shininess);
        
        glBegin(GL_TRIANGLES);

        // Vertex 1
        Vector3D vertexp1 = positions3[indices3[i]];
        Vector3D vertexn1 = normals3[indices3[i]];
        glNormal3f(vertexn1.x, vertexn1.y, vertexn1.z);
        glVertex3f(vertexp1.x, vertexp1.y, vertexp1.z);

        // Vertex 2
        Vector3D vertexp2 = positions3[indices3[i + 1]];
        Vector3D vertexn2 = normals3[indices3[i + 1]];
        glNormal3f(vertexn2.x, vertexn2.y, vertexn2.z);
        glVertex3f(vertexp2.x, vertexp2.y, vertexp2.z);

        // Vertex 3
        Vector3D vertexp3 = positions3[indices3[i + 2]];
        Vector3D vertexn3 = normals3[indices3[i + 2]];
        glNormal3f(vertexn3.x, vertexn3.y, vertexn3.z);
        glVertex3f(vertexp3.x, vertexp3.y, vertexp3.z);

        glEnd();
    }

    glPopMatrix();
}


// Function to set up the camera for the normal world view
void setupWorldCamera() {
    gluLookAt(0.0, 6.0, 47.0, // Camera position (above the world)
              0.0, 0.0, 0.0,  // Look at the center of the world
              0.0, 1.0, 0.0); // Up vector (assuming up is in the negative y-direction)

}
// Function to set up the camera for the first person view
void setupFPVCamera(float eyeX, float eyeY, float eyeZ) {
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    // Calculate the look at position based on Zeppelin's rotation
    float angleInRadians = zeppelinUserAngle * M_PI / 180.0f;
    float lookAtX = objectPositionPlayer[0] - cos(angleInRadians);
    float lookAtZ = objectPositionPlayer[2] + sin(angleInRadians);
    
    // Adjust the camera position to be relative to the zeppelin's position and orientation
//        glTranslatef(0, -moveUpDown, 0); // Adjust the camera height as needed
        glTranslatef(-5, 11, 2); // Move camera underneath the head of the zeppelin
    
    // Set up the camera at the specified position looking at the calculated look at position
    gluLookAt(eyeX, eyeY, eyeZ,  // Camera position
              lookAtX, objectPositionPlayer[1], lookAtZ,  // Look at position
              0.0, 1.0, 0.0);  // Up vector (assuming up is in the positive y direction)
    

}
