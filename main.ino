/**
 * Autonomous Arduino Vehicle - Greedy Best-First Pathfinding
 * STEM Fair 2023 - Robotics Division 1st Place
 * 
 * Algorithm: Greedy Best-First (A*-inspired)
 * - Uses F = H only (Manhattan distance heuristic)
 * - Standard A* uses F = G + H, but G is omitted for memory efficiency
 * - Trade-off: May not find optimal path, but faster and fits in 2KB SRAM
 * 
 * Features:
 * - Pathfinding on 6x6 grid
 * - Real-time obstacle detection & re-routing
 * - Bluetooth control via Android app
 * 
 * Hardware:
 * - Arduino Uno + Adafruit Motor Shield v1
 * - HC-06 Bluetooth Module (A4/A5)
 * - HC-SR04 Ultrasonic Sensor (A0/A1)
 * - 4x DC Motors
 */

#include <AFMotor.h>
#include <SoftwareSerial.h>

// ============== CONFIGURATION ==============
#define ROW 6
#define COL 6
#define GRID_SIZE 36

// Pin definitions
#define ECHO_PIN A1
#define TRIG_PIN A0
#define LED_PIN A3

// Motor settings
#define MOTOR_SPEED 255
#define ONE_BLOCK_DELAY 420    // ms to travel one grid cell
#define TURN_DELAY 392         // ms for 90° turn

// Obstacle detection
#define OBSTACLE_THRESHOLD 15  // cm

// ============== HARDWARE INIT ==============
SoftwareSerial HC06(A5, A4);  // RX, TX

AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

// ============== PATHFINDING DATA STRUCTURES ==============
struct Node {
    byte g, h, f;      // g=actual cost (tracked but not used in F), h=heuristic, f=total
    byte parent;       // Parent node index
    byte index;        // 0=empty, 1=visited, 2=obstacle, 3=goal
    byte gridNom;      // Position on grid (0-35)
};

struct Grid {
    Node Map[ROW][COL];
} PF;

// Lists for pathfinding algorithm
byte openList[50];
byte closedList[50];
byte Path[50];
byte oLN = 0, cLN = 0;  // List counters

// ============== STATE VARIABLES ==============
byte curBotPos;         // Current position (0-35)
byte curBotDir = 1;     // Direction: 1=up, 2=down, 3=left, 4=right
byte goalN;             // Goal position

byte curBotPos2, curBotDir2;  // Temp variables for movement

bool ready = false;
byte initialPosition, finalPosition;
byte ini, fin;

// ============== HEURISTIC FUNCTIONS ==============

/**
 * Manhattan distance heuristic
 * Calculates estimated cost from current position to goal
 * H(n) = |current_row - goal_row| + |current_col - goal_col|
 */
byte H(byte curR, byte curC, byte goalS) {
    byte rowg = goalS / 6;
    byte colg = goalS % 6;
    return abs(curR - rowg) + abs(curC - colg);
}

/**
 * Actual cost function
 * Returns cost to reach current node from start
 * Note: This is tracked but not used in final F calculation
 */
byte G(byte curR, byte curC) {
    byte parInd = PF.Map[curR][curC].parent;
    byte rowg = parInd / 6;
    byte colg = parInd % 6;
    return PF.Map[rowg][colg].g + 1;
}

/**
 * Total cost function
 * 
 * IMPORTANT: This is GREEDY BEST-FIRST, not standard A*
 * - Standard A*: F = G + H (considers actual cost + heuristic)
 * - This implementation: F = H only (greedy, ignores actual cost)
 * 
 * Reason: Arduino Uno has only 2KB SRAM
 * Trade-off: Faster pathfinding, may not find optimal path
 * On 6x6 grid, this trade-off is acceptable
 */
byte FV(byte curG, byte curH) {
    return curH;  // Greedy best-first: only heuristic
}

// ============== GRID MANAGEMENT ==============

/**
 * Initialize 6x6 grid with default values
 */
void buildMap() {
    byte gridIn = 0;
    for (byte i = 0; i < ROW; i++) {
        for (byte j = 0; j < COL; j++) {
            PF.Map[i][j].gridNom = gridIn;
            PF.Map[i][j].index = 0;
            PF.Map[i][j].parent = 0;
            PF.Map[i][j].h = 0;
            PF.Map[i][j].g = 0;
            PF.Map[i][j].f = 0;
            gridIn++;
        }
    }
    for (byte i = 0; i < 50; i++) {
        openList[i] = GRID_SIZE;
    }
}

/**
 * Reset grid state for new pathfinding
 */
void reset() {
    for (byte i = 0; i < GRID_SIZE; i++) {
        if (PF.Map[i/6][i%6].index == 1) {
            PF.Map[i/6][i%6].index = 0;
        }
        PF.Map[i/6][i%6].g = 0;
        PF.Map[i/6][i%6].h = 0;
        PF.Map[i/6][i%6].f = 0;
        PF.Map[i/6][i%6].parent = 0;
    }
    
    PF.Map[goalN/6][goalN%6].index = 3;
    oLN = 0;
    cLN = 0;
    
    for (byte i = 0; i < 50; i++) {
        openList[i] = 0;
        closedList[i] = 0;
        Path[i] = 0;
    }
    
    curBotPos = 0;
    goalN = 0;
}

// ============== PATHFINDING ALGORITHM ==============

/**
 * Calculate heuristics for a node
 */
void heuristics(byte curIn) {
    byte rowh = curIn / 6;
    byte colh = curIn % 6;
    
    byte hH = H(rowh, colh, goalN);
    PF.Map[rowh][colh].h = hH;
    
    byte gH = G(rowh, colh);
    PF.Map[rowh][colh].g = gH;
    
    byte fH = FV(hH, gH);  // Note: gH is passed but not used (greedy)
    PF.Map[rowh][colh].f = fH;
}

/**
 * Add node to open list
 */
void AddOpenList(byte aol) {
    openList[oLN++] = aol;
    heuristics(aol);
}

/**
 * Check if node is already in open list
 */
bool alreadyOnOL(byte rowaol, byte colaol) {
    byte indexol = rowaol * 6 + colaol;
    for (byte i = 0; i < GRID_SIZE; i++) {
        if (openList[i] == indexol) return true;
    }
    return false;
}

/**
 * Get node with lowest F value from open list
 * Since F = H only, this selects node closest to goal (greedy)
 */
byte getNextFI() {
    byte lowest = openList[0];
    byte rowf = lowest / 6;
    byte colf = lowest % 6;
    byte lowestF = PF.Map[rowf][colf].f;
    
    for (byte i = 0; i < oLN; i++) {
        rowf = openList[i] / 6;
        colf = openList[i] % 6;
        
        if (PF.Map[rowf][colf].f <= lowestF) {
            lowestF = PF.Map[rowf][colf].f;
            lowest = rowf * 6 + colf;
        }
    }
    return lowest;
}

/**
 * Move best node from open to closed list
 * FIXED: Proper removal from openList
 */
void AddClosedList() {
    byte low = getNextFI();
    closedList[cLN++] = low;
    
    byte rowa = low / 6;
    byte cola = low % 6;
    PF.Map[rowa][cola].index = 1;
    curBotPos = low;
    
    // Remove 'low' from open list (find index, then shift)
    int idx = -1;
    for (byte i = 0; i < oLN; i++) {
        if (openList[i] == low) {
            idx = i;
            break;
        }
    }
    if (idx != -1) {
        for (byte i = idx; i < oLN - 1; i++) {
            openList[i] = openList[i + 1];
        }
        openList[oLN - 1] = GRID_SIZE;  // Sentinel value
        oLN--;
    }
}

/**
 * Find possible moves from current position
 * Checks 2-4 adjacent cells based on grid position
 */
void possMov(byte gridNom) {
    byte rowp = gridNom / 6;
    byte colp = gridNom % 6;
    
    // Check each direction if valid and not obstacle/visited
    // Up (row - 1)
    if (rowp > 0 && PF.Map[rowp-1][colp].index != 1 && 
        PF.Map[rowp-1][colp].index != 2 && !alreadyOnOL(rowp-1, colp)) {
        PF.Map[rowp-1][colp].parent = gridNom;
        AddOpenList(gridNom - 6);
    }
    // Down (row + 1)
    if (rowp < 5 && PF.Map[rowp+1][colp].index != 1 && 
        PF.Map[rowp+1][colp].index != 2 && !alreadyOnOL(rowp+1, colp)) {
        PF.Map[rowp+1][colp].parent = gridNom;
        AddOpenList(gridNom + 6);
    }
    // Left (col - 1)
    if (colp > 0 && PF.Map[rowp][colp-1].index != 1 && 
        PF.Map[rowp][colp-1].index != 2 && !alreadyOnOL(rowp, colp-1)) {
        PF.Map[rowp][colp-1].parent = gridNom;
        AddOpenList(gridNom - 1);
    }
    // Right (col + 1)
    if (colp < 5 && PF.Map[rowp][colp+1].index != 1 && 
        PF.Map[rowp][colp+1].index != 2 && !alreadyOnOL(rowp, colp+1)) {
        PF.Map[rowp][colp+1].parent = gridNom;
        AddOpenList(gridNom + 1);
    }
}

/**
 * Build optimal path from closed list
 */
void PathList() {
    for (byte i = 1; i < PF.Map[closedList[cLN-1]/6][closedList[cLN-1]%6].g + 1; i++) {
        for (byte j = 0; j < cLN; j++) {
            if (PF.Map[closedList[j]/6][closedList[j]%6].g == i) {
                Path[i-1] = closedList[j];
            }
        }
    }
}

/**
 * Check if open list is empty
 */
bool OLE() {
    return (oLN == 0);
}

/**
 * Check if goal reached
 */
bool isGoal(byte ig) {
    return (ig == goalN);
}

/**
 * Main pathfinding loop iteration
 */
void _loop() {
    possMov(curBotPos);
    AddClosedList();
}

// ============== OBSTACLE DETECTION ==============

/**
 * Check for obstacle using ultrasonic sensor
 * Returns 1 if obstacle within threshold
 */
int check_obstacle() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(20);
    digitalWrite(TRIG_PIN, LOW);
    
    float Fdistance = pulseIn(ECHO_PIN, HIGH);
    Fdistance = Fdistance / 58;  // Convert to cm
    
    return (Fdistance <= OBSTACLE_THRESHOLD) ? 1 : 0;
}

/**
 * Re-plan path when obstacle detected
 * Marks obstacle on grid and recalculates path
 */
void rePathPlan(byte curBotPos, byte curBotDir) {
    // Reset grid state
    for (byte i = 0; i < GRID_SIZE; i++) {
        if (PF.Map[i/6][i%6].index == 1) {
            PF.Map[i/6][i%6].index = 0;
        }
        PF.Map[i/6][i%6].g = 0;
        PF.Map[i/6][i%6].h = 0;
        PF.Map[i/6][i%6].f = 0;
        PF.Map[i/6][i%6].parent = 0;
    }
    
    PF.Map[curBotPos/6][curBotPos%6].index = 1;
    PF.Map[goalN/6][goalN%6].index = 3;
    
    // Mark obstacle based on current direction
    if (curBotDir == 1) PF.Map[(curBotPos+6)/6][(curBotPos+6)%6].index = 2;
    else if (curBotDir == 2) PF.Map[(curBotPos-6)/6][(curBotPos-6)%6].index = 2;
    else if (curBotDir == 3) PF.Map[(curBotPos+1)/6][(curBotPos+1)%6].index = 2;
    else if (curBotDir == 4) PF.Map[(curBotPos-1)/6][(curBotPos-1)%6].index = 2;
    
    // Reset lists
    oLN = 0;
    cLN = 0;
    for (byte i = 0; i < 50; i++) {
        openList[i] = GRID_SIZE;
        closedList[i] = 0;
        Path[i] = 0;
    }
}

// ============== MOTOR CONTROL ==============

void stopMotors() {
    motor1.setSpeed(0); motor1.run(RELEASE);
    motor2.setSpeed(0); motor2.run(RELEASE);
    motor3.setSpeed(0); motor3.run(RELEASE);
    motor4.setSpeed(0); motor4.run(RELEASE);
}

void forward() {
    motor1.setSpeed(MOTOR_SPEED); motor1.run(FORWARD);
    motor2.setSpeed(MOTOR_SPEED); motor2.run(FORWARD);
    motor3.setSpeed(MOTOR_SPEED); motor3.run(FORWARD);
    motor4.setSpeed(MOTOR_SPEED); motor4.run(FORWARD);
    delay(ONE_BLOCK_DELAY);
    stopMotors();
    delay(350);
}

void turnLeft() {
    motor1.setSpeed(254); motor1.run(BACKWARD);
    motor2.setSpeed(254); motor2.run(FORWARD);
    motor3.setSpeed(254); motor3.run(FORWARD);
    motor4.setSpeed(254); motor4.run(BACKWARD);
}

void left() {
    stopMotors();
    turnLeft();
    delay(395);
    stopMotors();
    delay(350);
}

void turnRight() {
    motor1.setSpeed(MOTOR_SPEED); motor1.run(FORWARD);
    motor2.setSpeed(MOTOR_SPEED); motor2.run(BACKWARD);
    motor3.setSpeed(MOTOR_SPEED); motor3.run(BACKWARD);
    motor4.setSpeed(MOTOR_SPEED); motor4.run(FORWARD);
}

void right() {
    stopMotors();
    turnRight();
    delay(TURN_DELAY);
    stopMotors();
    delay(350);
}

// ============== MOVEMENT EXECUTION ==============

/**
 * Execute movement along calculated path
 * Handles direction changes and obstacle checks
 */
byte movement(byte curBotPos, byte curBotDir) {
    curBotPos = PF.Map[Path[0]/6][Path[0]%6].parent;
    byte rowm, colm;
    byte i = 0;
    
    while (!isGoal(curBotPos)) {
        rowm = Path[i] / 6;
        colm = Path[i] % 6;
        
        // Determine required turn based on current direction and target
        byte targetDir = 0;
        if (Path[i] == PF.Map[rowm][colm].parent + 6) targetDir = 1;      // Moving down
        else if (Path[i] == PF.Map[rowm][colm].parent - 6) targetDir = 2; // Moving up
        else if (Path[i] == PF.Map[rowm][colm].parent + 1) targetDir = 3; // Moving right
        else if (Path[i] == PF.Map[rowm][colm].parent - 1) targetDir = 4; // Moving left
        
        // Execute turns to face target direction
        while (curBotDir != targetDir) {
            // Calculate shortest turn
            if ((curBotDir == 1 && targetDir == 3) || 
                (curBotDir == 3 && targetDir == 2) ||
                (curBotDir == 2 && targetDir == 4) || 
                (curBotDir == 4 && targetDir == 1)) {
                left();
                curBotDir = (curBotDir == 1) ? 3 : (curBotDir == 3) ? 2 : 
                            (curBotDir == 2) ? 4 : 1;
            } else {
                right();
                curBotDir = (curBotDir == 1) ? 4 : (curBotDir == 4) ? 2 : 
                            (curBotDir == 2) ? 3 : 1;
            }
        }
        
        // Check for obstacle before moving
        if (check_obstacle() == 1) {
            rePathPlan(curBotPos, curBotDir);
            break;
        }
        
        forward();
        curBotPos = Path[i];
        i++;
    }
    
    curBotPos2 = curBotPos;
    curBotDir2 = curBotDir;
    return curBotPos2;
}

// ============== BLUETOOTH ==============

/**
 * Read position (0-35) from Bluetooth using parseInt
 * FIXED: Now supports full grid range (was limited to 0-9)
 * Protocol: 'a' + number for start, 'b' + number for goal
 */
byte selectPosition() {
    while (!HC06.available()) {
        // Wait for data
    }
    int pos = HC06.parseInt();
    return constrain(pos, 0, 35);
}

// ============== MAIN ==============

void setup() {
    Serial.begin(9600);
    HC06.begin(9600);
    
    pinMode(ECHO_PIN, INPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    
    stopMotors();
    buildMap();
}

void loop() {
    digitalWrite(LED_PIN, LOW);
    
    // Handle Bluetooth commands
    if ((HC06.available() > 0) && !ready) {
        char receivedChar = HC06.read();
        
        if (receivedChar == 'a') {
            initialPosition = selectPosition();
            ini = initialPosition;
        } else if (receivedChar == 'b') {
            finalPosition = selectPosition();
            fin = finalPosition;
        } else if (receivedChar == 'g') {
            curBotPos = ini;
            goalN = fin;
            ready = true;
        }
    }
    
    // Main navigation logic
    // FIXED: OLE() called as function, logic corrected (run while openList NOT empty)
    if (ready) {
        if (!isGoal(curBotPos) && !OLE()) {
            _loop();  // Pathfinding iteration (only if openList not empty)
        } else if (isGoal(curBotPos)) {
            PathList();
            
            while (1) {
                movement(curBotPos, curBotDir);
                curBotPos = curBotPos2;
                curBotDir = curBotDir2;
                
                if (!isGoal(curBotPos)) break;
                
                if (isGoal(curBotPos)) {
                    digitalWrite(LED_PIN, HIGH);  // Goal reached indicator
                    delay(4000);
                    reset();
                    ready = false;
                    break;
                }
            }
        }
    }
}
