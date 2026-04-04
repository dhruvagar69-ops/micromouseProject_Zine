#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050.h>
#include <vector>
#include <queue>
#include <stack>

using namespace std;

#define START_ROW 0
#define START_COL 0
#define GOAL_4CELLS 1
#define GOAL_ROW  3
#define GOAL_COL  3
#define GOAL4_ROW 3
#define GOAL4_COL 3
#define STABLE_NEEDED   2
#define MAX_ITERATIONS  20
#define AUTO_CONVERGE   1
#define FIXED_ITERATIONS 3

#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_INT_PIN 18

#define XSHUT_FL 5
#define XSHUT_FR 15
#define XSHUT_L  16
#define XSHUT_R  17

#define ADDR_FL 0x30
#define ADDR_FR 0x31
#define ADDR_L  0x32
#define ADDR_R  0x33

#define AIN1 32
#define AIN2 13
#define PWMA 27
#define BIN1 33
#define BIN2 25
#define PWMB 26

#define ENC_A1 36
#define ENC_B1 39
#define ENC_A2 34
#define ENC_B2 35

#define TICKS_PER_CELL  1085 // to be tuned
#define TICKS_TURN      375 // to be tuned
#define BASE_SPEED      150 // to be tuned
#define MIN_SPEED       80 // to be tuned
#define TURN_SPEED      120 // to be tuned

#define WALL_THRESHOLD_FRONT 180 // to be tuned
#define WALL_THRESHOLD_SIDE  180 // to be tuned
#define WALL_CENTER_DIST     100 // to be tuned

#define KP_ENC  2.0f // to be tuned
#define KI_ENC  0.001f // to be tuned
#define KD_ENC  0.5f // to be tuned
#define KP_WALL 0.3f // to be tuned
#define KI_WALL 0.0f // to be tuned
#define KD_WALL 0.1f // to be tuned

#define UP    0
#define DOWN  1
#define LEFT  2
#define RIGHT 3

const int rows = 8;
const int cols = 8;

VL53L0X sensor_fl, sensor_fr, sensor_l, sensor_r;
MPU6050 mpu;

volatile long enc1 = 0;
volatile long enc2 = 0;

float current_yaw = 0.0f;
unsigned long last_mpu_time = 0;

typedef struct { int row, col, value; } coord;
typedef struct {
    bool walls[4];
    bool visited;
    int  angle_update;
    bool dead = 0;
} cell_info;
typedef struct { cell_info cells[8][8]; } wall_maze;

wall_maze maze;
const int dx[] = {1, -1, 0, 0};
const int dy[] = {0, 0, -1, 1};
queue<coord> myQueue;

void IRAM_ATTR enc1_ISR() {
    if (digitalRead(ENC_A1) == digitalRead(ENC_B1)) enc1++;
    else enc1--;
}
void IRAM_ATTR enc2_ISR() {
    if (digitalRead(ENC_A2) == digitalRead(ENC_B2)) enc2++;
    else enc2--;
}

void motors_stop() {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
    analogWrite(PWMA, 0);    analogWrite(PWMB, 0);
}

void set_motor1(int speed) {
    speed = constrain(speed, -255, 255);
    if (speed >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); }
    else            { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); speed = -speed; }
    analogWrite(PWMA, speed);
}

void set_motor2(int speed) {
    speed = constrain(speed, -255, 255);
    if (speed >= 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); }
    else            { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); speed = -speed; }
    analogWrite(PWMB, speed);
}

void update_yaw() {
    unsigned long now = micros();
    float dt = (now - last_mpu_time) / 1000000.0f;
    last_mpu_time = now;
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    float gz_dps = gz / 131.0f;
    if (abs(gz_dps) < 1.0f) gz_dps = 0;
    current_yaw += gz_dps * dt;
}

int read_front_left()  { return sensor_fl.readRangeContinuousMillimeters(); }
int read_front_right() { return sensor_fr.readRangeContinuousMillimeters(); }
int read_left()        { return sensor_l.readRangeContinuousMillimeters(); }
int read_right()       { return sensor_r.readRangeContinuousMillimeters(); }

bool wallFront() {
    int avg = (read_front_left() + read_front_right()) / 2;
    return avg < WALL_THRESHOLD_FRONT;
}

bool wallLeft()  { return read_left()  < WALL_THRESHOLD_SIDE; }
bool wallRight() { return read_right() < WALL_THRESHOLD_SIDE; }

void moveForward() {
    enc1 = 0; enc2 = 0;
    long prev_enc_error = 0;
    float enc_integral  = 0;
    float prev_wall_error = 0;
    float wall_integral   = 0;

    while (enc1 < TICKS_PER_CELL && enc2 < TICKS_PER_CELL) {
        update_yaw();

        
        long enc_error      = enc1 - enc2;
        enc_integral       += enc_error;
        long enc_deriv      = enc_error - prev_enc_error;
        prev_enc_error      = enc_error;
        enc_integral        = constrain(enc_integral, -500, 500);
        int enc_corr = (int)(KP_ENC*enc_error + KI_ENC*enc_integral + KD_ENC*enc_deriv);

        
        int wall_corr = 0;
        int ld = read_left(), rd = read_right();
        if (ld < 200 && rd < 200) {
            float we       = ld - rd;
            wall_integral += we;
            float wd       = we - prev_wall_error;
            prev_wall_error = we;
            wall_integral   = constrain(wall_integral, -300, 300);
            wall_corr = (int)(KP_WALL*we + KI_WALL*wall_integral + KD_WALL*wd);
        } else if (ld < 200) {
            wall_corr =  (int)(KP_WALL * (ld - WALL_CENTER_DIST));
        } else if (rd < 200) {
            wall_corr = -(int)(KP_WALL * (rd - WALL_CENTER_DIST));
        }

        int speed1 = BASE_SPEED - enc_corr - wall_corr;
        int speed2 = BASE_SPEED + enc_corr + wall_corr;

        long remaining = TICKS_PER_CELL - max(enc1, enc2);
        if (remaining < 200) { speed1 -= 50; speed2 -= 50; }

        set_motor1(constrain(speed1, MIN_SPEED, 255));
        set_motor2(constrain(speed2, MIN_SPEED, 255));
        delay(5);
    }
    motors_stop();
    delay(100);
}

void turnRight() {
    float target_yaw = current_yaw - 90.0f;
    enc1 = 0; enc2 = 0;

    
    while (abs(enc1) < TICKS_TURN && abs(enc2) < TICKS_TURN) {
        update_yaw();
        set_motor1(TURN_SPEED);
        set_motor2(-TURN_SPEED);
        delay(2);
    }
    motors_stop();
    delay(50);

    
    update_yaw();
    float yaw_error = current_yaw - target_yaw;
    while (abs(yaw_error) > 4.0f) {
        update_yaw();
        yaw_error = current_yaw - target_yaw;
        if (yaw_error > 0) { set_motor1(60);  set_motor2(-60); }
        else               { set_motor1(-60); set_motor2(60);  }
        delay(2);
    }
    motors_stop();
    delay(100);
}

void turnLeft() {
    float target_yaw = current_yaw + 90.0f;
    enc1 = 0; enc2 = 0;

    while (abs(enc1) < TICKS_TURN && abs(enc2) < TICKS_TURN) {
        update_yaw();
        set_motor1(-TURN_SPEED);
        set_motor2(TURN_SPEED);
        delay(2);
    }
    motors_stop();
    delay(50);

    update_yaw();
    float yaw_error = current_yaw - target_yaw;
    while (abs(yaw_error) > 4.0f) {
        update_yaw();
        yaw_error = current_yaw - target_yaw;
        if (yaw_error < 0) { set_motor1(-60); set_motor2(60);  }
        else               { set_motor1(60);  set_motor2(-60); }
        delay(2);
    }
    motors_stop();
    delay(100);
}


bool isValid(int x, int y) { return (x >= 0 && x < rows && y >= 0 && y < cols); }

void init_arr(vector<vector<int>>& arr) {
    for (int i = 0; i < rows; i++) {
        vector<int> row;
        for (int j = 0; j < cols; j++) row.push_back(-1);
        arr.push_back(row);
    }
}

void check_and_fill(vector<vector<int>>& arr, int row, int col, int value) {
    if (row<0||col<0||row>=(int)arr.size()||col>=(int)arr[0].size()||arr[row][col]!=-1) return;
    value++;
    coord p = {row, col, value};
    myQueue.push(p);
    arr[row][col] = value;
}

void init_flood(vector<vector<int>>& arr, int row, int col) {
    int c = 0;
    myQueue.push({row, col, c}); arr[row][col] = 0;
#if GOAL_4CELLS == 1
    myQueue.push({row+1, col, c});   arr[row+1][col]   = 0;
    myQueue.push({row, col+1, c});   arr[row][col+1]   = 0;
    myQueue.push({row+1, col+1, c}); arr[row+1][col+1] = 0;
#endif
    while (!myQueue.empty()) {
        coord f = myQueue.front(); myQueue.pop();
        check_and_fill(arr, f.row+1, f.col, f.value);
        check_and_fill(arr, f.row-1, f.col, f.value);
        check_and_fill(arr, f.row, f.col+1, f.value);
        check_and_fill(arr, f.row, f.col-1, f.value);
    }
}

void init_flood_start(vector<vector<int>>& arr, int row_, int col_, int back_) {
    int c = 0;
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++) {
            arr[i][j] = -1;
            if (back_==2 && !maze.cells[i][j].visited) {
                arr[i][j] = 255;
                maze.cells[i][j].dead = true;
            }
        }
    if (back_ != 1) {
#if GOAL_4CELLS == 1
        myQueue.push({row_+1, col_, c});   arr[row_+1][col_]   = 0;
        myQueue.push({row_, col_+1, c});   arr[row_][col_+1]   = 0;
        myQueue.push({row_+1, col_+1, c}); arr[row_+1][col_+1] = 0;
#endif
    }
    myQueue.push({row_, col_, c}); arr[row_][col_] = 0;
    while (!myQueue.empty()) {
        coord f = myQueue.front(); myQueue.pop();
        for (int i = 0; i < 4; i++) {
            int nr = f.row + dy[i], nc = f.col + dx[i];
            if (!maze.cells[f.row][f.col].walls[i])
                check_and_fill(arr, nr, nc, f.value);
        }
    }
}

bool check_wall_angle(cell_info cell, int& dir) {
    switch (cell.angle_update) {
        case 270: if(dir%2==0)dir++; else dir--; break;
        case 0:   if(dir==0||dir==1)dir+=2; else if(dir==2)dir=1; else dir=0; break;
        case 180: if(dir==2||dir==3)dir-=2; else if(dir==0)dir=3; else dir=2; break;
    }
    return cell.walls[dir];
}

cell_info cell_direction_adjust(cell_info cell) {
    cell_info cn = cell;
    for (int i = 0; i < 4; i++) {
        int ind = i;
        switch (cell.angle_update) {
            case 270: if(i%2==0)ind++; else ind--; break;
            case 0:   if(i==0||i==1)ind+=2; else if(i==2)ind=1; else ind=0; break;
            case 180: if(i==2||i==3)ind-=2; else if(i==0)ind=3; else ind=2; break;
        }
        cn.walls[i] = cell.walls[ind];
    }
    return cn;
}

void go_to_cell(int& angle_now, int dir) {
    switch (dir) {
        case UP:    moveForward(); break;
        case DOWN:  angle_now-=180; turnRight(); turnRight(); moveForward(); break;
        case LEFT:  angle_now+=90;  turnLeft();  moveForward(); break;
        case RIGHT: angle_now-=90;  turnRight(); moveForward(); break;
    }
    angle_now = ((angle_now % 360) + 360) % 360;
}

coord get_min_neighbour(cell_info cw, coord cur, vector<vector<int>>& arr, bool change_=0) {
    int mn = 255;
    coord ns; ns.value = -1;
    for (int dir = 0; dir < 4; dir++) {
        int nr = cur.row+dy[dir], nc = cur.col+dx[dir], ind = dir;
        bool chk = cw.walls[dir];
        if (change_) chk = check_wall_angle(cw, ind);
        if (isValid(nr,nc) && !chk && arr[nr][nc] <= mn) {
            mn = arr[nr][nc];
            ns = {nr, nc, ind};
        }
    }
    return ns;
}

void flood(stack<coord>& sf, vector<vector<int>>& arr) {
    while (!sf.empty()) {
        coord cs = sf.top(); sf.pop();
        coord ns = get_min_neighbour(maze.cells[cs.row][cs.col], cs, arr);
        int mn = arr[ns.row][ns.col];
        if (arr[cs.row][cs.col]-1 != mn) {
            for (int i = 0; i < 4; i++) {
                int nr = cs.row+dy[i], nc = cs.col+dx[i];
                if (isValid(nr,nc) && arr[nr][nc]!=0 && !maze.cells[cs.row][cs.col].walls[i])
                    sf.push({nr, nc, 0});
            }
            if (arr[cs.row][cs.col] != 0) arr[cs.row][cs.col] = mn+1;
        }
    }
}

cell_info update_walls(int angle_now, int row, int col) {
    cell_info nc;
    nc.angle_update  = angle_now;
    nc.walls[UP]     = wallFront();
    nc.walls[DOWN]   = 0;
    nc.walls[LEFT]   = wallLeft();
    nc.walls[RIGHT]  = wallRight();
    nc.dead = 0; nc.visited = 1;
    maze.cells[row][col] = cell_direction_adjust(nc);
    if (nc.walls[UP]&&nc.walls[LEFT]&&nc.walls[RIGHT]&&row!=0&&col!=0)
        maze.cells[row][col].dead = 1;
    for (int i = 0; i < 4; i++) {
        int nr = row+dy[i], ncc = col+dx[i];
        if (isValid(nr, ncc)) {
            if (i==UP)    maze.cells[nr][ncc].walls[DOWN]  = maze.cells[row][col].walls[UP];
            if (i==LEFT)  maze.cells[nr][ncc].walls[RIGHT] = maze.cells[row][col].walls[LEFT];
            if (i==RIGHT) maze.cells[nr][ncc].walls[LEFT]  = maze.cells[row][col].walls[RIGHT];
        }
    }
    return nc;
}

coord floodfill(coord start, coord dest, vector<vector<int>>& arr, int& angle_now) {
    queue<coord> pq; pq.push(start);
    coord cur = start, next_step;
    stack<coord> sf; sf.push(start);
    int cost = 0;

    while (1) {
        if (!pq.empty()) {
            cur = pq.front();
            update_walls(angle_now, cur.row, cur.col);
            if (arr[cur.row][cur.col] == arr[dest.row][dest.col]) break;
            flood(sf, arr);
            pq.pop();
            next_step = get_min_neighbour(maze.cells[cur.row][cur.col], cur, arr, 1);
            pq.push(next_step); sf.push(next_step);
            go_to_cell(angle_now, next_step.value);
            cost++;
        } else break;
    }
    while (!pq.empty()) pq.pop();
    Serial.printf("total_cost:%d\n", cost);
    return {next_step.row, next_step.col, 0};
}

void init_maze() {
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++) {
            maze.cells[i][j].visited = 0;
            maze.cells[i][j].angle_update = 90;
            maze.cells[i][j].dead = 0;
            for (int k = 0; k < 4; k++) maze.cells[i][j].walls[k] = 0;
        }
}

void go_to_cell_shorted(int& angle, int dir) {
    int nd = dir;
    switch (angle) {
        case 270: if(dir%2==0)nd++; else nd--; break;
        case 0:   if(dir==0||dir==1)nd+=2; else if(dir==2)nd=1; else nd=0; break;
        case 180: if(dir==2||dir==3)nd-=2; else if(dir==0)nd=3; else nd=2; break;
    }
    go_to_cell(angle, nd);
}

void shortest_path_go(vector<vector<int>>& arr, int angle_now, coord start, coord dest) {
    coord cur = start;
    int angle = angle_now;
    Serial.println("Running shortest path...");
    for (int i = 0; i < arr[start.row][start.col]; i++) {
        int next_dir = -1, sr, sc;
        for (int dir = 0; dir < 4; dir++) {
            int nr = cur.row+dy[dir], nc = cur.col+dx[dir];
            if (isValid(nr,nc) && !maze.cells[cur.row][cur.col].walls[dir]) {
                if (arr[nr][nc] < arr[cur.row][cur.col]) {
                    next_dir = dir; sr = nr; sc = nc;
                }
            }
        }
        if (next_dir != -1) {
            go_to_cell_shorted(angle, next_dir);
            cur.row = sr; cur.col = sc;
        } else { Serial.println("Path blocked!"); break; }
    }
    Serial.println("Speed run complete!");
}


void init_sensors() {
    pinMode(XSHUT_FL, OUTPUT); pinMode(XSHUT_FR, OUTPUT);
    pinMode(XSHUT_L,  OUTPUT); pinMode(XSHUT_R,  OUTPUT);
    digitalWrite(XSHUT_FL, LOW); digitalWrite(XSHUT_FR, LOW);
    digitalWrite(XSHUT_L,  LOW); digitalWrite(XSHUT_R,  LOW);
    delay(10);

    digitalWrite(XSHUT_FL, HIGH); delay(10);
    sensor_fl.init(); sensor_fl.setAddress(ADDR_FL);
    sensor_fl.setTimeout(500); sensor_fl.startContinuous();

    digitalWrite(XSHUT_FR, HIGH); delay(10);
    sensor_fr.init(); sensor_fr.setAddress(ADDR_FR);
    sensor_fr.setTimeout(500); sensor_fr.startContinuous();

    digitalWrite(XSHUT_L, HIGH); delay(10);
    sensor_l.init(); sensor_l.setAddress(ADDR_L);
    sensor_l.setTimeout(500); sensor_l.startContinuous();

    digitalWrite(XSHUT_R, HIGH); delay(10);
    sensor_r.init(); sensor_r.setAddress(ADDR_R);
    sensor_r.setTimeout(500); sensor_r.startContinuous();

    Serial.println("TOF sensors ready");
}

void init_motors() {
    pinMode(AIN1,OUTPUT); pinMode(AIN2,OUTPUT); pinMode(PWMA,OUTPUT);
    pinMode(BIN1,OUTPUT); pinMode(BIN2,OUTPUT); pinMode(PWMB,OUTPUT);
    pinMode(ENC_A1,INPUT); pinMode(ENC_B1,INPUT);
    pinMode(ENC_A2,INPUT); pinMode(ENC_B2,INPUT);
    attachInterrupt(digitalPinToInterrupt(ENC_A1), enc1_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_A2), enc2_ISR, CHANGE);
    motors_stop();
    Serial.println("Motors ready");
}

void init_mpu() {
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 failed!");
        while(1);
    }
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    Serial.println("Keep robot still for 2 seconds...");
    delay(2000);
    last_mpu_time = micros();
    Serial.println("MPU6050 ready");
}


void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);

    init_sensors();
    init_motors();
    init_mpu();

    vector<vector<int>> arr;
    init_arr(arr);
    init_maze();

#if GOAL_4CELLS == 1
    init_flood(arr, GOAL4_ROW, GOAL4_COL);
    coord dest  = {GOAL4_ROW, GOAL4_COL, arr[GOAL4_ROW][GOAL4_COL]};
#else
    init_flood(arr, GOAL_ROW, GOAL_COL);
    coord dest  = {GOAL_ROW, GOAL_COL, arr[GOAL_ROW][GOAL_COL]};
#endif

    coord start = {START_ROW, START_COL, arr[START_ROW][START_COL]};
    int angle_now = 90;
    coord new_coord;

    Serial.println("Place robot at start. Starting in 3 seconds...");
    delay(3000);

#if AUTO_CONVERGE == 1
    Serial.println("AUTOMATIC MODE");
    int prev_cost = -1, stable_count = 0;

    for (int m = 0; m < MAX_ITERATIONS; m++) {
        Serial.printf("=== Iteration %d ===\n", m+1);
        new_coord = floodfill(start, dest, arr, angle_now);
        init_flood_start(arr, START_ROW, START_COL, 1);
        new_coord = floodfill(new_coord, start, arr, angle_now);

#if GOAL_4CELLS == 1
        init_flood_start(arr, GOAL4_ROW, GOAL4_COL, 2);
#else
        init_flood_start(arr, GOAL_ROW, GOAL_COL, 2);
#endif

        int current_cost = arr[START_ROW][START_COL];
        Serial.printf("path_length:%d\n", current_cost);

        if (current_cost == prev_cost) {
            stable_count++;
            if (stable_count >= STABLE_NEEDED) {
                Serial.printf("Converged at iteration %d!\n", m+1);
                break;
            }
        } else stable_count = 0;
        prev_cost = current_cost;
    }

#else
    Serial.printf("FIXED MODE: %d iterations\n", FIXED_ITERATIONS);
    for (int m = 0; m < FIXED_ITERATIONS; m++) {
        Serial.printf("=== Iteration %d ===\n", m+1);
        new_coord = floodfill(start, dest, arr, angle_now);
        init_flood_start(arr, START_ROW, START_COL, 1);
        new_coord = floodfill(new_coord, start, arr, angle_now);
#if GOAL_4CELLS == 1
        init_flood_start(arr, GOAL4_ROW, GOAL4_COL, 2);
#else
        init_flood_start(arr, GOAL_ROW, GOAL_COL, 2);
#endif
        Serial.printf("path_length:%d\n", arr[START_ROW][START_COL]);
    }
#endif

    Serial.println("Exploration done! Starting speed run in 5 seconds...");
    delay(5000);
    shortest_path_go(arr, angle_now, new_coord, dest);
}

void loop() {
    update_yaw();
    delay(5);
}
