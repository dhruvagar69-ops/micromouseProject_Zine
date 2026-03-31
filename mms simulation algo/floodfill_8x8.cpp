#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <thread>
#include <chrono>
#include <conio.h>
#include "API.h"
#include <stack>

using namespace std;

// Maze is 8x8, valid coordinates are 0 to 7
// Row 0 = left,     Row 7 = right
// Col 0 = bottom,   Col 7 = top

#define GOAL_4CELLS 0 // Goal type: set GOAL_4CELLS to 1 for 4-cell center goal
                      // set GOAL_4CELLS to 0 for single cell goal

#define START_ROW 0       // Starting row
#define START_COL 0       // Starting col

// If GOAL_4CELLS = 0, set your single goal cell here
#define GOAL_ROW 7
#define GOAL_COL 7

// If GOAL_4CELLS = 1, top-left cell of the 4-cell goal
// The 4 cells will be (GOAL4_ROW, GOAL4_COL),
//                     (GOAL4_ROW+1, GOAL4_COL),
//                     (GOAL4_ROW, GOAL4_COL+1),
//                     (GOAL4_ROW+1, GOAL4_COL+1)
#define GOAL4_ROW 3       // top-left of 4-cell goal
#define GOAL4_COL 3       // top-left of 4-cell goal

// Set to 1 for automatic convergence, 0 for fixed iterations
#define AUTO_CONVERGE 0

// If AUTO_CONVERGE = 0, how many iterations to run
#define FIXED_ITERATIONS 1

// Convergence: stop after path is stable for this many iterations
#define STABLE_NEEDED 2

// Safety cap on max iterations even if not converged
#define MAX_ITERATIONS 20

// Speed run wait time in seconds after yellow path is shown
#define SPEEDRUN_WAIT 5

#define UP 0
#define DOWN 1
#define LEFT 2
#define RIGHT 3

const int rows = 8;
const int cols = 8;

typedef struct coor {
    int row;
    int col;
    int value;
} coord;

typedef struct cell_infos {
    bool walls[4];
    bool visited;
    int angle_update;
    bool dead = 0;
} cell_info;

typedef struct wall_mazes {
    cell_info cells[8][8];
} wall_maze;

void log(const std::string& text) {
    std::cerr << text << std::endl;
}

bool isValid(int x, int y) {
    return (x >= 0 && x < rows && y >= 0 && y < cols);
}

wall_maze maze;
const int dx[] = {1, -1, 0, 0};
const int dy[] = {0, 0, -1, 1};
std::queue<coord> myQueue;

void init_arr(std::vector<std::vector<int>>& arr, int row, int col) {
    for (int i = 0; i < row; i++) {
        std::vector<int> arr_row;
        for (int j = 0; j < col; j++) arr_row.push_back(-1);
        arr.push_back(arr_row);
    }
}

void check_and_fill(std::vector<std::vector<int>>& arr, int row, int col, int value) {
    if (row < 0 || col < 0 || row >= (int)arr.size() || col >= (int)arr[0].size() || arr[row][col] != -1) return;
    value += 1;
    coord point = {row, col, value};
    myQueue.push(point);
    arr[row][col] = value;
}

void init_flood(std::vector<std::vector<int>>& arr, int row, int col) {
    int count_ = 0;
    coord point = {row, col, count_};
    myQueue.push(point);
    arr[row][col] = 0;

#if GOAL_4CELLS == 1
    // Seed all 4 center cells as distance 0
    coord point2 = {row + 1, col, count_};
    myQueue.push(point2);
    arr[row + 1][col] = 0;
    coord point3 = {row, col + 1, count_};
    myQueue.push(point3);
    arr[row][col + 1] = 0;
    coord point4 = {row + 1, col + 1, count_};
    myQueue.push(point4);
    arr[row + 1][col + 1] = 0;
#endif

    while (!myQueue.empty()) {
        coord frontCoord = myQueue.front();
        myQueue.pop();
        check_and_fill(arr, frontCoord.row + 1, frontCoord.col, frontCoord.value);
        check_and_fill(arr, frontCoord.row - 1, frontCoord.col, frontCoord.value);
        check_and_fill(arr, frontCoord.row, frontCoord.col + 1, frontCoord.value);
        check_and_fill(arr, frontCoord.row, frontCoord.col - 1, frontCoord.value);
    }
}

void init_flood_start(std::vector<std::vector<int>>& arr, int row_, int col_, int back_) {
    int count_ = 0;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            arr[i][j] = -1;
            if (back_ == 2 && maze.cells[i][j].visited == false) {
                arr[i][j] = 255;
                maze.cells[i][j].dead = true;
            }
        }
    }
    if (back_ != 1) {
#if GOAL_4CELLS == 1
        coord point2 = {row_ + 1, col_, count_};
        myQueue.push(point2);
        arr[row_ + 1][col_] = 0;
        coord point3 = {row_, col_ + 1, count_};
        myQueue.push(point3);
        arr[row_][col_ + 1] = 0;
        coord point4 = {row_ + 1, col_ + 1, count_};
        myQueue.push(point4);
        arr[row_ + 1][col_ + 1] = 0;
#endif
    }
    coord point = {row_, col_, count_};
    myQueue.push(point);
    arr[row_][col_] = 0;
    while (!myQueue.empty()) {
        coord frontCoord = myQueue.front();
        myQueue.pop();
        for (int i = 0; i < 4; ++i) {
            int newRow = frontCoord.row + dy[i];
            int newCol = frontCoord.col + dx[i];
            bool check_ = maze.cells[frontCoord.row][frontCoord.col].walls[i];
            if (!check_) check_and_fill(arr, newRow, newCol, frontCoord.value);
        }
    }
}

void update_wall_debug(std::vector<std::vector<int>>& arr) {
    char dir;
    bool clear_ = 0;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            std::string value = std::to_string(arr[i][j]);
            for (int k = 0; k < 4; k++) {
                clear_ = maze.cells[i][j].walls[k];
                if (k == 0) dir = 'n';
                else if (k == 1) dir = 's';
                else if (k == 2) dir = 'w';
                else dir = 'e';
                if (clear_) API::setWall(i, j, dir);
                else API::clearWall(i, j, dir);
            }
            if (maze.cells[i][j].dead == true) {
                API::setText(i, j, "Dead");
                API::setColor(i, j, 'r');
            } else if (maze.cells[i][j].visited == true) {
                API::setColor(i, j, 'g');
                API::setText(i, j, value);
            } else {
                API::clearColor(i, j);
                API::setText(i, j, value);
            }
        }
    }
}

bool check_wall_angle(cell_info cell, int& dir) {
    switch (cell.angle_update) {
        case 90: break;
        case 270:
            if (dir % 2 == 0) dir += 1;
            else dir -= 1;
            break;
        case 0:
            if (dir == 0 || dir == 1) dir += 2;
            else if (dir == 2) dir = 1;
            else dir = 0;
            break;
        case 180:
            if (dir == 2 || dir == 3) dir -= 2;
            else if (dir == 0) dir = 3;
            else dir = 2;
            break;
    }
    return cell.walls[dir];
}

cell_info cell_direction_adjust(cell_info cell) {
    cell_info cell_new = cell;
    for (int i = 0; i < 4; i++) {
        int ind = i;
        switch (cell.angle_update) {
            case 90: break;
            case 270:
                if (i % 2 == 0) ind += 1;
                else ind -= 1;
                break;
            case 0:
                if (i == 0 || i == 1) ind += 2;
                else if (i == 2) ind = 1;
                else ind = 0;
                break;
            case 180:
                if (i == 2 || i == 3) ind -= 2;
                else if (i == 0) ind = 3;
                else ind = 2;
                break;
        }
        cell_new.walls[i] = cell.walls[ind];
    }
    return cell_new;
}

void go_to_cell(int& angle_now, int dir) {
    switch (dir) {
        case -1: log("not dir"); break;
        case UP:
            API::moveForward();
            break;
        case DOWN:
            angle_now -= 180;
            API::turnRight();
            API::turnRight();
            API::moveForward();
            break;
        case LEFT:
            angle_now += 90;
            API::turnLeft();
            API::moveForward();
            break;
        case RIGHT:
            angle_now -= 90;
            API::turnRight();
            API::moveForward();
            break;
        default: break;
    }
    angle_now = angle_now % 360;
    if (angle_now < 0) angle_now += 360;
}

coord get_min_neighbour(cell_info cell_wall, coord cur, std::vector<std::vector<int>>& arr, bool change_ = 0) {
    int min_neightbor = 255;
    coord next_step;
    next_step.value = -1;
    int ind;
    for (int dir = 0; dir < 4; ++dir) {
        int newRow = cur.row + dy[dir];
        int newCol = cur.col + dx[dir];
        ind = dir;
        bool check_ = cell_wall.walls[dir];
        if (change_) check_ = check_wall_angle(cell_wall, ind);
        if (isValid(newRow, newCol) && !check_) {
            if (arr[newRow][newCol] <= min_neightbor) {
                min_neightbor = arr[newRow][newCol];
                next_step.row = newRow;
                next_step.col = newCol;
                next_step.value = ind;
            }
        }
    }
    return next_step;
}

void flood(std::stack<coord>& stack_flood, std::vector<std::vector<int>>& arr) {
    coord cur_stack;
    coord next_step;
    while (!stack_flood.empty()) {
        cur_stack = stack_flood.top();
        stack_flood.pop();
        bool check_;
        next_step = get_min_neighbour(maze.cells[cur_stack.row][cur_stack.col], cur_stack, arr);
        int min_neightbor = arr[next_step.row][next_step.col];
        if (arr[cur_stack.row][cur_stack.col] - 1 != min_neightbor) {
            for (int i = 0; i < 4; i++) {
                coord cur_add;
                cur_add.row = cur_stack.row + dy[i];
                cur_add.col = cur_stack.col + dx[i];
                check_ = maze.cells[cur_stack.row][cur_stack.col].walls[i];
                if (isValid(cur_add.row, cur_add.col) && arr[cur_add.row][cur_add.col] != 0 && !check_) {
                    stack_flood.push(cur_add);
                }
            }
            if (arr[cur_stack.row][cur_stack.col] != 0)
                arr[cur_stack.row][cur_stack.col] = min_neightbor + 1;
        }
    }
}

cell_info update_walls(int angle_now, int row, int col) {
    cell_info new_cell;
    new_cell.angle_update = angle_now;
    new_cell.walls[UP] = API::wallFront();
    new_cell.walls[DOWN] = 0;
    new_cell.walls[LEFT] = API::wallLeft();
    new_cell.walls[RIGHT] = API::wallRight();
    new_cell.dead = 0;
    new_cell.visited = 1;
    maze.cells[row][col] = cell_direction_adjust(new_cell);
    if (new_cell.walls[UP] == 1 && new_cell.walls[LEFT] == 1 && new_cell.walls[RIGHT] == 1 && row != 0 && col != 0) {
        log("dead");
        maze.cells[row][col].dead = 1;
    }
    for (int i = 0; i < 4; i++) {
        int newRow = row + dy[i];
        int newCol = col + dx[i];
        if (isValid(newRow, newCol)) {
            if (i == UP) maze.cells[newRow][newCol].walls[DOWN] = maze.cells[row][col].walls[UP];
            else if (i == LEFT) maze.cells[newRow][newCol].walls[RIGHT] = maze.cells[row][col].walls[LEFT];
            else if (i == RIGHT) maze.cells[newRow][newCol].walls[LEFT] = maze.cells[row][col].walls[RIGHT];
        }
    }
    return new_cell;
}

coord floodfill(coord start, coord dest, std::vector<std::vector<int>>& arr, int& angle_now) {
    std::queue<coord> path_queue;
    path_queue.push(start);
    coord cur = start;
    cell_info new_cell;
    std::stack<coord> stack_flood;
    stack_flood.push(start);
    int path_distance_value_find = 0;
    coord next_step;
    while (1) {
        if (!path_queue.empty()) {
            cur = path_queue.front();
            new_cell = update_walls(angle_now, cur.row, cur.col);
            if (arr[cur.row][cur.col] == arr[dest.row][dest.col]) break;
            flood(stack_flood, arr);
            path_queue.pop();
            next_step = get_min_neighbour(new_cell, cur, arr, 1);
            path_queue.push(next_step);
            stack_flood.push(next_step);
            go_to_cell(angle_now, next_step.value);
            update_wall_debug(arr);
            path_distance_value_find++;
        } else {
            log("empty Queue- break");
            break;
        }
    }
    while (!path_queue.empty()) path_queue.pop();
    std::cerr << "total_cost:" << path_distance_value_find << std::endl;
    coord p_return = {next_step.row, next_step.col, 0};
    return p_return;
}

void init_maze() {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            maze.cells[i][j].visited = 0;
            maze.cells[i][j].angle_update = 90;
            maze.cells[i][j].dead = 0;
            for (int k = 0; k < 4; k++) maze.cells[i][j].walls[k] = 0;
        }
    }
}

void go_to_cell_shorted(int& angle, int dir) {
    int new_dir = dir;
    switch (angle) {
        case 90: break;
        case 270:
            if (dir % 2 == 0) new_dir += 1;
            else new_dir -= 1;
            break;
        case 0:
            if (dir == 0 || dir == 1) new_dir += 2;
            else if (dir == 2) new_dir = 1;
            else new_dir = 0;
            break;
        case 180:
            if (dir == 2 || dir == 3) new_dir -= 2;
            else if (dir == 0) new_dir = 3;
            else new_dir = 2;
            break;
    }
    go_to_cell(angle, new_dir);
}

void shorted_path_go(std::vector<std::vector<int>>& arr, int angle_now, coord start, coord dest) {
    coord cur = start;
    int angle = angle_now;

    // ── Phase 1: Visualize shortest path in yellow ──
    coord viz_cur = start;
    for (int i = 0; i < arr[start.row][start.col]; i++) {
        int next_dir = -1;
        int save_row, save_col;
        for (int dir = 0; dir < 4; ++dir) {
            int newRow = viz_cur.row + dy[dir];
            int newCol = viz_cur.col + dx[dir];
            bool check_ = maze.cells[viz_cur.row][viz_cur.col].walls[dir];
            if (isValid(newRow, newCol) && !check_) {
                if (arr[newRow][newCol] < arr[viz_cur.row][viz_cur.col]) {
                    next_dir = dir;
                    save_row = newRow;
                    save_col = newCol;
                }
            }
        }
        if (next_dir != -1) {
            API::setColor(save_row, save_col, 'y');
            API::setText(save_row, save_col, std::to_string(arr[save_row][save_col]));
            viz_cur.row = save_row;
            viz_cur.col = save_col;
        } else break;
    }

    // ── Phase 2: Wait ──
    std::cerr << "Shortest path shown. Speed run starts in "
              << SPEEDRUN_WAIT << " seconds..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(SPEEDRUN_WAIT));

    // ── Phase 3: Speed run ──
    for (int i = 0; i < arr[start.row][start.col]; i++) {
        int next_dir = -1;
        int save_row, save_col;
        for (int dir = 0; dir < 4; ++dir) {
            int newRow = cur.row + dy[dir];
            int newCol = cur.col + dx[dir];
            bool check_ = maze.cells[cur.row][cur.col].walls[dir];
            if (isValid(newRow, newCol) && !check_) {
                if (arr[newRow][newCol] < arr[cur.row][cur.col]) {
                    next_dir = dir;
                    save_row = newRow;
                    save_col = newCol;
                }
            }
        }
        if (next_dir != -1) {
            go_to_cell_shorted(angle, next_dir);
            cur.row = save_row;
            cur.col = save_col;
        } else {
            log("Path blocked during speed run!");
            break;
        }
    }
    log("Speed run complete!");
}

int main(int argc, char* argv[])
{
    std::vector<std::vector<int>> arr;
    init_arr(arr, rows, cols);

#if GOAL_4CELLS == 1
    init_flood(arr, GOAL4_ROW, GOAL4_COL);
#else
    init_flood(arr, GOAL_ROW, GOAL_COL);
#endif

    init_maze();

    coord start = {START_ROW, START_COL, arr[START_ROW][START_COL]};

#if GOAL_4CELLS == 1
    coord dest = {GOAL4_ROW, GOAL4_COL, arr[GOAL4_ROW][GOAL4_COL]};
    API::setColor(GOAL4_ROW,     GOAL4_COL,     'r');
    API::setColor(GOAL4_ROW + 1, GOAL4_COL,     'r');
    API::setColor(GOAL4_ROW,     GOAL4_COL + 1, 'r');
    API::setColor(GOAL4_ROW + 1, GOAL4_COL + 1, 'r');
    API::setText(GOAL4_ROW, GOAL4_COL, "Goal");
#else
    coord dest = {GOAL_ROW, GOAL_COL, arr[GOAL_ROW][GOAL_COL]};
    API::setColor(GOAL_ROW, GOAL_COL, 'r');
    API::setText(GOAL_ROW, GOAL_COL, "Goal");
#endif

    API::setColor(START_ROW, START_COL, 'r');
    API::setText(START_ROW, START_COL, "Start");

    int angle_now = 90;
    coord new_coord;

#if AUTO_CONVERGE == 1
    std::cerr << "AUTOMATIC MODE running..." << std::endl;
    int prev_cost = -1;
    int stable_count = 0;

    for (int m = 0; m < MAX_ITERATIONS; m++)
    {
        std::cerr << "=== Iteration " << m + 1 << " ===" << std::endl;
        new_coord = floodfill(start, dest, arr, angle_now);
        init_flood_start(arr, START_ROW, START_COL, 1);
        std::cerr << "done2" << std::endl;
        new_coord = floodfill(new_coord, start, arr, angle_now);

#if GOAL_4CELLS == 1
        init_flood_start(arr, GOAL4_ROW, GOAL4_COL, 2);
#else
        init_flood_start(arr, GOAL_ROW, GOAL_COL, 2);
#endif

        int current_cost = arr[START_ROW][START_COL];
        std::cerr << "path_length:" << current_cost << std::endl;

        if (current_cost == prev_cost) {
            stable_count++;
            if (stable_count >= STABLE_NEEDED) {
                std::cerr << "Converged at iteration " << m+1 << "!" << std::endl;
                break;
            }
        } else {
            stable_count = 0;
        }
        prev_cost = current_cost;
    }

#else
    std::cerr << "FIXED MODE: running " << FIXED_ITERATIONS << " iterations..." << std::endl;

    for (int m = 0; m < FIXED_ITERATIONS; m++)
    {
        std::cerr << "=== Iteration " << m + 1 << " ===" << std::endl;
        new_coord = floodfill(start, dest, arr, angle_now);
        init_flood_start(arr, START_ROW, START_COL, 1);
        std::cerr << "done2" << std::endl;
        new_coord = floodfill(new_coord, start, arr, angle_now);

#if GOAL_4CELLS == 1
        init_flood_start(arr, GOAL4_ROW, GOAL4_COL, 2);
#else
        init_flood_start(arr, GOAL_ROW, GOAL_COL, 2);
#endif

        std::cerr << "path_length:" << arr[START_ROW][START_COL] << std::endl;
    }
#endif

    shorted_path_go(arr, angle_now, new_coord, dest);
}