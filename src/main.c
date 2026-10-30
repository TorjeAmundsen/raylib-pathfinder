#include "raylib.h"
#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>

#define GRID_COLUMNS    51
#define GRID_ROWS       27
#define NODE_SIZE       20
#define GRID_MARGIN     8
#define GRID_GAP        1
#define GRID_PITCH      (NODE_SIZE + GRID_GAP)
#define PANEL_HEIGHT    40
#define FONT_SIZE       10
#define WINDOW_WIDTH    (GRID_COLUMNS * NODE_SIZE + (GRID_COLUMNS - 1) * GRID_GAP + GRID_MARGIN * 2)
#define WINDOW_HEIGHT   (GRID_ROWS * NODE_SIZE + (GRID_ROWS - 1) * GRID_GAP + GRID_MARGIN * 2 + PANEL_HEIGHT)
#define NODE_COUNT      (GRID_COLUMNS * GRID_ROWS)
#define HEAP_CAPACITY   (NODE_COUNT * 4 + 1)
#define TARGET_FPS      60

#define DEFAULT_NODE_COLOR  ((Color){  40,  45,  55, 255 })
#define START_NODE_COLOR    ((Color){  40, 200,  60, 255 })
#define GOAL_NODE_COLOR     ((Color){ 130,  80, 240, 255 })
#define WALL_NODE_COLOR     ((Color){   0,   0,   0, 255 })
#define QUEUED_NODE_COLOR   ((Color){ 255, 215,  80, 255 })
#define PATH_LINE_COLOR     ((Color){ 255, 255,   0, 255 })
#define PATH_LINE_THICKNESS 6.0f

typedef struct {
    bool is_wall;
    int  distance_from_start;
    int  previous_node_index;
    bool is_queued;
    bool is_finalized;
    bool is_discovered;
} Node;

typedef struct {
    int priority;
    int heuristic;
    int insertion_order;
    int node_index;
} HeapEntry;

typedef enum {
    ALGORITHM_DIJKSTRA,
    ALGORITHM_A_STAR,
} Algorithm;

typedef enum {
    MAZE_RECURSIVE_DIVISION,
    MAZE_RANDOMIZED_PRIMS,
} MazeAlgorithm;

typedef enum {
    SEARCH_IDLE,
    SEARCH_RUNNING,
    SEARCH_FOUND,
    SEARCH_NO_PATH,
} SearchState;

typedef enum {
    DRAG_STATE_NONE,
    DRAG_STATE_PAINTING_WALLS,
    DRAG_STATE_START_NODE,
    DRAG_STATE_GOAL_NODE,
} DragState;

typedef enum {
    PRIMS_CELL,
    PRIMS_WALL,
    PRIMS_PILLAR,
} PrimsNodeType;

static const char* algorithm_names[] = {
    "Dijkstra", "A*"
};

static const char* maze_algorithm_names[] = {
    "Recursive division", "Randomized Prim's algorithm"
};

static Algorithm     current_algorithm = ALGORITHM_DIJKSTRA;
static MazeAlgorithm current_maze_algorithm = MAZE_RECURSIVE_DIVISION;
static Node          nodes[NODE_COUNT];
static HeapEntry     min_heap[HEAP_CAPACITY];
static int           min_heap_size = 0;
static int           insertion_counter = 0;
static int           start_node_index = 0;
static int           goal_node_index = NODE_COUNT - 1;
static int           queued_node_count = 0;
static int           path_node_indexes[NODE_COUNT];
static int           path_length = 0;
static int           path_revealed_count;
static SearchState   search_state = SEARCH_IDLE;
static bool          is_paused = false;
static int           steps_per_frame = 4;
static int           paint_buffer[NODE_COUNT];
static int           paint_buffer_size = 0;
static int           paint_buffer_index = 0;
static bool          is_in_cell_buffer[NODE_COUNT];

static const int    cell_column_offsets[4]     = {  0, 2, 0,-2 };
static const int    cell_row_offsets[4]        = { -2, 0, 2, 0 };

static const int    cardinal_column_offsets[4] = {  0, 1, 0,-1 };
static const int    cardinal_row_offsets[4]    = { -1, 0, 1, 0 };

static int node_index(int col, int row) {
    return row * GRID_COLUMNS + col;
}

static int node_column(int node_index) {
    return node_index % GRID_COLUMNS;
}

static int node_row(int node_index) {
    return node_index / GRID_COLUMNS;
}

static bool is_out_of_bounds(int column, int row) {
    return column < 0 || column >= GRID_COLUMNS || row < 0 || row >= GRID_ROWS;
}

static int manhattan_distance_to_goal(int node_index) {
    int column_difference = node_column(node_index) - node_column(goal_node_index);
    int row_difference = node_row(node_index) - node_row(goal_node_index);
    if (column_difference < 0) {
        column_difference = -column_difference;
    }
    if (row_difference < 0) {
        row_difference = -row_difference;
    }

    return column_difference + row_difference;
}

static void swap_entries(int first_position, int second_position) {
    HeapEntry temp = min_heap[first_position];
    min_heap[first_position] = min_heap[second_position];
    min_heap[second_position] = temp;
}

static bool entry_comes_before(HeapEntry first, HeapEntry second) {
    if (first.priority != second.priority) {
        return first.priority < second.priority;
    }

    if (first.heuristic != second.heuristic) {
        return first.heuristic < second.heuristic;
    }

    return first.insertion_order < second.insertion_order;
}

static void min_heap_push(int node_index, int priority, int heuristic) {
    int position = min_heap_size++;
    min_heap[position] = (HeapEntry){ priority, heuristic, insertion_counter++, node_index };

    while (position > 0) {
        int parent_position = (position - 1) / 2;

        if (!entry_comes_before(min_heap[position], min_heap[parent_position])) {
            break;
        }

        swap_entries(position, parent_position);
        position = parent_position;
    }
}

static HeapEntry min_heap_pop(void) {
    HeapEntry best = min_heap[0];
    min_heap[0] = min_heap[--min_heap_size];

    int position = 0;
    while (1) {
        int left_child = 2 * position + 1;
        int right_child = left_child + 1;
        int winner = position;

        if (left_child < min_heap_size && entry_comes_before(min_heap[left_child], min_heap[winner])) {
            winner = left_child;
        }
        if (right_child < min_heap_size && entry_comes_before(min_heap[right_child], min_heap[winner])) {
            winner = right_child;
        }
        if (winner == position) {
            break;
        }
        swap_entries(position, winner);
        position = winner;
    }

    return best;
}

static void build_path(void) {
    path_length = 0;
    for (int i = goal_node_index; i != -1; i = nodes[i].previous_node_index) {
        path_node_indexes[path_length++] = i;
    }

    for (int left = 0, right = path_length - 1; left < right; left++, right--) {
        int temp = path_node_indexes[left];
        path_node_indexes[left] = path_node_indexes[right];
        path_node_indexes[right] = temp;
    }

    path_revealed_count = 0;
}

static void find_path(bool step_once) {
    if (search_state == SEARCH_FOUND) {
        return;
    }

    while (min_heap_size > 0) {
        HeapEntry entry = min_heap_pop();
        Node* current_node = &nodes[entry.node_index];

        if (current_node->is_finalized) {
            continue;
        }

        current_node->is_finalized = true;
        queued_node_count++;

        if (entry.node_index == goal_node_index) {
            build_path();
            search_state = SEARCH_FOUND;
            return;
        }

        int column = node_column(entry.node_index);
        int row = node_row(entry.node_index);

        for (int direction = 0; direction < 4; direction++) {
            int neighbor_column = column + cardinal_column_offsets[direction];
            int neighbor_row = row + cardinal_row_offsets[direction];

            if (neighbor_column < 0 || neighbor_column >= GRID_COLUMNS) {
                continue;
            }
            if (neighbor_row < 0 || neighbor_row >= GRID_ROWS) {
                continue;
            }

            int neighbor_index = node_index(neighbor_column, neighbor_row);
            Node* neighbor_node = &nodes[neighbor_index];
            if (neighbor_node->is_wall || neighbor_node->is_finalized) {
                continue;
            }

            int new_distance = current_node->distance_from_start + 1;
            if (new_distance < neighbor_node->distance_from_start) {
                neighbor_node->distance_from_start = new_distance;
                neighbor_node->previous_node_index = entry.node_index;
                neighbor_node->is_discovered = true;

                int heuristic = current_algorithm == ALGORITHM_A_STAR ? manhattan_distance_to_goal(neighbor_index) : 0;
                int priority = new_distance;
                if (current_algorithm == ALGORITHM_A_STAR) {
                    priority += heuristic;
                }

                min_heap_push(neighbor_index, priority, heuristic);
            }
        }

        if (step_once) {
            return;
        }
    }
    search_state = SEARCH_NO_PATH;
}

static void reset_search(void) {
    for (int i = 0; i < NODE_COUNT; i++) {
        nodes[i].distance_from_start = INT_MAX;
        nodes[i].previous_node_index = -1;
        nodes[i].is_discovered = false;
        nodes[i].is_finalized = false;
    }

    min_heap_size = 0;
    insertion_counter = 0;
    queued_node_count = 0;
    path_length = 0;
    path_revealed_count = 0;
    search_state = SEARCH_IDLE;
}

static void begin_search(void) {
    reset_search();

    nodes[start_node_index].distance_from_start = 0;
    nodes[start_node_index].is_discovered = true;

    int heuristic = current_algorithm == ALGORITHM_A_STAR ? manhattan_distance_to_goal(start_node_index) : 0;
    int priority = current_algorithm == ALGORITHM_A_STAR ? heuristic : 0;

    min_heap_push(start_node_index, priority, heuristic);

    search_state = SEARCH_RUNNING;
}

static void clear_grid(void) {
    paint_buffer_size = 0;
    paint_buffer_index = 0;
    for (int i = 0; i < NODE_COUNT; i++) {
        nodes[i].is_wall = false;
    }
    reset_search();
}

int get_random_num(int from, int to) {
    return from + rand() % (to - from + 1);
}

bool get_is_horizontal(int width, int height) {
    if (width < height) {
        return true;
    }

    if (width > height) {
        return false;
    }

    return rand() % 2;
}

static void fill_nodes_with_walls(void) {
    for (int i = 0; i < NODE_COUNT; i++) {
        nodes[i].is_wall = true;
    }
}

static bool is_prims_cell(int index) {
    bool column_is_odd = node_column(index) % 2;
    bool row_is_odd = node_row(index) % 2;

    return !column_is_odd && !row_is_odd;
}

int get_random_nearest_prims_cell_index(int index) {
    static const int column_offsets[8] = {  0, 1, 0,-1, 1, 1,-1,-1 };
    static const int row_offsets[8]    = { -1, 0, 1, 0,-1, 1, 1,-1 };

    int column = node_column(index);
    int row = node_row(index);

    int cells[4];
    int cells_size = 0;

    for (int i = 0; i < 8; i++) {
        int current_column = column + column_offsets[i];
        int current_row = row + row_offsets[i];

        if (is_out_of_bounds(current_column, current_row)) {
            continue;
        }

        int current_index = node_index(current_column, current_row);
        bool is_cell = is_prims_cell(current_index);

        if (is_cell) {
            cells[cells_size++] = current_index;
        }
    }

    return cells[get_random_num(0, cells_size - 1)];
}

static void add_walled_neighbors_to_cell_buffer(int cell_node_index, int* cell_buffer, int* cell_buffer_size) {
    int column = node_column(cell_node_index);
    int row = node_row(cell_node_index);

    for (int i = 0; i < 4; i++) {
        int neighbor_column = column + cell_column_offsets[i];
        int neighbor_row = row + cell_row_offsets[i];

        if (is_out_of_bounds(neighbor_column, neighbor_row)) {
            continue;
        }

        int neighbor_index = node_index(neighbor_column, neighbor_row);

        if (is_in_cell_buffer[neighbor_index] || !nodes[neighbor_index].is_wall) {
            continue;
        }

        is_in_cell_buffer[neighbor_index] = true;
        cell_buffer[(*cell_buffer_size)++] = neighbor_index;
    }
}

static void connect_node_to_maze(int index) {
    nodes[index].is_wall = false;
    paint_buffer[paint_buffer_size++] = index;

    int column = node_column(index);
    int row = node_row(index);

    int walled_neighbors[4];
    int walled_neighbor_size = 0;

    for (int i = 0; i < 4; i++) {
        int neighbor_column = column + cardinal_column_offsets[i];
        int neighbor_row = row + cardinal_row_offsets[i];

        if (is_out_of_bounds(neighbor_column, neighbor_row)) {
            continue;
        }

        int neighbor_index = node_index(neighbor_column, neighbor_row);

        if (!nodes[neighbor_index].is_wall) {
            return;
        }

        walled_neighbors[walled_neighbor_size++] = neighbor_index;
    }

    int random_neighbor_index = walled_neighbors[get_random_num(0, walled_neighbor_size - 1)];
    nodes[random_neighbor_index].is_wall = false;
    paint_buffer[paint_buffer_size++] = random_neighbor_index;
}

static int get_random_maze_neighbor_index(int cell_node_index) {
    int column = node_column(cell_node_index);
    int row = node_row(cell_node_index);

    int maze_neighbors[4];
    int maze_neighbor_size = 0;

    for (int i = 0; i < 4; i++) {
        int neighbor_column = column + cell_column_offsets[i];
        int neighbor_row = row + cell_row_offsets[i];

        if (is_out_of_bounds(neighbor_column, neighbor_row)) {
            continue;
        }

        int neighbor_index = node_index(neighbor_column, neighbor_row);

        if (!nodes[neighbor_index].is_wall) {
            maze_neighbors[maze_neighbor_size++] = neighbor_index;
        }
    }

    return maze_neighbors[get_random_num(0, maze_neighbor_size - 1)];
}

static void randomized_prims_maze(){
    fill_nodes_with_walls();
    for (int i = 0; i < NODE_COUNT; i++) {
        is_in_cell_buffer[i] = 0;
    }

    int cells[NODE_COUNT];
    int cell_buffer_size = 0;

    int current_cell_index;
    if (is_prims_cell(start_node_index)) {
        current_cell_index = start_node_index;
    } else {
        current_cell_index = get_random_nearest_prims_cell_index(start_node_index);
    }

    nodes[current_cell_index].is_wall = false;
    paint_buffer[paint_buffer_size++] = current_cell_index;
    add_walled_neighbors_to_cell_buffer(current_cell_index, cells, &cell_buffer_size);

    while (cell_buffer_size > 0) {
        int cell_index = get_random_num(0, cell_buffer_size - 1);
        int cell_node_index = cells[cell_index];
        cells[cell_index] = cells[--cell_buffer_size];

        int maze_neighbor_index = get_random_maze_neighbor_index(cell_node_index);

        int dividing_wall_column = (node_column(cell_node_index) + node_column(maze_neighbor_index)) / 2;
        int dividing_wall_row = (node_row(cell_node_index) + node_row(maze_neighbor_index)) / 2;

        nodes[node_index(dividing_wall_column, dividing_wall_row)].is_wall = false;
        nodes[cell_node_index].is_wall = false;

        paint_buffer[paint_buffer_size++] = node_index(dividing_wall_column, dividing_wall_row);
        paint_buffer[paint_buffer_size++] = cell_node_index;

        add_walled_neighbors_to_cell_buffer(cell_node_index, cells, &cell_buffer_size);
    }

    connect_node_to_maze(start_node_index);
    connect_node_to_maze(goal_node_index);

    fill_nodes_with_walls();
}

static void recursive_division_maze(int column, int row, int width, int height, bool is_horizontal) {
    if (width < 3 || height < 3) {
        return;
    }

    int wall_column, wall_row, hole_column, hole_row, wall_length;

    if (is_horizontal) {
        wall_column = column;
        wall_row = row + get_random_num(1, height / 2) * 2 - 1;
        hole_column = wall_column + get_random_num(0, width / 2) * 2;
        wall_length = width;
    } else {
        wall_column = column + get_random_num(1, width / 2) * 2 - 1;
        wall_row = row;
        hole_row = wall_row + get_random_num(0, height / 2) * 2;
        wall_length = height;
    }

    if (is_horizontal) {
        for (int i = 0; i < wall_length; i++) {
            if (wall_column + i != hole_column) {
                paint_buffer[paint_buffer_size++] = node_index(wall_column + i, wall_row);
            }
        }
    } else {
        for (int i = 0; i < wall_length; i++) {
            if (wall_row + i != hole_row) {
                paint_buffer[paint_buffer_size++] = node_index(wall_column, wall_row + i);
            }
        }
    }

    if (is_horizontal) {
        recursive_division_maze(
            column,
            row,
            width,
            abs(wall_row - row),
            get_is_horizontal(width, abs(wall_row - row)));

        recursive_division_maze(
            column,
            wall_row + 1,
            width,
            height - (wall_row - row) - 1,
            get_is_horizontal(width,height - (wall_row - row) - 1));
    } else {
        recursive_division_maze(
            column,
            row,
            abs(wall_column - column),
            height,
            get_is_horizontal(abs(wall_column - column), height));

        recursive_division_maze(
            wall_column + 1,
            row,
            width - (wall_column - column) - 1,
            height,
            get_is_horizontal(width - (wall_column - column) - 1, height));
    }
}

static Vector2 get_node_center(int node_index) {
    return (Vector2){
        GRID_MARGIN + node_column(node_index) * GRID_PITCH + NODE_SIZE / 2.0f,
        PANEL_HEIGHT + GRID_MARGIN + node_row(node_index) * GRID_PITCH + NODE_SIZE / 2.0f,
    };
}

static Color get_node_color(int node_index) {
    const Node* node = &nodes[node_index];

    if (node_index == start_node_index) {
        return START_NODE_COLOR;
    }

    if (node_index == goal_node_index) {
        return GOAL_NODE_COLOR;
    }

    if (node->is_wall) {
        return WALL_NODE_COLOR;
    }

    if (node->is_finalized) {
        float hue = fmodf(190.0f + (node->distance_from_start / 2.0f), 360.0f);
        return ColorFromHSV(hue, 0.67f, 0.8f);
    }

    if (node->is_discovered) {
        return QUEUED_NODE_COLOR;
    }

    return DEFAULT_NODE_COLOR;
}

static void draw_grid_nodes(void) {
    for (int i = 0; i < NODE_COUNT; i++) {
        DrawRectangle(
            node_column(i) * GRID_PITCH + GRID_MARGIN,
            node_row(i) * GRID_PITCH + GRID_MARGIN + PANEL_HEIGHT,
            NODE_SIZE,
            NODE_SIZE,
            get_node_color(i)
        );
    }
}

static void draw_path(void) {
    for (int i = 1; i < path_revealed_count; i++) {
        Vector2 from = get_node_center(path_node_indexes[i - 1]);
        Vector2 to = get_node_center(path_node_indexes[i]);

        DrawLineEx(from, to, PATH_LINE_THICKNESS, PATH_LINE_COLOR);
        DrawCircleV(to, PATH_LINE_THICKNESS / 2.0f, PATH_LINE_COLOR);
    }
}

static void draw_panel(void) {
    static const char* state_texts[] = {
        "Idle",
        "Paused",
        "Searching",
        "Path found",
        "No path"
    };

    int state_text_index = 0;

    if (search_state == SEARCH_RUNNING) {
        state_text_index = is_paused ? 1 : 2;
    }
    if (search_state == SEARCH_FOUND) {
        state_text_index = 3;
    }
    if (search_state == SEARCH_NO_PATH) {
        state_text_index = 4;
    }
    DrawText(TextFormat(
        "Search algorithm: %s  |  Maze algorithm: %s  |  %s  |  searched: %d  |  queue: %d  |  speed: %d/frame",
        algorithm_names[current_algorithm],
        maze_algorithm_names[current_maze_algorithm],
        state_texts[state_text_index],
        queued_node_count,
        min_heap_size,
        steps_per_frame),
    10, 10, FONT_SIZE, RAYWHITE);

    if (search_state == SEARCH_FOUND) {
        DrawText(TextFormat("Found path: %d steps", path_length - 1), 10, 28, 10, (Color){ 255, 214, 90, 255 });
    }
}

static void step_search_frame(void) {
    for (int i = 0; i < steps_per_frame; i++) {
        find_path(true);
    }
}

static void instant_search_and_display_path(void) {
    reset_search();
    begin_search();
    find_path(false);
    path_revealed_count = path_length;
}

static void paint_wall_line(int start_column, int start_row, int end_column, int end_row, bool fill) {
    int horizontal_distance = abs(end_column - start_column);
    int vertical_distance = abs(end_row - start_row);

    int horizontal_direction = (start_column < end_column) ? 1 : -1;
    int vertical_direction = (start_row < end_row) ? 1 : -1;

    int error = horizontal_distance - vertical_distance;

    while (1) {
        int current_node_index = node_index(start_column, start_row);
        if (current_node_index != start_node_index && current_node_index != goal_node_index) {
            nodes[node_index(start_column, start_row)].is_wall = fill;
        }

        if (start_column == end_column && start_row == end_row) {
            break;
        }

        int error_2x = error * 2;

        if (error_2x > -vertical_direction) {
            error -= vertical_distance;
            start_column += horizontal_direction;
        }

        if (error_2x < horizontal_distance) {
            error += horizontal_distance;
            start_row += vertical_direction;
        }
    }
}

static void generate_maze(void) {
    clear_grid();
    switch (current_maze_algorithm) {
        case MAZE_RECURSIVE_DIVISION:
            recursive_division_maze(0, 0, GRID_COLUMNS, GRID_ROWS, get_is_horizontal(GRID_COLUMNS, GRID_ROWS));
            break;
        case MAZE_RANDOMIZED_PRIMS:
            randomized_prims_maze();
    }
}

static void paint_maze_frame(void) {
    nodes[paint_buffer[paint_buffer_index++]].is_wall = current_maze_algorithm == MAZE_RECURSIVE_DIVISION;
    if (search_state == SEARCH_FOUND || search_state == SEARCH_NO_PATH) {
        instant_search_and_display_path();
    }
}

static void handle_input(void) {
    static DragState drag_state  = DRAG_STATE_NONE;
    static int last_frame_column = -1;
    static int last_frame_row    = -1;

    Vector2 mouse_position = GetMousePosition();
    int column = (int)floorf((mouse_position.x - GRID_MARGIN) / GRID_PITCH);
    int row = (int)floorf((mouse_position.y - (PANEL_HEIGHT + GRID_MARGIN)) / GRID_PITCH);

    bool is_mouse_inbounds = column >= 0 && column < GRID_COLUMNS && row >= 0 && row < GRID_ROWS;
    bool is_shift_held = IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT);

    if (is_mouse_inbounds) {
        int hovered_index = node_index(column, row);
        bool is_hovering_start_or_end = hovered_index == start_node_index || hovered_index == goal_node_index;
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            if (hovered_index == start_node_index) {
                drag_state = DRAG_STATE_START_NODE;
            } else if (hovered_index == goal_node_index) {
                drag_state = DRAG_STATE_GOAL_NODE;
            } else {
                drag_state = DRAG_STATE_PAINTING_WALLS;
            }
        }
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
            if (hovered_index != start_node_index && hovered_index != goal_node_index) {
                drag_state = DRAG_STATE_PAINTING_WALLS;
            }
        }
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            if (drag_state == DRAG_STATE_PAINTING_WALLS) {
                if (!is_hovering_start_or_end) {
                    paint_wall_line(
                        column,
                        row,
                        last_frame_column >= 0 ? last_frame_column : column,
                        last_frame_row >= 0 ? last_frame_row : row,
                        true
                    );
                    if (search_state == SEARCH_FOUND) {
                        instant_search_and_display_path();
                    }
                }
            } else if (drag_state == DRAG_STATE_START_NODE) {
                if (!nodes[hovered_index].is_wall && !is_hovering_start_or_end) {
                    start_node_index = hovered_index;
                    if (search_state == SEARCH_FOUND) {
                        instant_search_and_display_path();
                    }
                }
            } else if (drag_state == DRAG_STATE_GOAL_NODE) {
                if (!nodes[hovered_index].is_wall && !is_hovering_start_or_end) {
                    goal_node_index = hovered_index;
                    if (search_state == SEARCH_FOUND) {
                        instant_search_and_display_path();
                    }
                }
            }
        } else if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            if (drag_state == DRAG_STATE_PAINTING_WALLS) {
                if (!is_hovering_start_or_end) {
                    paint_wall_line(
                        column,
                        row,
                        last_frame_column >= 0 ? last_frame_column : column,
                        last_frame_row >= 0 ? last_frame_row : row,
                        false
                    );
                    if (search_state == SEARCH_FOUND) {
                        instant_search_and_display_path();
                    }
                }
            }
        }
    }

    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) || IsMouseButtonReleased(MOUSE_BUTTON_RIGHT)) {
        drag_state = DRAG_STATE_NONE;
    }

    if ((IsKeyPressed(KEY_S) || IsKeyPressed(KEY_SPACE)) && !(paint_buffer_size > 0 && paint_buffer_size > paint_buffer_index)) {
        if (is_paused) {
            if (search_state == SEARCH_IDLE) {
                begin_search();
            }
            if (is_shift_held) {
                step_search_frame();
            } else {
                find_path(true);
            }
        } else {
            reset_search();
            begin_search();
        }
    }

    if (IsKeyPressedRepeat(KEY_S) && is_paused) {
        if (is_shift_held) {
            step_search_frame();
        } else {
            find_path(true);
        }
    }

    if (IsKeyPressed(KEY_RIGHT) || IsKeyPressedRepeat(KEY_RIGHT)) {
        if (is_shift_held) {
            steps_per_frame += 4;
        } else {
            steps_per_frame++;
        }
    } else if (IsKeyPressed(KEY_LEFT) || IsKeyPressedRepeat(KEY_LEFT)) {
        if (is_shift_held) {
            steps_per_frame -= 4;
        } else {
            steps_per_frame--;
        }

        if (steps_per_frame < 1) {
            steps_per_frame = 1;
        }
    }

    if (IsKeyPressed(KEY_R)) {
        clear_grid();
    }

    if (IsKeyPressed(KEY_P)) {
        is_paused = !is_paused;
    }

    if (steps_per_frame < 1) {
        steps_per_frame = 1;
    }

    if (IsKeyPressed(KEY_ONE)) {
        if (is_shift_held) {
            clear_grid();
            current_maze_algorithm = MAZE_RECURSIVE_DIVISION;
        } else {
            reset_search();
            current_algorithm = ALGORITHM_DIJKSTRA;
        }
    }
    if (IsKeyPressed(KEY_TWO)) {
        if (is_shift_held) {
            clear_grid();
            current_maze_algorithm = MAZE_RANDOMIZED_PRIMS;
        } else {
            reset_search();
            current_algorithm = ALGORITHM_A_STAR;
        }
    }

    if (IsKeyPressed(KEY_G)) {
        if (search_state == SEARCH_FOUND || search_state == SEARCH_NO_PATH) {
            generate_maze();
            instant_search_and_display_path();
        }
        if (search_state == SEARCH_IDLE) {
            generate_maze();
        }
    }

    if (is_mouse_inbounds) {
        last_frame_column = column;
        last_frame_row    = row;
    } else {
        last_frame_column = -1;
        last_frame_row    = -1;
    }
}

int main(void) {
    srand(time(NULL));

    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Pathfinding Visualizer");

    SetTargetFPS(TARGET_FPS);

    while (!WindowShouldClose()) {
        if (search_state == SEARCH_RUNNING && !is_paused) {
            step_search_frame();
        }

        if (search_state == SEARCH_FOUND && path_revealed_count < path_length) {
            path_revealed_count++;
        }

        if (paint_buffer_size > 0 && paint_buffer_size > paint_buffer_index) {
            // TODO: Draw failed search path when no path found while painting
            // Do this by finding lowest radius n around goal node that has been searched,
            // find lowest travel distance node in n radius square around goal node,
            // and display path to this node in red.
            paint_maze_frame();
        }
        handle_input();
        BeginDrawing();
            ClearBackground(CLITERAL(Color){ 25, 25, 35, 255 });
            draw_grid_nodes();
            draw_path();
            draw_panel();
        EndDrawing();
    }

    CloseWindow();

    return 0;
}
