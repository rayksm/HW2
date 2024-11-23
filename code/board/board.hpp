#ifndef __BOARD__
#define __BOARD__ 1

#define RED 0
#define BLUE 1

#define PIECE_NUM 6

typedef struct _board
{
    // all captured: piece_bits becomes 0
    unsigned char piece_bits[2];
    int board[25];
    int piece_position[2][PIECE_NUM];
    // blank is -1
    
    char moves[PIECE_NUM][2];
    int move_count;
    char moving_color;
    char dice;
    
    int depth;
    int parent_id;
    
    int this_id;
    
    int child_id_start;
    
    
    int totaln;
    long double csqrtlogtn;
    long double csqrttn;
    
    int wins;
    long double winrate;

    int be_pruned = 0;
    int child_exist_num = 0;

    void init_with_piecepos(int input_piecepos[2][6], char input_color);
    void move(int id_with_dice);
    void generate_moves();
    bool check_winner();
    void print_board();

    // not basic functions, written in decide.cpp
    bool simulate();
    int decide();
    int first_move_decide_dice();

    void clear(){
        //nchild = 0;
        totaln = 0;
        wins = 0;
        move_count = 0;

    }
} Board;
#endif