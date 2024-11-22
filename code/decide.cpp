// These are not basic function for board.
// write them in separate file makes code cleaner.
// I recommend write all your complicated algorithms in this file.
// Or sort them into even more files to make code even cleaner!

#include "board/board.hpp"
#include "math_lib/maths.hpp"
#include <stdio.h>
#include <random>
#include <math.h>
#include <cmath>
#include <time.h>
#define SIMULATION_BATCH_NUM 100
#define MAX_SIMULATION_COUNT 3000000

//  best para now
// SIMULATION_BATCH_NUM = 150
// 0.2 = 0

// init lots board
#define MaxBoard 200000
static Board allboard[MaxBoard];
int index_allboard = 0;
float visited = 0;
float UCB_rand[64];

// lower bound for exploration
#define Lowbound_explore 400

//float start_time, end_time;
struct timespec start, end;
float remain_time = 30.0, can_use;
const float timeratio = 1;

// MCS UCB version
// I use max sample number as the simulation limit.
// Recommend: Using wall clock time to precisely limit the time.
// Advanced: Using both max sample number and wall clock time to limit the simiulation time.

void re_update(int cur_id, int thiswin, int thisloss, int flag){
    allboard[cur_id].wins += (flag * thiswin + (1 - flag) * thisloss);
    allboard[cur_id].totaln += (thiswin + thisloss);

    if(cur_id == 0) return;
    re_update(allboard[cur_id].parent_id, thiswin, thisloss, flag ^ 1);
}

int re_argmax(int child_id[], int move_num, int parent_id){
    if(move_num == 0){
        return parent_id;
    }

    int argmax; 
    for(int i = 0; i < move_num; i++){
        if(child_id[i] != -1){
            argmax = child_id[i];
            break;
        }
    }
    

    // max search
    float max_UCB = -1000;
    float LCB_array[64];
    float not_pruning[64] = {0};
    int parent_log = fast_log2(allboard[parent_id].totaln);

    // pruning
    for (int i = 0; i < move_num; i++){
        if(child_id[i] < 0){
            LCB_array[i] = 1000;
        }
        LCB_array[i] = fast_LCB(allboard[child_id[i]].wins, allboard[child_id[i]].totaln, parent_log);
        if (LCB_array[i] > max_UCB && child_id[i] >= 0)
        {
            max_UCB = LCB_array[i];
            argmax = child_id[i];
        }
    }

    for (int i = 0; i < move_num; i++)
    {   
        if(child_id[i] >= 0){
            // float child_UCB = fast_UCB(win_num[i], node_sample_num[i], total_sample_num);
            float child_UCB = fast_UCB(allboard[child_id[i]].wins, allboard[child_id[i]].totaln, parent_log);
            
            if (child_UCB > max_UCB)
            {
                max_UCB = child_UCB;
                argmax = child_id[i];
            }

            // pruning
            for(int j = 0; j < move_num; j++){
                if(i != j && child_UCB > LCB_array[j]){
                    not_pruning[i] = 1;
                    break;
                }
            }
        }
    }

    //pruning
    for (int i = 0; i < move_num; i++)
        if(not_pruning[i] == 0 && child_id[i] != argmax){
            allboard[parent_id].child_id[i] = -1;

            //printf("%d, %lf\n", argmax, max_UCB);       
        }
    
    // check lower bound
    // if not return parent if yes keep searching
    if(allboard[argmax].totaln < Lowbound_explore && parent_id != 0) return parent_id;
    else return re_argmax(allboard[argmax].child_id, allboard[argmax].nchild, argmax);
    
}

int MCS_UCB_argmax(int parent_id, int child_id[], int move_num, int target_id)
{
    // you can change the parameter C of the UCB here
    // if you comment out this line, the default value will be 1.41421
    ucb_param_C = 1.2;

    unsigned int total_sample_num = 0;
    // start simulating
    // Prevent 0 appears in Denominator, do a even sampling first
    for (int i = 0; i < move_num; i++)
    {   
        int loss_sim = 0, win_sim = 0;
        for (int j = 0; j < Lowbound_explore; j++)
        {
            total_sample_num += 1;
            if(allboard[child_id[i]].simulate() == 1) win_sim++;
            else loss_sim++;
        }
        re_update(child_id[i], win_sim, loss_sim, 1);
    }
    
    // Then do MCS UCB
    while (total_sample_num < MAX_SIMULATION_COUNT && index_allboard + 1 < MaxBoard)
    {   
        // for timer
        clock_gettime(CLOCK_REALTIME, &end);
        double wall_clock_time = (double)((end.tv_sec + end.tv_nsec * 1e-9) - (double)(start.tv_sec + start.tv_nsec * 1e-9));
        if(wall_clock_time > can_use) break;

        // Simulate all childrens first
        for (int i = 0; i < move_num; i++)
        {   
            // pruning
            if(child_id[i] < 0) continue;

            int loss_sim = 0, win_sim = 0;
            for (int j = 0; j < SIMULATION_BATCH_NUM / 2; j++)
            {
                total_sample_num += 1;
                if(allboard[child_id[i]].simulate() == 1) win_sim++;
                else loss_sim++;
            }
            re_update(child_id[i], win_sim, loss_sim, 1);
        }

        // Simulate all childrens from randomness (cause by UCB and time step)
        float sumUCB = 0;
        int parent_log = fast_log2(allboard[parent_id].totaln);
        for(int i = 0; i < move_num; i++){
            if(child_id[i] < 0) continue;

            UCB_rand[i] = fast_UCB(allboard[child_id[i]].wins, allboard[child_id[i]].totaln, parent_log);
            UCB_rand[i] = exp((visited + 0.2 * (allboard[parent_id].depth + 1)) * UCB_rand[i]);
            sumUCB += UCB_rand[i];    
        }
        
        for(int i = 0; i < move_num; i++){

            // pruning
            if(child_id[i] < 0) continue;

            int playnum = round(UCB_rand[i] * move_num * SIMULATION_BATCH_NUM / sumUCB);

            int loss_sim = 0, win_sim = 0;
            for (int j = 0; j < playnum; j++)
            {
                total_sample_num += 1;
                if(allboard[child_id[i]].simulate() == 1) win_sim++;
                else loss_sim++;
            }
            re_update(child_id[i], win_sim, loss_sim, 1);
        }

        // find next point to simulate
        // now child_id, parent_id, move_num need to update
        parent_id = re_argmax(allboard[target_id].child_id, allboard[target_id].nchild, target_id);

        // is this node is goal can break now
        // always make sure the node has children
        while(total_sample_num < MAX_SIMULATION_COUNT){
            if(allboard[parent_id].check_winner()){
                re_update(parent_id, SIMULATION_BATCH_NUM, 0, 1);
                total_sample_num += SIMULATION_BATCH_NUM;
            }
            else break;
            parent_id = re_argmax(allboard[target_id].child_id, allboard[target_id].nchild, target_id);
        }
        if(total_sample_num >= MAX_SIMULATION_COUNT) break;
        
        //MCTS find next node
        // if the child is assigned not to be assigned again
        if(allboard[parent_id].move_count > 0){
            move_num = allboard[parent_id].move_count;
            child_id = allboard[parent_id].child_id;
            continue;
        }

        //MCTS find next node
        allboard[parent_id].generate_moves();
        for (int j = 0; j < allboard[parent_id].move_count; j++)
        {
            allboard[++index_allboard] = allboard[parent_id];
            allboard[index_allboard].parent_id = parent_id;
            allboard[index_allboard].this_id = index_allboard;
            allboard[index_allboard].depth = allboard[parent_id].depth + 1;
            allboard[index_allboard].nchild = 0;
            allboard[index_allboard].totaln = 0;
            allboard[index_allboard].wins = 0;
            allboard[index_allboard].move_count = 0;

            allboard[parent_id].child_id[j] = index_allboard;
            allboard[parent_id].nchild++;
            
            allboard[index_allboard].move(j);

            // never overflow
            if(index_allboard + 1 > MaxBoard){
                printf("overflow\n");
                break;
            }
        }
        move_num = allboard[parent_id].move_count;
        child_id = allboard[parent_id].child_id;
    }
    
    // Then return best step according to the win rate
    // NOT UCB! NOT UCB! NOT UCB!
    int return_argmax = 0;
    float max_WR = -1;
    for (int i = 0; i < allboard[target_id].nchild; i++)
    {   
        // pruning
        if(child_id[i] < 0) continue;
        float child_WR = (float)allboard[allboard[target_id].child_id[i]].wins / (float)allboard[allboard[target_id].child_id[i]].totaln;
        if (child_WR > max_WR)
        {
            max_WR = child_WR;
            return_argmax = i;
        }
    }
    return return_argmax;
}

// Except first move, call decide to decide which move to perform
int Board::decide()
{   
    visited += .5;
    generate_moves();
    allboard[0] = *(this);

    // A nice example for expansion
    // quick and elegant!
    
    for (int i = 0; i < allboard[0].move_count; i++)
    {
        allboard[++index_allboard] = allboard[0];
        allboard[index_allboard].parent_id = allboard[0].this_id;
        allboard[index_allboard].this_id = index_allboard;
        allboard[index_allboard].depth = allboard[0].depth + 1;
        allboard[index_allboard].nchild = 0;
        allboard[index_allboard].totaln = 0;
        allboard[index_allboard].wins = 0;
        allboard[index_allboard].move_count = 0;

        allboard[0].child_id[i] = index_allboard;
        allboard[0].nchild++;
        
        allboard[index_allboard].move(i);
    }

    // for timer
    clock_gettime(CLOCK_REALTIME, &start);
    can_use = remain_time * timeratio - 0.2;

    int action = MCS_UCB_argmax(allboard[0].this_id, allboard[0].child_id, allboard[0].nchild, allboard[0].this_id);
    for(int i = 0; i < index_allboard; i++) allboard[i].clear();
    index_allboard = 0;
    
    // for timer
    clock_gettime(CLOCK_REALTIME, &end);
    remain_time -= (double)((end.tv_sec + end.tv_nsec * 1e-9) - (double)(start.tv_sec + start.tv_nsec * 1e-9));

    return action;
}

// Only used in first move
int Board::first_move_decide_dice()
{
    // A nice example for expansion
    // quick and elegant!

    //visited += .4;
    allboard[index_allboard] = *(this);
    for (int i = 0; i < PIECE_NUM; i++)
    {
        // into child
        allboard[++index_allboard] = *(this);
        allboard[index_allboard].dice = i;
        allboard[index_allboard].parent_id = allboard[0].this_id;
        allboard[index_allboard].this_id = index_allboard;
        allboard[index_allboard].depth = allboard[this_id].depth + 1;
        allboard[index_allboard].nchild = 0;
        allboard[index_allboard].totaln = 0;
        allboard[index_allboard].wins = 0;
        allboard[index_allboard].move_count = 0;

        allboard[this_id].child_id[i] = index_allboard;
        allboard[this_id].nchild++;
        
    }

    // for timer
    clock_gettime(CLOCK_REALTIME, &start);
    can_use = remain_time * timeratio - 0.2;

    int action = MCS_UCB_argmax(allboard[0].this_id, allboard[0].child_id, allboard[0].nchild, allboard[0].this_id);
    for(int i = 0; i < index_allboard; i++) allboard[i].clear();
    index_allboard = 0;
    
    // for timer
    clock_gettime(CLOCK_REALTIME, &end);
    remain_time -= (double)((end.tv_sec + end.tv_nsec * 1e-9) - (double)(start.tv_sec + start.tv_nsec * 1e-9));

    return action;
}
// You should use mersenne twister and a random_device seed for the simulation
// But no worries, I've done it for you. Hope it can save you some time!
// Call random_num()%num to randomly pick number from 0 to num-1
std::mt19937 random_num(std::random_device{}());

// Very fast and clean simulation!
bool Board::simulate()
{
    Board board_copy = *(this);
    // run until game ends.
    while (!board_copy.check_winner())
    {
        board_copy.generate_moves();
        board_copy.move(random_num() % board_copy.move_count);
    }

    // game ends! find winner.
    // Win!
    if (board_copy.moving_color == moving_color)
        return true;
    // Lose!
    else
        return false;
}