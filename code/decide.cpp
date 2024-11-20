// These are not basic function for board.
// write them in separate file makes code cleaner.
// I recommend write all your complicated algorithms in this file.
// Or sort them into even more files to make code even cleaner!

#include "board/board.hpp"
#include "math_lib/maths.hpp"
#include <stdio.h>
#include <random>
#define SIMULATION_BATCH_NUM 100
#define MAX_SIMULATION_COUNT 600000
#define MAX_Tree_SIMULATION_COUNT 12

// init lots board
#define MaxBoard 500009
static Board allboard[MaxBoard];
int index_allboard = 0;

// MCS UCB version
// I use max sample number as the simulation limit.
// Recommend: Using wall clock time to precisely limit the time.
// Advanced: Using both max sample number and wall clock time to limit the simiulation time.

/*
int hashing_index(int moving_color, int dice, int &piece_position){
    long long int hash_value = dice + 26 * moving_color;
    for(int j = 0; j < 2; j++)
        for(int k = 0; k < 6; k++)
            hash_value += pow(26, j * 6 + k + 2) * piece_position[j][k];
    hash_value = hash_value % MaxBoard;
    return hash_value
}
*/

void re_update(int cur_id, int thiswin, int thisloss, int flag){
    allboard[cur_id].wins += (flag * thiswin + (1 - flag) * thisloss);
    allboard[cur_id].totaln += (thiswin + thisloss);

    if(cur_id == 0) return;
    //printf("%d\n", cur_id);
    re_update(allboard[cur_id].parent_id, thiswin, thisloss, flag ^ 1);
}

int re_argmax(int child_id[], int move_num, int parent_id){
    if(move_num == 0){
        return parent_id;
    }
    int argmax = child_id[0];

    // max search
    float max_UCB = -1;
    // printf("%d\n", move_num);
    for (int i = 0; i < move_num; i++)
    {
        // float child_UCB = fast_UCB(win_num[i], node_sample_num[i], total_sample_num);
        float child_UCB = fast_UCB(allboard[child_id[i]].wins, allboard[child_id[i]].totaln, allboard[parent_id].totaln);
        if (child_UCB > max_UCB)
        {
            max_UCB = child_UCB;
            argmax = child_id[i];
        }
    }
    
    //printf("%d\n", argmax);
    return re_argmax(allboard[argmax].child_id, allboard[argmax].nchild, argmax);
}

int MCS_UCB_argmax(int parent_id, int child_id[], int move_num, int target_id)
{
    // you can change the parameter C of the UCB here
    // if you comment out this line, the default value will be 1.41421
    // ucb_param_C = 1.41421

    //unsigned int node_sample_num[64] = {};
    //unsigned int win_num[64] = {};
    unsigned int total_sample_num = 0;
    // start simulating
    // Prevent 0 appears in Denominator, do a even sampling first
    for (int i = 0; i < move_num; i++)
    {   
        int loss_sim = 0, win_sim = 0;
        for (int j = 0; j < SIMULATION_BATCH_NUM; j++)
        {
            /*
            if (allboard[child_id[i]].simulate() == true)
            {
                win_num[i] += 1;
            }
            node_sample_num[i] += 1;
            total_sample_num += 1;
            */
            total_sample_num += 1;
            if(allboard[child_id[i]].simulate() == 1) win_sim++;
            else loss_sim++;
        }
        re_update(child_id[i], win_sim, loss_sim, 1);
    }
    
    // Then do MCS UCB
    while (total_sample_num < MAX_SIMULATION_COUNT)
    {   
        if (allboard[parent_id].check_winner()){
            re_update(parent_id, SIMULATION_BATCH_NUM, 0, 1);
            total_sample_num += 1000;
            continue;
        }
        // find the child which has the highest UCB
        for (int i = 0; i < move_num; i++)
        {   
            int loss_sim = 0, win_sim = 0;
            for (int j = 0; j < SIMULATION_BATCH_NUM; j++)
            {
                /*
                if (allboard[child_id[i]].simulate() == true)
                {
                    win_num[i] += 1;
                }
                node_sample_num[i] += 1;
                total_sample_num += 1;
                */
                total_sample_num += 1;
                if(allboard[child_id[i]].simulate() == 1) win_sim++;
                else loss_sim++;
            }
            re_update(child_id[i], win_sim, loss_sim, 1);
        }

        /*
        int argmax = child_id[0];
        float max_UCB = -1;
        for (int i = 0; i < move_num; i++)
        {
            // float child_UCB = fast_UCB(win_num[i], node_sample_num[i], total_sample_num);
            float child_UCB = fast_UCB(allboard[child_id[i]].wins, allboard[child_id[i]].totaln, allboard[parent_id].totaln);
            if (child_UCB > max_UCB)
            {
                max_UCB = child_UCB;
                argmax = child_id[i];
            }
        }
        
        //printf("Go in 108\n");
        // do simulation on child[argmax]
        int loss_sim = 0, win_sim = 0;
        for (int j = 0; j < SIMULATION_BATCH_NUM; j++)
        {   
            //printf("%d\n", argmax);
            //printf("%d\n", allboard[argmax].simulate());
            //printf("Hi\n");
            total_sample_num += 1;
            //re_update(argmax, allboard[argmax].simulate());
            if(allboard[argmax].simulate() == 1) win_sim++;
            else loss_sim++;
        }
        re_update(argmax, win_sim, loss_sim, 1);
        */
        //printf("Go in 122\n");
        // find next point to simulate
        // now child_id, parent_id, move_num need to update
        parent_id = re_argmax(allboard[target_id].child_id, allboard[target_id].nchild, target_id);
        //printf("Go in 126\n");
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
        }
        move_num = allboard[parent_id].move_count;
        child_id = allboard[parent_id].child_id;
        //printf("Go in 140 in %d %d\n", move_num, allboard[parent_id].nchild);
    }
    //printf("Go in 151\n");
    // Then return best step according to the win rate
    // NOT UCB! NOT UCB! NOT UCB!
    int return_argmax = 0;
    float max_WR = -1;
    /*
    for (int i = 0; i < move_num; i++)
    {
        // float child_WR = (float)win_num[i]/(float)node_sample_num[i];
        float child_WR = (float)allboard[child_id[i]].wins,/(float)allboard[child_id[i]].totaln;
        if (child_WR > max_WR)
        {
            max_WR = child_WR;
            return_argmax = i;
        }
    }
    */
    //printf("Go in 168\n");
    for (int i = 0; i < allboard[target_id].nchild; i++)
    {
        // float child_WR = (float)win_num[i]/(float)node_sample_num[i];
        float child_WR = (float)allboard[allboard[target_id].child_id[i]].wins / (float)allboard[allboard[target_id].child_id[i]].totaln;
        if (child_WR > max_WR)
        {
            max_WR = child_WR;
            return_argmax = i;
        }
        //printf("Go in 178\n");
    }
    return return_argmax;
}

// Except first move, call decide to decide which move to perform
int Board::decide()
{   
    generate_moves();
    allboard[0] = *(this);
    allboard[0].generate_moves();
    // A nice example for expansion
    // quick and elegant!
    // Board child_nodes[64];
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
    int action = MCS_UCB_argmax(allboard[0].this_id, allboard[0].child_id, allboard[0].nchild, allboard[0].this_id);
    for(int i = 0; i < index_allboard; i++) allboard[i].clear();
    index_allboard = 0;
    //printf("Actions = %d\n", action);
    return action;
}

// Only used in first move
int Board::first_move_decide_dice()
{
    // A nice example for expansion
    // quick and elegant!
    //Board child_nodes[PIECE_NUM];

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

    // recursively search
    /*
    int next_id = MCS_UCB_argmax(this_id, allboard[this_id].child_id, allboard[this_id].nchild, this_id);
    for(int i = 1; i < MAX_Tree_SIMULATION_COUNT; i++){
        allboard[next_id].generate_moves();

        for (int j = 0; j < move_count; j++)
        {
            allboard[++index_allboard] = *(allboard[next_id]);
            allboard[index_allboard].parent_id = next_id;
            allboard[index_allboard].this_id = index_allboard;
            allboard[index_allboard].depth = allboard[next_id].depth + 1;
            allboard[index_allboard].nchild = 0;
            allboard[index_allboard].totaln = 0;
            allboard[index_allboard].wins = 0;
            allboard[index_allboard].move_count = 0;

            allboard[next_id].child_id[j] = index_allboard;
            allboard[next_id].nchild++;
            
            allboard[index_allboard].move(j);
        }
        next_id = MCS_UCB_argmax(this_id, child_id, nchild, this_id);
    }
    */
    
    int action = MCS_UCB_argmax(allboard[0].this_id, allboard[0].child_id, allboard[0].nchild, allboard[0].this_id);
    for(int i = 0; i < index_allboard; i++) allboard[i].clear();
    index_allboard = 0;
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