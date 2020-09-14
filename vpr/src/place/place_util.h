#ifndef PLACE_UTIL_H
#define PLACE_UTIL_H
#include <string>
#include "vpr_types.h"
#include "vtr_util.h"  
#include "vtr_vector_map.h"

//Initialize the placement context
void init_placement_context();

//Placement checkpoint
struct placement_checkpoint{
    vtr::vector_map<ClusterBlockId, t_block_loc> block_locs;
    vtr::vector_map<ClusterPinId, int> physical_pins;
    vtr::Matrix<t_grid_blocks> grid_blocks; //[0..device_ctx.grid.width()-1][0..device_ctx.grid.width()-1]
    float cpd;
    double bb_cost;
    bool valid = false;
};

//Placement Checkpointing
void save_placement();
void restore_placement();

float get_cp_cpd ();
double get_cp_bb_cost();
bool cp_is_valid();

#endif
