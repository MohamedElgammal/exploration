#include "place_util.h"
#include "globals.h"

//Placement Checkpoint 
placement_checkpoint place_cp;

static vtr::Matrix<t_grid_blocks> init_grid_blocks();

void init_placement_context() {
    auto& place_ctx = g_vpr_ctx.mutable_placement();
    auto& cluster_ctx = g_vpr_ctx.clustering();

    place_ctx.block_locs.clear();
    place_ctx.block_locs.resize(cluster_ctx.clb_nlist.blocks().size());

    place_ctx.grid_blocks = init_grid_blocks();
}

static vtr::Matrix<t_grid_blocks> init_grid_blocks() {
    auto& device_ctx = g_vpr_ctx.device();

    auto grid_blocks = vtr::Matrix<t_grid_blocks>({device_ctx.grid.width(), device_ctx.grid.height()});
    for (size_t x = 0; x < device_ctx.grid.width(); ++x) {
        for (size_t y = 0; y < device_ctx.grid.height(); ++y) {
            auto type = device_ctx.grid[x][y].type;

            int capacity = type->capacity;

            grid_blocks[x][y].blocks.resize(capacity, EMPTY_BLOCK_ID);
        }
    }

    return grid_blocks;
}

float get_cp_cpd() {return place_cp.cpd;}
double get_cp_bb_cost() {return place_cp.bb_cost;}
bool cp_is_valid() {return place_cp.valid;}

void save_placement(){
    auto& place_ctx = g_vpr_ctx.placement();
    place_cp.block_locs  = place_ctx.block_locs;
    place_cp.physical_pins = place_ctx.physical_pins;
    place_cp.grid_blocks = place_ctx.grid_blocks;
    place_cp.valid = true;
}

void restore_placement(){
    auto& mutable_place_ctx = g_vpr_ctx.mutable_placement();
    mutable_place_ctx.block_locs = place_cp.block_locs;
    mutable_place_ctx.physical_pins = place_cp.physical_pins;
    mutable_place_ctx.grid_blocks = place_cp.grid_blocks;
}
