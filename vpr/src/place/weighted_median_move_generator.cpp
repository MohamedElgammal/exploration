#include "weighted_median_move_generator.h"
#include "globals.h"
#include <algorithm>
#include "math.h"


static void get_bb_for_net_excluding_block(ClusterNetId net_id, t_bb* coords, ClusterBlockId block_id);


e_create_move WeightedMedianMoveGenerator::propose_move(t_pl_blocks_to_be_moved& blocks_affected, float ,
    std::vector<int>& X_coord, std::vector<int>& Y_coord, std::vector<int>&, int &,int place_high_fanout_net) {
    /* Pick a random block to be swapped with another random block.   */
    ClusterBlockId b_from = pick_from_block();
    if (!b_from) {
        return e_create_move::ABORT; //No movable block found
    }

    auto& place_ctx = g_vpr_ctx.placement();
    auto& cluster_ctx = g_vpr_ctx.clustering();

    t_pl_loc from = place_ctx.block_locs[b_from].loc;
    auto cluster_from_type = cluster_ctx.clb_nlist.block_type(b_from);
    auto grid_from_type = g_vpr_ctx.device().grid[from.x][from.y].type;
    VTR_ASSERT(is_tile_compatible(grid_from_type, cluster_from_type));

    /* Calculate the median region */
    t_pl_loc to;

    t_bb coords;
    t_bb limit_coords;
    X_coord.clear();
    Y_coord.clear();
    //std::vector<std::pair<int,float>> X,Y;
    for (ClusterPinId pin_id : cluster_ctx.clb_nlist.block_pins(b_from)) {
        ClusterNetId net_id = cluster_ctx.clb_nlist.pin_net(pin_id);
        if (cluster_ctx.clb_nlist.net_is_ignored(net_id))
            continue;
        if(int(cluster_ctx.clb_nlist.net_pins(net_id).size()) >  place_high_fanout_net)
            continue;
                
        get_bb_for_net_excluding_block(net_id, &coords, b_from);
        X_coord.push_back(coords.xmin);
        X_coord.push_back(coords.xmax);
        Y_coord.push_back(coords.ymin);
        Y_coord.push_back(coords.ymax);
    }
    std::sort(X_coord.begin(),X_coord.end());
    std::sort(Y_coord.begin(),Y_coord.end());

    if((X_coord.size()==0) || (Y_coord.size()==0))
        return e_create_move::ABORT;

    limit_coords.xmin = X_coord[floor((X_coord.size()-1)/2)];
    limit_coords.xmax = X_coord[floor((X_coord.size()-1)/2)+1];

    limit_coords.ymin = Y_coord[floor((Y_coord.size()-1)/2)];
    limit_coords.ymax = Y_coord[floor((Y_coord.size()-1)/2)+1];
    
    
    if (!find_to_loc_median(cluster_from_type, &limit_coords, from, to)) {
        return e_create_move::ABORT;
    }

    return ::create_move(blocks_affected, b_from, to);
}



/* This routine finds the bounding box of a net from scratch excluding   *
 * a specific block. This is very useful in some directed moves.         *
 * It updates coordinates of the bb                                      */
static void get_bb_for_net_excluding_block(ClusterNetId net_id, t_bb* coords, ClusterBlockId block_id) {
    int pnum, x, y, xmin, xmax, ymin, ymax;
    float xmin_cost,xmax_cost,ymin_cost,ymax_cost;
    xmin=0;
    xmax=0;
    ymin=0;
    ymax=0;
    xmin_cost=0.0;
    xmax_cost=0.0;
    ymin_cost=0.0;
    ymax_cost=0.0;

    auto& cluster_ctx = g_vpr_ctx.clustering();
    auto& place_ctx = g_vpr_ctx.placement();
    auto& device_ctx = g_vpr_ctx.device();
    auto& grid = device_ctx.grid;

    ClusterBlockId bnum;
    bool first_block_excluding = true;
    int ipin = 0;
    for (auto pin_id : cluster_ctx.clb_nlist.net_pins(net_id)) {
        bnum = cluster_ctx.clb_nlist.pin_block(pin_id);
        ++ipin;
        if(bnum != block_id)
        {
            pnum = tile_pin_index(pin_id);
            VTR_ASSERT(pnum >= 0);
            x = place_ctx.block_locs[bnum].loc.x + physical_tile_type(bnum)->pin_width_offset[pnum];
            y = place_ctx.block_locs[bnum].loc.y + physical_tile_type(bnum)->pin_height_offset[pnum];

            x = std::max(std::min(x, (int)grid.width() - 2), 1);  //-2 for no perim channels
            y = std::max(std::min(y, (int)grid.height() - 2), 1); //-2 for no perim channels

            if(first_block_excluding){
                xmin = x;
                xmin_cost = get_timing_place_crit(net_id, ipin);
                ymin = y;
                ymin_cost = get_timing_place_crit(net_id, ipin);
                xmax = x;
                xmax_cost = get_timing_place_crit(net_id, ipin);
                ymax = y;
                ymax_cost = get_timing_place_crit(net_id, ipin);
                first_block_excluding = false;
            }
            else {
                if (x < xmin) {
                    xmin = x;
                    xmin_cost = get_timing_place_crit(net_id, ipin);
                } else if (x > xmax) {
                    xmax = x;
                    xmax_cost = get_timing_place_crit(net_id, ipin);
                }

                if (y < ymin) {
                    ymin = y;
                    ymin_cost = get_timing_place_crit(net_id, ipin);
                } else if (y > ymax) {
                    ymax = y;
                    ymax_cost = get_timing_place_crit(net_id, ipin);
                }
            }
        }
    }

    /* Copy the coordinates and number on edges information into the proper   *
     * structures.                                                            */
    coords->xmin = round(xmin*xmin_cost);
    coords->xmax = round(xmax*xmax_cost);
    coords->ymin = round(ymin*ymin_cost);
    coords->ymax = round(ymax*ymax_cost);
}