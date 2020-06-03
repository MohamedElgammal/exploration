/* 
 * @file: ilp_move.h
 *
 * @brief: This file specifies the interface for an ILP based move. The implementation creates the formulation and solves it
 *
 * @autho: Abed Yassine
 * @date_created: March 23rd, 2020
 */

#ifndef VPR_ILP_MOVE_H
#define VPR_ILP_MOVE_H

// Include all required Gurobi headers 
#include "gurobi_c++.h"

// Any other includes from stl 
#include <vector>
#include <unordered_map>
#include <string>
#include <iostream>

class ILPMove
{   
public:
    //! Default constructor
    ILPMove();

    //! Regular Constructor
    //! log_filename [in]: Filename of the log file for Gurobi. Default is "gurobi.log"
    ILPMove(const std::string& log_filename="gurobi.log");

    //! Destructor
    ~ILPMove();
    
    // Getters
    //! Returns the placement of all blocks after optimization
    std::vector<int> get_placement_solution(); 

    //! Returns the delay at each block after optimization 
    std::vector<double> get_block_delays(); 

    //! Returns the runtime of the optimization 
    double get_opt_runtime(); 


    // Setters
    //! This method sets the loc_type_ matrix. If loc_type_[i][j] = 1, then block i can be place in location j
    void set_location_types(const std::vector<std::vector<unsigned>> & loc_types); 

    //! This method sets the placement of the fixed (unmovable) blocks
    void set_initial_placement(const std::vector<int> & init_placement); 

    //! This method sets the lookup table of delays between each two locations in the grid 
    void set_edge_delays(const std::vector<std::vector<double>> & edge_delays); 

    //! This method resets the optimziation model 
    //! Returns true if successful
    bool reset_model(); 

    //! This method initializes the optimization model (defines variables and constraints)
    //! cp_blocks [in]: vector of critical path block indices to move 
    //! Returns true if successful
    bool initialize_ilp_move(const std::vector<unsigned>& cp_blocks); 

    //! This method solves the ILP, and stores the solutions 
    //! Returns true if successful.
    bool solve_ilp(); 

private:
    /********* Helper methods *******/
    
    //! Creats the binary x variables 
    //! returns true if successful
    bool create_placement_vars(); 

    //! Creates the variables for location index
    //! returns true if successful
    bool create_location_idx_vars();

    //! Creates the block delay variables 
    //! returns ture if successful 
    bool create_delay_vars(); 

    //! Creates objective of optimization 
    //! cp_block_idx [in]: Index of the last block in the critical path that we are trying to optimize its delay
    //! Returns true if successful
    bool create_gurobi_objective(unsigned cp_block_idx); 

    //! This creates the exclusivity constraints. No two blocks occupy the same location, and no block occupies two different locations 
    //! Returns true if successful
    bool create_exclusivity_constraints(); 

    //! This creates the constraints for the block and location types
    //! Returns true if successful
    bool create_type_constraints(); 

    //! This creates constraints to keep the fixed blocks in their places
    //! Returns true if successful
    bool create_fixed_blocks_constraints(); 

    //! This creates constraints for the ILP to figure out the actual index for the location in which block is placed (sets location_idx_vars_)
    //! Returns true if successful
    bool create_location_idx_constraints(); 

    //! This creates the delay constraints
    //! cp_blocks [in]: vector of the critical path block indices to move. Should be ordered topologically.
    //! Returns true if successful 
    bool create_delay_constraints(const std::vector<unsigned> & cp_blocks); 

    /***** Member variables *******/

    // Gurobi related parameters
    GRBEnv*     gurobi_env_ = nullptr;    // Gurobi Environment
    GRBModel*   gurobi_model_ = nullptr;  // Gurobi model 

    GRBVar*     placement_vars_ = nullptr; // binary x var in ilp 
    GRBVar*     location_idx_vars_ = nullptr; // stores the index of the location a block i is placed at 
    GRBVar*     delay_vars_ = nullptr;        // Delay variables in ILP 
    GRBVar*     edge_delay_vars_ = nullptr;    // capture the delay between two locations from a lookup table

    std::vector<GRBConstr> gurobi_constraints_; // vector that holds all the constraints in the model. Might be useful to modify them each temp update


    // FPGA grid related parameters  
    std::vector<std::vector<unsigned>> loc_type_;        // corresponds to the variable y in the ILP formulation. if loc_type_[i][j] = 1, then block i 
                                                         // CAN be placed in location j // TODO can be put in compressed column format as it is sparse 
    std::vector<int>                   block_var_;       // Stores the solution of the  x variable the ILP formulation. 
                                                         // If block_var_[i] = j, then block i IS placed in location j
    std::vector<int>                   init_placement_;  // Stores the initial placement of the fixed blocks so the ILP disregards them 
    std::vector<double>                delays_;          // Stores the solution of the delay variable in the ILP. delays_[i] is delay at block i.
    std::vector<std::vector<double>>   delay_edge_;      // Stores the estimated delay between each two locations in the grid

    unsigned                           num_locations_;   // number of locations in the grid
    unsigned                           num_blocks_;      // number of blocks to place. Includes fixed blocks 

    double                             optimization_runtime_; // Result: Runtime of the optimization 
};
#endif
