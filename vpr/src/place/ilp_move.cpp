/*
 * @file: ilp_move.cpp
 * @brief: contains implementation of class ILPMove
 *
 * @author: Abed Yassine
 * @date_created: March 23rd, 2020
 * 
 */

#include "ilp_move.h"

ILPMove::ILPMove()
{
    gurobi_env_ = new GRBEnv("gurobi.log");
    gurobi_env_->start(); 
    gurobi_model_ = new GRBModel(*gurobi_env_);

    num_locations_ = 0; 
    num_blocks_ = 0;
    
    optimization_runtime_ = 0.0;
}

ILPMove::ILPMove(const std::string& log_filename)
{
    std::cout << "ILPMove::creating Gurobi env. " << std::endl;
    gurobi_env_ = new GRBEnv(log_filename); 
    gurobi_env_->start(); 
    std::cout << "ILPMove::creating Gurobi model. " << std::endl;
    
    try{
        gurobi_model_ = new GRBModel(*gurobi_env_);
    } catch (GRBException e) {
        std::cerr << "ILPMove::Error in creating the model. Gurobi error code = " << e.getErrorCode() << std::endl;
        std::cerr << e.getMessage() << std::endl;
    }

    std::cout << "ILPMove::Done constructor. " <<  std::endl;
    optimization_runtime_ = 0.0;
}

ILPMove::~ILPMove()
{
    if (delay_vars_) {
        delete delay_vars_; 
    }
    
    if (placement_vars_) {
        delete placement_vars_;
    }

    if (location_idx_vars_) {
        delete location_idx_vars_;
    }

    if (gurobi_model_) {
        delete gurobi_model_;
    }

    if (!gurobi_env_) {
        delete gurobi_env_;
    }
}

double ILPMove::get_opt_runtime() 
{
    return optimization_runtime_;
}

std::vector<int> ILPMove::get_placement_solution() 
{
    return block_var_;
}

std::vector<double> ILPMove::get_block_delays() 
{
    return delays_;
}

void ILPMove::set_location_types(const std::vector<std::vector<unsigned>>& loc_types)
{
    num_blocks_ = loc_types.size(); 
    num_locations_ = !loc_types.empty() ? loc_types[0].size() : 0;

    loc_type_ = loc_types;
}

void ILPMove::set_initial_placement(const std::vector<int>& init_placement)
{
    init_placement_ = init_placement;
}

void ILPMove::set_edge_delays(const std::vector<std::vector<double>>& edge_delays)
{
    delay_edge_ = edge_delays;
}

bool ILPMove::create_placement_vars() 
{
    if (!gurobi_model_) {
        return false;
    }

    try {
        placement_vars_ = gurobi_model_->addVars(num_blocks_*num_locations_, GRB_BINARY);
        gurobi_model_->update(); 
    } catch (GRBException e) {
        std::cerr << "ILPMove::create_placement_vars::Error in creating the placement variables. Gurobi error code = " << e.getErrorCode() << std::endl;
        std::cerr << e.getMessage() << std::endl;

        return false;
    } catch (...) {

        std::cerr << "ILPMove::create_placement_vars::Error in creating the placement variables. " << std::endl;
        return false;
    }

    return true;
}

bool ILPMove::create_location_idx_vars() 
{
    if (!gurobi_model_) {
        return false;
    }

    try {
        double * ub = new double [num_blocks_];
        char* type = new char [num_blocks_]; 
        for (unsigned i = 0; i < num_blocks_; i++) {
            ub[i] = num_locations_;
            type[i] = GRB_INTEGER;
        }


        location_idx_vars_ = gurobi_model_->addVars(NULL, ub, NULL, type, NULL, num_blocks_);
        gurobi_model_->update(); 
    } catch (GRBException e) {
        std::cerr << "ILPMove::create_location_idx_vars::Error in creating the location index variables. Gurobi error code = " << e.getErrorCode() << std::endl;
        std::cerr << e.getMessage() << std::endl;

        return false;
    } catch (...) {
        std::cerr << "ILPMove::create_location_idx_vars::Error in creating the locations index variables. " << std::endl;
        return false;
    }

    return true;
}

bool ILPMove::create_delay_vars()
{
    if (!gurobi_model_) {
        return false;
    }

    try {
        delay_vars_ = gurobi_model_->addVars(num_blocks_, GRB_CONTINUOUS); 

        gurobi_model_->update(); 
    } catch(GRBException e) {
        std::cerr << "ILPMove::create_delay_vars::Error in creating the delay variables. Gurobi error code = " << e.getErrorCode() << std::endl;
        std::cerr << e.getMessage() << std::endl;

        return false;
    } catch(...) {
        std::cerr << "ILPMove::create_delay_vars::Error in creating the delay variables. " << std::endl;
        return false; 
    }

    return true; 
}

bool ILPMove::create_gurobi_objective(unsigned cp_block_idx)
{
    if(!gurobi_model_) {
        return false;
    }

    try {
        GRBLinExpr obj = 1.0*delay_vars_[cp_block_idx];
        gurobi_model_->setObjective(obj, GRB_MINIMIZE); 
        gurobi_model_->update(); 
    } catch(GRBException e) {
        std::cerr << "ILPMove::create_gurobi_objective::Error in setting the objective. Gurobi error code = " << e.getErrorCode() << std::endl;
        std::cerr << e.getMessage() << std::endl;

        return false;
    } catch(...) {
        std::cerr << "ILPMove::create_gurobi_objective::Error in setting the objective. " << std::endl;
        return false; 
    }
    
    return true;
}

bool ILPMove::create_exclusivity_constraints()
{
    if (!gurobi_model_ || !placement_vars_) {
        return false;
    }

    try {
        // 1- No blocks occupies two locations 
        // Loop over all blocks 
        for (unsigned i = 0; i < num_blocks_; i++) {
            GRBLinExpr excl_expr = 0;
            
            // Loop over all locations 
            for (unsigned j = 0; j < num_locations_; j++) {
                excl_expr += placement_vars_[num_locations_*i + j];
            }

            excl_expr = excl_expr - 1; 

            gurobi_constraints_.push_back(gurobi_model_->addConstr(excl_expr, GRB_EQUAL, 0)); 
        }

        // 2- No two blocks occupy the same location 
        // Loop over all locations 
        GRBLinExpr expr = 0;
        for (unsigned j = 0; j < num_locations_; j++) {
            // Loop over all blocks
            expr = 0;
            for (unsigned i = 0; i < num_blocks_; i++) {
                        
                expr += placement_vars_[num_locations_*i + j];

            }

            gurobi_constraints_.push_back(gurobi_model_->addConstr(expr - 1, GRB_LESS_EQUAL, 0)); 
        }

    } catch (GRBException e) {
        std::cerr << "ILPMove::create_exclusivity_constraints::Error in creating constraints. Gurobi error code = " << e.getErrorCode() << std::endl;
        std::cerr << e.getMessage() << std::endl;

        return false;
    } catch (...) {
        std::cerr << "ILPMove::create_exclusivity_constraints::Error in creating constraints. " << std::endl;
        return false; 
    }

    return true;
}

bool ILPMove::create_type_constraints()
{
     if (!gurobi_model_ || !placement_vars_) {
        return false;
    }

    try {
        // Loop over all blocks 
        for (unsigned i = 0; i < num_blocks_; i++) {
            GRBLinExpr excl_expr = 0;
            
            // Loop over all locations 
            for (unsigned j = 0; j < num_locations_; j++) {
                excl_expr = placement_vars_[num_locations_*i + j] - loc_type_[i][j];
            }

            gurobi_constraints_.push_back(gurobi_model_->addConstr(excl_expr, GRB_LESS_EQUAL, 0)); 
        }

    } catch (GRBException e) {
        std::cerr << "ILPMove::create_type_constraints::Error in creating constraints. Gurobi error code = " << e.getErrorCode() << std::endl;
        std::cerr << e.getMessage() << std::endl;

        return false;
    } catch (...) {
        std::cerr << "ILPMove::create_type_constraints::Error in creating constraints. " << std::endl;
        return false; 
    }

    return true;
}

bool ILPMove::create_fixed_blocks_constraints()
{
     if (!gurobi_model_ || !placement_vars_) {
        return false;
    }

    try {
        // Loop over all blocks 
        for (unsigned i = 0; i < num_blocks_; i++) {
            if (init_placement_[i] == -1) // this is not a fixed block
                continue;

            GRBLinExpr expr = placement_vars_[num_locations_*i + init_placement_[i]] - 1;

            gurobi_constraints_.push_back(gurobi_model_->addConstr(expr, GRB_EQUAL, 0)); 
        }

    } catch (GRBException e) {
        std::cerr << "ILPMove::create_fixed_blocks_constraints::Error in creating constraints. Gurobi error code = " << e.getErrorCode() << std::endl;
        std::cerr << e.getMessage() << std::endl;

        return false;
    } catch (...) {
        std::cerr << "ILPMove::create_fixed_blocks_constraints::Error in creating constraints. " << std::endl;
        return false; 
    }

    return true;
}

bool ILPMove::create_location_idx_constraints()
{
     if (!gurobi_model_ || !placement_vars_ || !location_idx_vars_) {
        return false;
    }

    try {
        // Loop over all blocks 
        for (unsigned i = 0; i < num_blocks_; i++) {
            
            // Loop over all locations 
            for (unsigned j = 0; j < num_locations_; j++) {

                GRBLinExpr expr1 = location_idx_vars_[i] - (j*placement_vars_[num_locations_*i + j]);
                GRBLinExpr expr2 = location_idx_vars_[i] - (j*placement_vars_[num_locations_*i + j]) - 
                        (num_locations_*(1 - placement_vars_[num_locations_*i + j]));

                gurobi_constraints_.push_back(gurobi_model_->addConstr(expr1, GRB_GREATER_EQUAL, 0)); 
                gurobi_constraints_.push_back(gurobi_model_->addConstr(expr2, GRB_LESS_EQUAL, 0)); 

            }
        }

    } catch (GRBException e) {
        std::cerr << "ILPMove::create_location_idx_constraints::Error in creating constraints. Gurobi error code = " << e.getErrorCode() << std::endl;
        std::cerr << e.getMessage() << std::endl;

        return false;
    } catch (...) {
        std::cerr << "ILPMove::create_location_idx_constraints::Error in creating constraints. " << std::endl;
        return false; 
    }

    return true;
}

bool ILPMove::create_delay_constraints(const std::vector<unsigned>& cp_blocks)
{
    if (!gurobi_model_ || !placement_vars_ || !delay_vars_ || !location_idx_vars_) {
        return false;
    }

    if (cp_blocks.empty() || cp_blocks.size() > num_blocks_ || cp_blocks.empty()) {
        return false;
    }

    try {
        
        // 1- add a constraint for delay of the first block 
        gurobi_constraints_.push_back(gurobi_model_->addConstr(delay_vars_[cp_blocks[0]], GRB_EQUAL, 0)); 

        // 2- Create edge delay variables 
        edge_delay_vars_ = gurobi_model_->addVars(cp_blocks.size() - 1, GRB_CONTINUOUS); 

        // 3- Loop over rest of cp blocks to add delay constraints 
        for (unsigned i = 1; i < cp_blocks.size(); i++) {
            //int placement_i = static_cast<int>(location_idx_vars_[cp_blocks[i]].get(GRB_DoubleAttr_X));
            //int placement_i_1 = static_cast<int>(location_idx_vars_[cp_blocks[i-1]].get(GRB_DoubleAttr_X));
            
            
            GRBLinExpr delay_expr = delay_vars_[cp_blocks[i]] - delay_vars_[cp_blocks[i-1]] 
                        - edge_delay_vars_[i-1];// delay_edge_[placement_i_1][placement_i];

            gurobi_constraints_.push_back(gurobi_model_->addConstr(delay_expr, GRB_EQUAL, 0)); 
        }

        // 4- Add edge delay constraints to capture lookup table 
        double LARGE_NUMBER=10;//999999;
        for (unsigned i = 0; i < cp_blocks.size() - 1; i++) {
            for (unsigned j = 0; j < num_locations_; j++) {
                for (unsigned k = 0; k < num_locations_; k++) {

                    GRBLinExpr edge_delay_expr_1 = edge_delay_vars_[i] - (1.5*delay_edge_[j][k]*(placement_vars_[cp_blocks[i]*num_locations_ + j]
                                + placement_vars_[cp_blocks[i+1]*num_locations_ + k]))
                                + 2 * delay_edge_[j][k];
                                //*( 2 - placement_vars_[cp_blocks[i]*num_locations_ + j]
                                //- placement_vars_[cp_blocks[i+1]*num_locations_ + k]); 
                     
                    GRBLinExpr edge_delay_expr_2 = edge_delay_vars_[i] - ((0.5*delay_edge_[j][k] - LARGE_NUMBER)*
                                (placement_vars_[cp_blocks[i]*num_locations_ + j]
                                + placement_vars_[cp_blocks[i+1]*num_locations_ + k]))
                                - 2*LARGE_NUMBER;//*( 2 - placement_vars_[cp_blocks[i]*num_locations_ + j]
                                //- placement_vars_[cp_blocks[i+1]*num_locations_ + k]); 

                    gurobi_constraints_.push_back(gurobi_model_->addConstr(edge_delay_expr_1, GRB_GREATER_EQUAL, 0));
                    gurobi_constraints_.push_back(gurobi_model_->addConstr(edge_delay_expr_2, GRB_LESS_EQUAL, 0));
                }
            }
        }

    } catch (GRBException e) {
        std::cerr << "ILPMove::create_delay_constraints::Error in creating constraints. Gurobi error code = " << e.getErrorCode() << std::endl;
        std::cerr << e.getMessage() << std::endl;

        return false;
    } catch (...) {
        std::cerr << "ILPMove::create_delay_constraints::Error in creating constraints. " << std::endl;
        return false; 
    }

    return true;
}

bool ILPMove::reset_model()
{
    std::cerr << "ILPMove::resetting model. " << std::endl;
    if (delay_vars_) {
        delete delay_vars_; 
    }
    
    if (placement_vars_) {
        delete placement_vars_;
    }

    if (location_idx_vars_) {
        delete location_idx_vars_;
    }

    gurobi_constraints_.clear(); 
    gurobi_constraints_.resize(0); 

    if (gurobi_model_) {
        delete gurobi_model_;
    }

    try {
        gurobi_model_ = new GRBModel(*gurobi_env_);
    } catch (GRBException e) {
        std::cerr << "ILPMove::reset_model::Error in resetting the model. Gurobi error code = " << e.getErrorCode() << std::endl;
        std::cerr << e.getMessage() << std::endl;
        
        return false;
    } catch (...) {
        std::cerr << "ILPMove::reset_model::Error in resetting the model. " << std::endl;
        return false; 
    }

    return true;
}

bool ILPMove::initialize_ilp_move(const std::vector<unsigned>& cp_blocks) 
{
    std::cerr << "ILPMove::initializing constraints" << std::endl;
    if (!gurobi_model_) {
        return false;
    }

    bool success = true; 

    // create variables
    success &= create_placement_vars();
    success &= create_delay_vars();
    success &= create_location_idx_vars();
    
    // set objective
    success &= create_gurobi_objective(cp_blocks[cp_blocks.size() - 1]); 

    // create the constraints
    success &= create_exclusivity_constraints(); 
    success &= create_type_constraints(); 
    success &= create_fixed_blocks_constraints(); 
    success &= create_location_idx_constraints(); 
    success &= create_delay_constraints(cp_blocks); 
   
//     gurobi_model_->write("ilp.lp"); 

    return success; 
}

bool ILPMove::solve_ilp()
{
    std::cerr << "ILPMove::Solving ILP" << std::endl;
    if (!gurobi_model_) {
        return false;
    }

    try {
        gurobi_model_->optimize(); 

    } catch (GRBException e) {
        std::cerr << "ILPMove::solve_ilp::Error in solving the model. Gurobi error code = " << e.getErrorCode() << std::endl;
        std::cerr << e.getMessage() << std::endl;
        
        return false;
    } catch (...) {
        std::cerr << "ILPMove::solve_ilp::Error in solving the model. " << std::endl;
        return false; 
    }

    if (gurobi_model_->get(GRB_IntAttr_Status) == GRB_OPTIMAL) // TODO might need to modify this if using callbacks to terminate early
    {
        optimization_runtime_ = gurobi_model_->get(GRB_DoubleAttr_Runtime); 

        // clear previous results 
       
        block_var_.clear();
        delays_.clear(); 
        
        block_var_.resize(num_blocks_); 
        for (unsigned i = 0; i < num_blocks_; i++) {
            block_var_[i] = static_cast<int>(location_idx_vars_[i].get(GRB_DoubleAttr_X)); 
            delays_[i] = delay_vars_[i].get(GRB_DoubleAttr_X); 
        }

        return true;
    }

    return false;
}
