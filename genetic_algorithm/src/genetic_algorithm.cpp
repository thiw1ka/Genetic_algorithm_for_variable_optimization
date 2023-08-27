#include "genetic_algorithm/genetic_algorithm.hpp"

genetic_algorithm::RandomNumberGenerator::RandomNumberGenerator (){
    std::printf("[genetic_alg] RandomNumberGenerator () \n");
    gen_ = new std::mt19937(rd_());
    int_dist_ = nullptr;
    double_dist_ = nullptr;
};

genetic_algorithm::genetic_algorithm(ros::NodeHandle n) : n_(n)/*, pf_static_member_setter()*/ {
    std::printf("[genetic_alg] genetic_algorithm object initiation. \n");
    ns = ros::this_node::getNamespace();

    //parameter list to be obtained from roslaunch
    //local

    //universal
    n_.param <int>  ("/Population_size", Population_size, 1);
    n_.param <int>  ("/top_selection_percentage", top_selection, 10); //default 10%
    n_.param <int>  ("/mutating_percentage", mutating_percentage, 10); //default 10%
    n_.param <int>  ("/crossover_percentage", crossover_percentage, 10); //default 10%
    bool is_groundtruth_DIR = n_.getParam     ("/GROUND_TRUTH_DIR",GROUND_TRUTH_DIR_);
    bool is_result_DIR      = n_.getParam     ("/RESULT_DIR", RESULT_DIR_);
    bool is_ROSBAG_DIR_     = n_.getParam     ("/ROSBAG_DIR_", ROSBAG_DIR_);
    running_speed = 1;
    std::printf("[genetic_algorithm] Population_size(%i), top_selection(%i), mutating_percentage(%i), crossover_percentage(%i) \n",
                                                            Population_size, top_selection, mutating_percentage, crossover_percentage);
    std::printf("[genetic_algorithm] paths. \n GROUND_TRUTH_DIR_(%s),\n RESULT_DIR_(%s),\n RESULT_DIR_(%s)  \n",
                            GROUND_TRUTH_DIR_.c_str(), RESULT_DIR_.c_str(), ROSBAG_DIR_.c_str());

    if( !experimental::filesystem::exists(GROUND_TRUTH_DIR_) | !experimental::filesystem::exists(RESULT_DIR_)) {
        std::printf("[genetic_algorithm] Error paths not valid. \n GROUND_TRUTH_DIR_(%s),\n RESULT_DIR_(%s)  \n",
                                                                GROUND_TRUTH_DIR_.c_str(), RESULT_DIR_.c_str());
        n_.shutdown();
    }

    //other class constructors
    // std::string bagpath = "/home/thiw1ka/catkin_ws/src/uri_soft_wip/cyanobloom_sabbatus/cyanobloom_phd_filter_sabbatus/results/2.Rosbags/2022-03-25-11-58-13_NEwind.bag";
    double speed = 10.0;
    bagPlayerPtr.reset(new bag_player(ROSBAG_DIR_, speed));

    /*generating random values to select pos of gene*/
    rand_for_selecting_element.int_dist_ = new std::uniform_int_distribution<> (0, first_chromosome.size()-1);
    rand_for_create_child.int_dist_ = new std::uniform_int_distribution<> (0, 100);

    /*creating random generator for each variable and storing in a map*/
    for (const auto& variable : variable_boundaries) {
        auto var_name = variable.first;
        auto lower_bound = variable.second.first;
        auto upper_bound = variable.second.second;
        map_of_random_generators[var_name] = std::make_unique <RandomNumberGenerator> ();
        map_of_random_generators.at(var_name) -> double_dist_ = new std::uniform_real_distribution <>(lower_bound, upper_bound);
        std::printf("[genetic_alg] var_name(%s), LBound(%f), UBound(%f), is_double_dist_(%d), is_int_dist(%d)\n",
                    var_name.c_str(), lower_bound, upper_bound, (map_of_random_generators.at(var_name) -> double_dist_ != nullptr),
                    (map_of_random_generators.at(var_name) -> int_dist_ != nullptr));
    }
    // while(ros::ok());
    // diagnostic_msgs/DiagnosticStatus.msg
    filter_process_shutdown_pub_ = n_.advertise <diagnostic_msgs::DiagnosticStatus> ("filter_shutdown_info_topic", 5,false);  //if latched it will trigger exit when next time is running 
    run_states();
    std::printf("[genetic_alg] genetic_algorithm initiation successful. \n");
}




void genetic_algorithm::run_states (void) {
    ///TODO decide how to proceed after implementing eval and sim. Not sure if running speed requires
    int counter = 0;
    ros::Rate r((double) running_speed); //run in 10hz
    current_state_ = STATES::INIT_;
    try {
        while ( current_state_ != STATES::ERROR_ && ros::ok()) {
            if (current_state_ == STATES::INIT_) {
                /*inital population*/
                std::printf("[run_states] INIT: STATES::INIT_ (%i) \n", current_state_);
                // population_ptr.reset (new Population(Population_size)); //initial population
                baberpond_population.reset_population(Population_size);
                PopulatePopulation(baberpond_population);
                baberpond_population.save_poupulation_file(RESULT_DIR_, Population::STATE_TO_SAVE::NEWPOPULATION);
                current_state_ = STATES::EVAL_;
                std::printf("[run_states] END: STATES::INIT_. NEXT_STATE(%i) \n", current_state_);
            }
            else if (current_state_ == STATES::EVAL_) {
                /*evaluate using simulation*/
                std::printf("[run_states] INIT: STATES::EVAL_ (%i) \n", current_state_);
                ///TODO
                evaluatePopulation(baberpond_population);
                /* already saving inside the eval function because need to save if there was a error in filter files */
                current_state_ = STATES::INTER_POPULATION_;
                std::printf("[run_states] current_state_ == STATES::EVAL_, evaluating exit conditon \n");
                std::printf("[run_states] baberpond_population.getHighestFitnessIndividual(%f) > 0.8 \n", baberpond_population.getHighestFitnessIndividual());
                if(baberpond_population.getHighestFitnessIndividual() > 0.8) {
                    std::printf("[GOAL_REACHED] baberpond_population.getHighestFitnessIndividual(%f) > 0.8 \n", baberpond_population.getHighestFitnessIndividual());
                    current_state_ = GOAL_REACHED_;
                }
                std::this_thread::sleep_for(5s);//just give some time system cleanup
                std::printf("[run_states] END: STATES::EVAL_. NEXT_STATE(%i) \n", current_state_);
            }
            else if (current_state_ == STATES::INTER_POPULATION_) {
                /*create children from fittest*/
                std::printf("[run_states] INIT: STATES::INTER_POPULATION_ (%i) \n", current_state_);
                getNewBornChildrenFromPopulation(baberpond_population, top_selection);
                // population_ptr = std::make_unique <Population> (p2);                
                current_state_ = STATES::RECOMBINE_;
                std::printf("[run_states] END: STATES::INTER_POPULATION_. NEXT_STATE(%i) \n", current_state_);
            }
            else if (current_state_ == STATES::RECOMBINE_) {
                /*apply crossover operator*/
                std::printf("[run_states] INIT: STATES::RECOMBINE_ (%i) \n", current_state_);
                getCrossOverPopulation(baberpond_population, crossover_percentage);
                current_state_ = STATES::MUTATION_;
                std::printf("[run_states] END: STATES::RECOMBINE_. NEXT_STATE(%i) \n", current_state_);
            }
            else if (current_state_ == STATES::MUTATION_) {
                /*apply mutation for population*/
                std::printf("[run_states] INIT: STATES::MUTATION_ (%i) \n", current_state_);
                getMutatantPopulation(baberpond_population, mutating_percentage);
                current_state_ = STATES::EVAL_;
                std::printf("[run_states] END: STATES::MUTATION_. NEXT_STATE(%i) \n", current_state_);
                // if(++counter >2) throw std::out_of_range("[exit condition] fitness value reached.");//for testing purpose. limiting the runs
                // if(baberpond_population.getHighestFitnessIndividual() > 0.8) throw std::out_of_range("[exit condition] fitness value reached.");//for testing purpose. limiting the runs
            }
            else if (current_state_ == STATES::GOAL_REACHED_){
                std::printf("[run_states] INIT: STATES::GOAL_REACHED (%i) \n", current_state_);
                std::printf("[GOAL_REACHED] Population index(%i) Fitness level (%f)\n",baberpond_population.get_index(), baberpond_population.getHighestFitnessIndividual() );
                n_.~NodeHandle();
                ros::requestShutdown();
            }
            else {
                std::printf("[run_states] NO STATE (%i) \n", current_state_);
                throw std::invalid_argument("[run_states] Error. Invalid state... ");
            }
            r.sleep();
        }
    }
    catch (std::exception& e) {
        std::cout << "[run_states] Error in  state execution. step :"<< current_state_<< std::endl;
        std::cout << "[run_states] standard exception: " << e.what() << std::endl;
        std::cout << "[run_states] exiting from genetic algo....." << std::endl;
        n_.shutdown();
        // exit(0);
    }
}

/**RandomNumberGenerator Functions**/

double genetic_algorithm::RandomNumberGenerator::getRandDoubleValue (){
    return double_dist_->operator()(*gen_);
}

int genetic_algorithm::RandomNumberGenerator::getRandIntValue (){
    return int_dist_->operator()(*gen_);
}

/**Individual HUMAN FUNCTIONS START HERE**/

void genetic_algorithm::getCrossoverOperation (AncientHuman& a, AncientHuman& b){
    std::cout << "[getCrossoverOperation] intiating. a and b init copy \n";
    print_chromosomes(a);
    print_chromosomes(b);
    int cross_over_size = (a.chromosome.size() / 2);
    std::vector<int> range_for_gene_postions(a.chromosome.size());
    auto printVec = [&] () {
        std::cout << "range_for_gene_postions: ";
        for (auto i : range_for_gene_postions) {
            std::cout << " " << i;
        }
        std::cout << std::endl;
    };
    std::iota(range_for_gene_postions.begin(), range_for_gene_postions.end(),0);
    printVec();
    std::shuffle(range_for_gene_postions.begin(), range_for_gene_postions.end(), *rand_for_selecting_element.gen_);
    printVec();
    for (int i = 0; i < cross_over_size; i++) {
        auto pos = range_for_gene_postions [i];
        std::swap (a.chromosome.at(pos), b.chromosome.at(pos));
    }
    std::cout << "[getCrossoverOperation] finished. no of crossedElements :"<<cross_over_size <<std::endl;
    print_chromosomes(a);
    print_chromosomes(b);
}

void genetic_algorithm::getMutedHuman (AncientHuman& g) {
    std::cout << "[getMutedGenome] intiating for gene: \n";
    print_chromosomes(g);
    auto pos = rand_for_selecting_element.getRandIntValue();
    try {
        g.chromosome.at(pos).second = map_of_random_generators.at(g.chromosome.at(pos).first) ->getRandDoubleValue();  
    }
    catch (...) {
        std::printf("[getMutedGenome] Error in populateGene fun. wrong variable name. exiting \n");
        throw std::invalid_argument("[getMutedGenome] step error. ");
    }
    std::cout << "Mututed :\n";
    print_chromosomes(g);
    printf("[getMutedGenome] completed. change pos(%i), value (%f) \n", pos, g.chromosome.at(pos).second);
}

void genetic_algorithm::CreateHuman (AncientHuman& g) {
    std::cout << "[CreateHuman] inside the CreateHuman \n";
    try {
            for (auto& v : g.chromosome) {
                v.second = map_of_random_generators.at(v.first) ->getRandDoubleValue(); 
            }    
        }
    catch (...) {
        std::printf("[genetic_algo] Error in CreateHuman fun. wrong variable name. exiting \n");
        throw std::invalid_argument("[CreateHuman] step error. ");
    }
    // print_chromosomes(g);
}

AncientHuman genetic_algorithm::getCreateChild (AncientHuman& a, AncientHuman& b) {
    std::cout << "[getCreateChild] creating child. parents a and b \n";
    print_chromosomes(a);
    print_chromosomes(b);
    AncientHuman child;
    std::cout << "random val >50(select a) <50(select b):";
    for (int i = 0; i < child.chromosome.size(); i++ ) {
        auto value = rand_for_create_child.getRandIntValue();
        std::cout << value <<" ";
        if (value > 50) {
            child.chromosome.at(i).second = a.chromosome.at(i).second; 
        }
        else {
            child.chromosome.at(i).second = b.chromosome.at(i).second; 
        }
    }
    std::cout << "\n[getCreateChild] child: \n";
    print_chromosomes(child);
    return child;
}

/**POPULATION  FUNCTIONS START HERE**/

void  genetic_algorithm::PopulatePopulation (Population& p) {
    std::printf("[PopulatePopulation] populating the residents \n");
    for (auto& h : p.residents_list) {
        CreateHuman(h);
    }
    std::printf("[PopulatePopulation] populating the residents ended\n");
}

void genetic_algorithm::getMutatantPopulation (Population& p, int percentage) {
    auto sz = p.getSize();
    auto no_of_persons_muted = int (std::round(sz * percentage / 100));
    std::printf("[getMutatantPopulation] mutating the residents. populatin size(%i) mutating size(%i)\n",
                sz, no_of_persons_muted);
    std::vector<int> range_for_gene_postions(sz);
    auto printVec = [&] () {
        std::cout << "range_for_gene_postions: ";
        for (auto i : range_for_gene_postions) {
            std::cout << " " << i;
        }
        std::cout << std::endl;
    };
    std::iota(range_for_gene_postions.begin(), range_for_gene_postions.end(),0);
    printVec();
    std::shuffle(range_for_gene_postions.begin(), range_for_gene_postions.end(), *rand_for_selecting_element.gen_);
    printVec();
    for (int i = 0; i < no_of_persons_muted; i++) {
        getMutedHuman(p.residents_list.at(i));
    }
    /*saving the population*/
    p.save_poupulation_file(RESULT_DIR_, Population::STATE_TO_SAVE::MUTATION);
    std::printf("[getMutatantPopulation] Mutation ended.\n");
}

void genetic_algorithm::getCrossOverPopulation (Population& p, int percentage) {
    auto sz = p.getSize();
    auto no_of_persons_muted = int (std::round(sz * percentage / 100));
    no_of_persons_muted = no_of_persons_muted == 0 ? 1 : no_of_persons_muted;
    std::printf("[getCrossOverPopulation] mutating the residents. populatin size(%i) crossover size(%i)\n",
                sz, no_of_persons_muted);
    std::vector<int> range_for_gene_postions(sz);
    auto printVec = [&] () {
        std::cout << "range_for_gene_postions: ";
        for (auto i : range_for_gene_postions) {
            std::cout << " " << i;
        }
        std::cout << std::endl;
    };
    std::iota(range_for_gene_postions.begin(), range_for_gene_postions.end(),0);
    printVec();
    std::shuffle(range_for_gene_postions.begin(), range_for_gene_postions.end(), *rand_for_selecting_element.gen_);
    printVec();
    for (int i = 0; i < no_of_persons_muted; i++) {
        getCrossoverOperation(p.residents_list.at(i), p.residents_list.at(i+1));
    }
    /*saving the population*/
    p.save_poupulation_file(RESULT_DIR_, Population::STATE_TO_SAVE::RECOMBINE);
    std::printf("[getCrossOverPopulation]  mutating ended.\n");
}

void genetic_algorithm::evaluatePopulation (Population& p) {
    std::printf("[evaluatePopulation] population evaluating started \n");
    try {
        std::mutex eval_populatin_lock;
        // std::condition_variable cv;
        std::string population_index (std::to_string(p.get_index()));
        auto pop_sz = p.getSize();
        std::unique_ptr<std::vector <std::future<int>* > > list_of_async_obj (new std::vector< std::future<int>* >);
        list_of_async_obj->reserve(pop_sz);
        // std::vector <std::promise<int> > list_of_promise;
        // list_of_promise.reserve(pop_sz);
        // std::vector <std::future<int>* > list_of_promise_future;
        // list_of_promise_future.reserve(pop_sz);
        for (int i = 0; i < p.residents_list.size(); i++) {
            std::string name = population_index + "_" + std::to_string(i);
            std::printf("[evaluatePopulation] Thread is initiating (%s) \n", name.c_str()); ///TODO add the variabels here
            // std::vector<std::tuple<std::string, double> > var_tuple = p.residents_list.at(i).getVariablesInTuple();
            // list_of_promise.push_back(std::promise<int>());
            // // std::promise <int> f_promise;
            // list_of_promise_future.push_back(new auto ( list_of_promise.at(i).get_future()));
            // // list_of_promise_future[i] = f_promise.get_future();
            // // auto f_future = f_promise.get_future();
            // std::printf("[evaluatePopulation] list_of_promise_future[i] = list_of_promise[i].get_future() \n"); ///TODO add the variabels here
            // list_of_async_obj.push_back( new auto(std::async(std::launch::async, 
            //                                     [&] {bloom_filter::parallel_filter(/*n_,*/ name, var_tuple, /*std::move(f_promise)*/std::move(list_of_promise[i]));}
            //                                   )));
            // std::printf("[evaluatePopulation] Waiting to thread to send the promise msg \n");
            // auto ret = list_of_promise_future.at(i)->get();
            // // auto ret = f_future.get();
            // // auto ret = list_of_promise_future.at(i);
            // // auto r = ret.get();

            auto var_list_in_string = p.residents_list.at(i).getVariableInString();
            //x-terminal-emulator -u -e  //if you want to spawn a terminal
            //cmd: roslaunch --log filter_object filter_object.launch list:="['tempreture','34.915597','ps','0.038237','pd','0.230078','wind_speed','14.200533','wind_heading','286.333232']" ns:="0_1"
            std::string cmd = "roslaunch --log filter_object filter_object.launch list:=\"[" + var_list_in_string +"]\" " + "ns:=\"" + name + "\"" ;
            std::printf("[evaluatePopulation] cmd: %s \n", cmd.c_str());
            // auto* future_return = new auto ( std::async(std::launch::async, &std::system, cmd.c_str() ));
            list_of_async_obj->push_back( new auto ( std::async(std::launch::async, &std::system, cmd.c_str()) ) );
            // list_of_async_obj->back()->wait_for(std::chrono::seconds(5));
            std::this_thread::sleep_for(5s);//just give some time to start the launch
            std::printf("[evaluatePopulation] thread is ready (%s). received promise () \n", name.c_str());
        }
        std::printf("[evaluatePopulation] rosbag is intiating. \n");
        auto bag_future = async(std::launch::async, &bag_player::play , bagPlayerPtr.get());
        std::printf("[evaluatePopulation] rosbag is running. waiting for it to finish \n");
        bag_future.get(); 
        std::cout <<"[evaluatePopulation] robag running completed. sending exit signal " << std::endl;
        // pf_static_member_setter.set_exiting_condition_true();
        ///TODO now implement how to read the results   
        // while(ros::ok());
        PassFilterShutDownSignal();
        std::cout <<"[evaluatePopulation] for (auto const& f : list_of_async_obj) " << std::endl;
        for (auto const& f : *list_of_async_obj) {
            auto isAsyncReady = f->valid();
            if(!isAsyncReady) {
                std::cout <<"[evaluatePopulation] future is not valid. Waiting for 5sec  "<< std::endl;
                f->wait_for(5s);
                std::cout <<"[evaluatePopulation] 5 second waited.Ignoring the state "<< std::endl;
            }
            else {
                std::cout <<"[evaluatePopulation] f.wait() waiting. value:  "<< f->get() << std::endl;
            }
        }
        std::cout <<"[evaluatePopulation] future wait done " << std::endl;
        std::printf("[evaluatePopulation] deleting the futures stored in unique pointer. to check malloc_consolidate(): invalid chunk size\n");
        list_of_async_obj.~unique_ptr();
        // for (auto i = 0; i < list_of_async_obj.size(); i++) {
        //     std::printf("[evaluatePopulation] (%i) list_of_async_obj.at(i)->get(); \n",i);
        //     list_of_async_obj.at(i)->get();
        // }
        p.calculateFitness(GROUND_TRUTH_DIR_, RESULT_DIR_);
        std::printf("[evaluatePopulation] population evaluating Completed \n");
    }
    catch (const std::future_error& e) {
        std::cout << "[evaluatePopulation] Caught a future_error with code \"" << e.code()
                  << "\"\n[evaluatePopulation] Message: \"" << e.what() << "\"\n";
    }
    catch(const std::exception& e) {
        std::cerr << e.what() << '\n';
    }
}

// void genetic_algorithm::calculateFitness (Population& p) {

// }

/*create children from population*/
/*1. pass fittest to next 
2.create children based on fittest*/
void genetic_algorithm::getNewBornChildrenFromPopulation (Population& p, int Fit_percentage) {
    Population newpopulation;
    newpopulation.set_index(p.get_index()); //copying prior index to new population
    auto sz = p.getSize();
    auto no_of_persons_fittest = int (std::round(sz * Fit_percentage / 100));
    no_of_persons_fittest = no_of_persons_fittest == 0 ? 1 : no_of_persons_fittest;
    auto top_selection_to_breed = int (std::round(sz * top_selection / 100));
    top_selection_to_breed = top_selection_to_breed == 0 ? 1 : top_selection_to_breed;
    std::printf("[getNewBornChildrenFromPopulation] population size(%i) fit pass count(%i), top_selection_to_breed(%i)\n",
                                                        sz, no_of_persons_fittest, top_selection_to_breed);
    p.sortDecendingOrderOfHumanFitness();
    std::cout << "[getNewBornChildrenFromPopulation] fittest passed to next population : \n";
    /*passing fittest to next generation based on percentage*/
    for (int i = 0; i < no_of_persons_fittest; i++) {
        newpopulation.residents_list.push_back(p.residents_list.at(i));
        print_chromosomes(newpopulation.residents_list.back());
    }
    /*calculting required remaing children*/
    auto remaining_population = sz - no_of_persons_fittest;
    /*selecting parents using random generator*/
    static std::random_device rd;  // a seed source for the random number engine
    static std::mt19937 gen(rd()); // mersenne_twister_engine seeded with rd()
    static std::uniform_int_distribution<> distrib(0, top_selection_to_breed);
    std::cout << "[getNewBornChildrenFromPopulation] creating childrend count : " << remaining_population <<std::endl;
    for (int i = 0; i < remaining_population; i++) {
        auto parent1 = distrib(gen);
        auto parent2 = distrib(gen);
        // if (parent1 == parent2) {parent2 = distrib(gen);}
        std::printf ("parents a(%i) b(%i): \n", parent1, parent2);
        auto child = getCreateChild(p.residents_list.at(parent1), p.residents_list.at(parent2));
        newpopulation.residents_list.push_back(child);
    }
    std::cout << "[getNewBornChildrenFromPopulation] End of creating children\n";
    p.ReplaceResidents(newpopulation.residents_list);
    /*saving the population*/
    p.save_poupulation_file(RESULT_DIR_, Population::STATE_TO_SAVE::NEWPOPULATION);
    std::cout << "[getNewBornChildrenFromPopulation] End of creating new population\n";
}

/**UTILS FUNC**/
void genetic_algorithm::print_chromos(chrom_dtype c) const {
    std::cout << "[chromosome] ";
    for (auto v : c) {
        std::cout << v.first << "(" << v.second << ") ";
    }
    std::cout << std::endl;
};

/**Destructors**/
genetic_algorithm::~genetic_algorithm() {
    std::printf("[genetic_algo] ~genetic_algorithm() is called\n");
};

genetic_algorithm::RandomNumberGenerator::~RandomNumberGenerator (){
    std::printf("[genetic_alg] ~RandomNumberGenerator ()\n");
    delete gen_;
    delete int_dist_;
    delete double_dist_;
};

void genetic_algorithm::PassFilterShutDownSignal() {
    diagnostic_msgs::DiagnosticStatus msg;
    msg.hardware_id = "genetic_algo_parent_inform";
    msg.name = "Filter_subprocess_shutter";
    msg.message = "shutdown";
    filter_process_shutdown_pub_.publish(msg);
}

/*testing for each function*/
void genetic_algorithm::test_function () {
    // AncientHuman c3, c4;
    // CreateHuman(c3);
    // CreateHuman(c4);
    // getCrossoverOperation(c3, c4);
    // getMutedHuman(c3);
    // AncientHuman c5 = getCreateChild(c3,c4);
    // // p1.reset_population (10);
    // // PopulatePopulation(p1);
    // // static int counter = 0;
    // // for (auto& v : p1.residents_list) {
    // //     v.fitness = ++counter;
    // // }
    // // p1.print();
    // // p1.sortDecendingOrderOfHumanFitness();
    // // p1.print();
    // // getMutatantPopulation(p1, 10);
    // // getCrossOverPopulation(p1, 10);
    // // auto p2 = getNewBornChildrenFromPopulation(p1, 10);
    // // auto p3 = getNewBornChildrenFromPopulation(p2, 10);
    // p1.reset_population (5);
    // PopulatePopulation(p1);
    // evaluatePopulation(p1);
    run_states();
    




}