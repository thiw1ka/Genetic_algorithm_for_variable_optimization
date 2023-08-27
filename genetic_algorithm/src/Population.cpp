#include "genetic_algorithm/Population.hpp"
/*static variables*/
int Population::index = 0;

Population::Population() {
    std::printf("[Population] Population constructed \n");
}

Population::Population (int size) {
    std::printf("[Population] Population constructed. population size(%i) \n", size);
    // index += index;
    residents_list.reserve(size);
    setPopulation(size);
}

void Population::setPopulation (int size) {
    AncientHuman h1;
    for (int i = 0; i < size; i++) {
        residents_list.push_back(h1);
    }
}

void  Population::reset_population (int size) {
    /*making sure population is reset before adding new*/
    std::printf("[population] population new size: (%i). \n", size);
    residents_list.clear();
    setPopulation (size);
}

void Population::incrementIndex(){
    ++index;
    std::printf("[Population::incrementIndex] index incrementd from (%i) --> to(%i) \n", index-1, index);
}

void Population::ReplaceResidents (std::vector <AncientHuman>& newResidents) {
    std::printf("[ReplaceResidents]  replacing residents in population ID(%i) \n", index);
    this->residents_list = newResidents;
    incrementIndex();//increment the population index
    std::printf("[ReplaceResidents]  replacing residents done new ID(%i) \n", index);
}

int Population::getSize () {
    return residents_list.size();
}

int Population::get_index() {
    return index;
}

void Population::set_index(int ind) {
    this->index = ind;
}

void Population::updatePopulationFitness() {
    population_total_fitness_ = population_total_fitness_ / getSize ();
    std::printf("[updatePopulationFitness] Population fitness is %f \n", population_total_fitness_);
}


void Population::sortDecendingOrderOfHumanFitness() {
    //decending order
    std::printf("[population] sorting population in decending order. \n");
    std::sort(residents_list.begin(), residents_list.end(),
        [&](AncientHuman& a, AncientHuman& b) {
            if (a.fitness ==0 && b.fitness ==0) return true;
            return a.fitness > b.fitness;}); 
    std::printf("[population] sorting population ended. \n");
}

void Population::print() const {
    std::cout << "[population] printing population \n";
    for (auto const& human : residents_list) {
        print_chromosomes(human);
    }
    std::cout << std::endl;
}

Population Population::operator= (Population& other) {
    residents_list.clear();
    residents_list.reserve(other.getSize());
    index = other.get_index();
    for (auto const& human : other.residents_list) residents_list.push_back(human);
}

Population Population::operator+ (Population& other) {
    auto newsize = getSize() + other.getSize(); 
    residents_list.reserve(newsize);
    for (auto const& human : other.residents_list) residents_list.push_back(human);
}

double Population::getHighestFitnessIndividual () {
    return this->highest_fitness_;
}


void Population::calculateFitness(std::string groundTruthDir, std::string filterOutputDir) {
    std::printf("[Population::calculateFitness] calculation fitness for population(%i) \n", get_index());
    auto population_no = get_index();
    auto population_sz = getSize();
    /*Selecting the appropriate population folder*/
    std::string population_dir = filterOutputDir + "/" + std::to_string(population_no) + "/";

    /*reading ground truth*/
    bloom_filter::readfile gtruth_file, output_file;
    auto is_gtr_successed = gtruth_file.updateGroundTruthFiles(groundTruthDir);
    std::printf("[calculateFitness] Reading ground truth folder successed? (%i) \n", int(is_gtr_successed));
    if(!is_gtr_successed) exit(0);
    int GroundTruthFilecount = gtruth_file.get_filter_timestep_count ();

    /*Initiation evaluator class with canvas*/
    IntersectionOverUnion evaluator;
    evaluator.setAreaOfImage(800, 800); //setting the canvas size

    population_total_fitness_ = 0.0;
    bool is_filter_output_folder_exists = true;
    /*iterating through each human in the population*/
    for (int i = 0; i < population_sz; i++) {
        std::printf("[calculateFitness] calculating fitness for human (%i) \n", i);
        int human_ID = i;
        residents_list.at(human_ID).fitness = 0.0;
        /*setting path for each human*/
        population_dir = filterOutputDir + std::to_string(population_no) + "_" + std::to_string(i) + "/";
        is_filter_output_folder_exists = output_file.SetPopulationFiles(population_dir);
        if (!is_filter_output_folder_exists) {
            std::printf("[calculateFitness] Reading filter output not successed (%i) for Human ID (%i). check for population ID (%i), (%i) folders of results exists? \n", 
                                                    int(is_filter_output_folder_exists), human_ID, population_no, population_sz);
            // throw std::out_of_range("[calculateFitness] Couldnt find correct no of filter output folders for all members in the population.");
            continue; // moving on to next file
        }
        /*testing if ground truth count and results mathch*/
        if (GroundTruthFilecount != output_file.get_filter_timestep_count()) {
            std::printf("[calculateFitness] Error. ground truth and filter output dont match. groundtruth(%i) == filterOutput(%i) \n",
                                                                        GroundTruthFilecount, output_file.get_filter_timestep_count());
            continue; //moving on to next one
            //throw std::invalid_argument("[calculateFitness] Error. ground truth and filter output dont match.");
        }
        /*For each Human -> iterating through all filter runs*/
        std::printf("[calculateFitness] initating for loop to go ground truth and result file to calculate IOU\n");
        for (int j = 1; j < GroundTruthFilecount; j++) {
            std::printf("Human (%i), filter run number = %i\n", human_ID, j);
            //file_types: "time_update", "meas_update", "all_cam_meas_points", "mu_weight_norm_with_added_noise", for groundtruth use ->"time_update"
            auto groundtruth = gtruth_file.get_population_next_file("time_update",j);
            auto filterOutput = output_file.get_population_next_file("meas_update",j);
            residents_list.at(human_ID).fitness += evaluator.getIoUForTwoPointclouds(filterOutput, groundtruth,cv::Point(800,800), 10, false);
        }
        /*averaging fitness value by runs*/
        residents_list.at(i).fitness = residents_list.at(i).fitness / GroundTruthFilecount;
        highest_fitness_ = std::max(highest_fitness_, residents_list.at(i).fitness);
        population_total_fitness_ += residents_list.at(i).fitness;
        std::printf("[calculateFitness] for loop ended calcultaing fitness for Human ID(%i), final fitness is (%f) averaged by filter runs , population fitness sum (%f) \n", 
                                                                    human_ID, residents_list.at(i).fitness, population_total_fitness_);
    }
    updatePopulationFitness();
    save_poupulation_file(filterOutputDir, STATE_TO_SAVE::EVAL);
    /*letting main program know one folder is missing*/
    // if (!is_filter_output_folder_exists) throw std::out_of_range("[calculateFitness] Couldnt find correct no of filter output folders for all members in the population.");
    std::printf("[calculateFitness] population index (%i) Overall fitness value (%f), Averg_Population_fitness(%f) \n",
                                                                        population_no,  population_total_fitness_, population_total_fitness_);
    ///TODO normalize each resident fitness
    ///TODO SAVE Population details
}

void Population::save_poupulation_file (std::string directory_Path, STATE_TO_SAVE ss) {
    std::printf("[save_poupulation_file] Saving population file started. Population ID(%i) \n", this->index);
    string saving_state = "";
    switch (ss)
    {
    case STATE_TO_SAVE::NEWPOPULATION :
        std::printf("[save_poupulation_file] Saving state NEWPOPULATION \n");
        saving_state = "new_population";
        break;
    case STATE_TO_SAVE::RECOMBINE :
        std::printf("[save_poupulation_file] Saving state RECOMBINE \n");
        saving_state = "recombine";
        break;
    case STATE_TO_SAVE::MUTATION :
        std::printf("[save_poupulation_file] Saving state MUTATION \n");
        saving_state = "mutation";
        break;
    case STATE_TO_SAVE::EVAL :
        std::printf("[save_poupulation_file] Saving state EVAL \n");
        saving_state = "evaluated";
        break;    
    default:
        std::printf("[save_poupulation_file] ERROR incorrect saving state \n");        
        saving_state = "INCORRECT_STATE";
        break;
    }
    int population_sz = residents_list.size();
    experimental::filesystem::path dir_Path_ = directory_Path;
    //checking the path
    if(!experimental::filesystem::exists(dir_Path_)){
                std::cerr << "[readfile] Directory path not valid. \npath: "<<dir_Path_<<std::endl;
                std::cerr << "[readfile] saving in default location: /home/..."<<std::endl;
                dir_Path_ = "dir_Path_";
    }
    std::cout<<"[save_poupulation_file] Directory path valid."<<std::endl;
    /*creating a folder name Populations*/
    dir_Path_ += "Populations/";
    //checking if the folder already exsits or not
    if(std::experimental::filesystem::create_directory(dir_Path_)){
        std::printf("[save_poupulation_file] folder successfully created \n");
    }
    else {
        std::printf("[save_poupulation_file] folder exists. \n");
        // throw std::invalid_argument("[readfile::createFolder] unsuccessfull in creating folder");;
    }
    dir_Path_ += "population-" + std::to_string(this->index) +"-" + saving_state+".txt";
    auto path = dir_Path_.generic_string();
    std::fstream file (dir_Path_.generic_string(), std::fstream::out | std::fstream::app); //writing at the end
    cout << "file_path:" << dir_Path_.generic_string() << std::endl;
    if (!file.is_open())
        std::cout << "[save_poupulation_file] failed to open " << dir_Path_.generic_string() << '\n';
    else {
        std::printf("[save_poupulation_file] file opened successfully \n");
        auto t_c = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        file <<"\npopulation_index-"<<this->index <<"-population_"<<population_total_fitness_<<"_saved at "<< std::ctime(&t_c)<<std::endl;
        for (int i = 0; i < population_sz; i++) {
            file<< i <<" :" << residents_list.at(i).getVariableToSave() << std::endl;
            // cout << "Success "<<i <<" :" << residents_list.at(i).getVariableToSave() << std::endl;
        }
    }
    cout <<"success: population_index:"<<this->index <<":saved at "<<std::endl;
    file.close();
    std::cout << std::boolalpha <<"file close successfully? "<< !file.is_open() << '\n';
    std::printf("[save_poupulation_file] Saved. Population ID(%i) \n", this->index);
}

// void Population::populate () {
//     std::printf("[PopulatePopulation] populating the residents \n");
//     for (auto& h : residents_list) {
//         genetic_algorithm::CreateHuman(h);
//     }
//     std::printf("[PopulatePopulation] populating the residents ended\n");
// }

Population::~Population() {
    printf("[Population] called ~Population()");
    // residents_list.~vector();
};