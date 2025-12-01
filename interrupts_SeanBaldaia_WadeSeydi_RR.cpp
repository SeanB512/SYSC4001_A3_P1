/**
 * @file interrupts.cpp
 * @author Sasisekhar Govind
 * @brief template main.cpp file for Assignment 3 Part 1 of SYSC4001
 * 
 */

#include "interrupts_SeanBaldaia_WadeSeydi.hpp"

void FCFS(std::vector<PCB> &ready_queue) {
    std::sort( 
                ready_queue.begin(),
                ready_queue.end(),
                []( const PCB &first, const PCB &second ){
                    return (first.arrival_time < second.arrival_time); 
                } 
            );
}

void RR(std::vector<PCB> &ready_queue, std::vector<PCB> &new_arrivals, PCB &running){
    FCFS(new_arrivals);
    std::vector<PCB> new_queue;

    if (running.state != TERMINATED){
        new_queue.push_back(running);
    }
    
    while (new_arrivals.size() > 0){
        new_queue.push_back(new_arrivals.back());
        new_arrivals.pop_back();
    }

    for (auto current: ready_queue){
        new_queue.push_back(current);
    }
    
    ready_queue = new_queue;
}

std::tuple<std::string /* add std::string for bonus mark */ > run_simulation(std::vector<PCB> list_processes) {

    std::vector<PCB> ready_queue;   //The ready queue of processes
    std::vector<PCB> wait_queue;    //The wait queue of processes
    std::vector<PCB> job_list;      //A list to keep track of all the processes. This is similar
                                    //to the "Process, Arrival time, Burst time" table that you
                                    //see in questions. You don't need to use it, I put it here
                                    //to make the code easier :).

    unsigned int current_time = 0;
    PCB running;

    //Initialize an empty running process
    idle_CPU(running);

    std::string execution_status;

    //make the output table (the header row)
    execution_status = print_exec_header();

    std::vector<PCB> new_arrivals;          //new arrivals to be added to ready_queue

    //Loop while till there are no ready or waiting processes.
    //This is the main reason I have job_list, you don't have to use it.
    while(!all_process_terminated(job_list) || job_list.empty()) {

        //Inside this loop, there are three things you must do:
        // 1) Populate the ready queue with processes as they arrive
        // 2) Manage the wait queue
        // 3) Schedule processes from the ready queue

        //Population of ready queue is given to you as an example.
        //Go through the list of proceeses
        for(auto &process : list_processes) {
            if(process.arrival_time == current_time) {//check if the AT = current time
                //if so, assign memory and put the process into the ready queue

                
                if (assign_memory(process)){
                    process.state = READY;  //Set the process state to READY
                    execution_status += print_exec_status(current_time, process.PID, NEW, READY);
                    new_arrivals.push_back(process);

                    
                } else{                     //if necessary memory cannot be allocated
                    process.state = WAITING;
                    wait_queue.push_back(process);
                    execution_status += print_exec_status(current_time, process.PID, NEW, WAITING);
                }
                job_list.push_back(process); //Add it to the list of processes                
            }
        }

        ///////////////////////MANAGE WAIT QUEUE/////////////////////////
        //This mainly involves keeping track of how long a process must remain in the ready queue

        //check if processes in wait queue are finished waiting
        for (auto counter = 0; counter < wait_queue.size(); counter++){
            auto current = wait_queue.at(counter);
            bool flag = false;
            
            if (current.partition_number < 0){
                if (assign_memory(current)){
                    flag = true;
                }
            } else if (current_time == current.start_time + current.io_duration + current.io_freq){
                flag = true;
            }

            if (flag){
                current.state = READY;  //Set the process state to READY
                wait_queue.erase(wait_queue.begin() + counter);
                counter--;

                execution_status += print_exec_status(current_time, current.PID, WAITING, READY);
            }
        }

        //check if running process is finished running
        if (running.remaining_time <= 0){

            running.state = TERMINATED;
            terminate_process(running, job_list);

            //prevents idle_CPU from appearing in execution.txt
            if (running.PID >= 0){
                execution_status += print_exec_status(current_time, running.PID, RUNNING, TERMINATED);
            }
        }

        //check if RR quantum expired
        else if (current_time - running.start_time == 100){
            running.state = READY;

            execution_status += print_exec_status(current_time, running.PID, RUNNING, READY);
        }

        //check if running process is kicked due to I/O
        else if (running.io_freq > 0 && running.state == RUNNING){
            if ((running.processing_time -  running.remaining_time) % running.io_freq == 0){
                //if so, prepare to kick out process
                running.state = WAITING;
                wait_queue.push_back(running);

                execution_status += print_exec_status(current_time, running.PID, RUNNING, WAITING);
            }
        }

        /////////////////////////////////////////////////////////////////

        //////////////////////////SCHEDULER//////////////////////////////
        
        //check if current process is still running
        if (running.state != RUNNING){
            RR(ready_queue, new_arrivals, running);                                            //reorganize ready queue
            if (ready_queue.size() > 0){
                run_process(running, job_list, ready_queue, current_time);  //run process
                if (running.PID >= 0){

                    execution_status += print_exec_status(current_time, running.PID, READY, RUNNING);
                }
                
            }
        }

        /////////////////////////////////////////////////////////////////

        //increment timer by one
        if (running.state == RUNNING){
            running.remaining_time -= 1;
        }
        current_time++;
    }
    
    //Close the output table
    execution_status += print_exec_footer();

    return std::make_tuple(execution_status);
}


int main(int argc, char** argv) {

    //Get the input file from the user
    if(argc != 2) {
        std::cout << "ERROR!\nExpected 1 argument, received " << argc - 1 << std::endl;
        std::cout << "To run the program, do: ./interrutps <your_input_file.txt>" << std::endl;
        return -1;
    }

    //Open the input file
    auto file_name = argv[1];
    std::ifstream input_file;
    input_file.open(file_name);

    //Ensure that the file actually opens
    if (!input_file.is_open()) {
        std::cerr << "Error: Unable to open file: " << file_name << std::endl;
        return -1;
    }

    //Parse the entire input file and populate a vector of PCBs.
    //To do so, the add_process() helper function is used (see include file).
    std::string line;
    std::vector<PCB> list_process;
    while(std::getline(input_file, line)) {
        auto input_tokens = split_delim(line, ", ");
        auto new_process = add_process(input_tokens);
        list_process.push_back(new_process);
    }
    input_file.close();

    //With the list of processes, run the simulation
    auto [exec] = run_simulation(list_process);

    write_output(exec, "execution_RR.txt");

    return 0;
}