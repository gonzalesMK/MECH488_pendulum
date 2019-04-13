// include necessary files from MEL library
#include <MEL/Core.hpp>
#include <MEL/Mechatronics.hpp>
#include <MEL/Math.hpp>
#include <MEL/Daq/NI/MyRio/MyRio.hpp>
#include <MEL/Communications/MelNet.hpp>
#include <MEL/Logging/Csv.hpp>

// use mel namespace so we can say foo() instead of mel::foo()
using namespace mel;

//=============================================================================
// GLOBALS AND FREE FUNCTIONS
//=============================================================================

// global stop flag
ctrl_bool STOP(false);

PidController pid(1,0.1,0.1);

// control handler function, called when Ctrl+C is pressed in terminal
bool my_handler(CtrlEvent event) {
    if (event == CtrlEvent::CtrlC || event == CtrlEvent::Close) {
        STOP = true;
        print("Application Terminated");
    }
    return true;
}

//=============================================================================
// MAIN
//=============================================================================

// main(), the entry point for all C/C++ programs (all code starts here!)
int main() {
    // register our handler so its called when ctrl+c is pressed
    register_ctrl_handler(my_handler);
    // create a myRIO object
    MyRio myrio;
    // open communication with myRIO FPGA
    myrio.open();
    // enable myRIO to set intial values
    myrio.enable();
    // enable encoder 0 (A = DIO 0, B = DIO 2)
    myrio.mspC.encoder.enable_channel(0);
    // set units per count on encoder (get_position() will return radians, make 360.0/500.0 if degrees prefered)
    myrio.mspC.encoder[0].set_units_per_count(2 * PI / 500.0);
    // zeros the encoder, meaning that its CURRENT position is 0 radians
    myrio.mspC.encoder[0].zero();
    // configure MSPC DIO[1] to be output (all DIOs are inputs by default)
    myrio.mspC.DIO[1].set_direction(Out);
    // create MelNet so we can stream data to host PC
    MelNet mn(55002, 55001, "172.22.11.1");
    // create your constants that you will use for control here
    const double k_theta = 6.3;
    const double k_omega = 0.01 ;
    // determining the Kd, Kp, & Ki values. the constants used to PID Control
    const double kd = k_omega;
    const double kp = k_theta;
    const double ki=0 ;
    const double period = 0.0001 ;
    // initiallizing the errors used for PD control
    double last_error =0;
    double last_last_error = 0;
    double last_control = 0;
    double last_last_control = 0;
    //Writing the equations needed for solving PID Control. 
    const double A = kp + 2*kd/period + period*ki/2;
    const double B = (period)*ki - 4*(kd/period);
    const double C = 2*(kd/period) + (period)*(ki/2) - kp;

    double position;     
    double error;
    double control; 
    
    // create Timer for our control loop
    Timer timer(hertz(1000));
    // create a Time point t
    Time t = Time::Zero;
    // create a second order butterworth filter to filter velocity
    Butterworth butt(2,hertz(10),hertz(1000));
    // headers for the csv file for datalogging
    std::vector<std::string> header = {"Time (s)", "Write (V)", "Read (V)"};
    // path to csv file (absolute_folder will be created in system root)
    std::string filepath = "/home/admin/MECH488/group9.csv";
    // vector of double vectors which will hold information to write
    std::vector<std::vector<double>> data;
    // start and run our control loop until STOP is true
    while(!STOP) {

        // get real world input values
        myrio.update_input();

        // --------------------------------------------------------------------
        // !!! BEGIN YOUR CONTROL IMPLEMENTATION !!!
        // --------------------------------------------------------------------
        //
        // The code here is not what you will use for you project,
        // but can serve as an example for implementation
        //

        // Get position of the pendulum 
        position = myrio.mspC.AI[1];
        
        // Calculate the error
        error =10*(3.5 - position); // desired position = 0

        // Calculate control
        control = (1/A)* (last_last_error - error - B*last_control - C*last_last_control);
        // control = Kp * position + Kd * velocity 
        
        // Send control
        myrio.mspC.AO[0] = - 5* control;
        
        // Update past values
        last_last_error = last_error;
        last_error = error;
        last_last_control = last_control;
        last_control = control; 
        // analog loopback (connect AO 0 to AI 0)

        // //PREVIOUS CODE MADE TO BE USED AS A GUIDELINE
        // double volts_write = std::sin(2 * PI * t.as_seconds());
        // myrio.mspC.AO[0] = volts_write;
        // double volts_read  = myrio.mspC.AI[0];

        // if (volts_read > 0)
        //     myrio.set_led(0, true);
        // else
        //     myrio.set_led(0, false);

        // // digital, button, led loop (connect DIO 1 to DIO 3)
        // if (myrio.is_button_pressed())
        //     myrio.mspC.DIO[1] = High;
        // else
        //     myrio.mspC.DIO[1] = Low; 

        // if (myrio.mspC.DIO[3] == High)
        //     myrio.set_led(3, true);
        // else
        //     myrio.set_led(3, false);

        // --------------------------------------------------------------------
        // !!! END YOUR CONTROL IMPLEMENTATION !!!
        // --------------------------------------------------------------------

        //append data to a vector to use for datalogging
        data.push_back({t.as_seconds(), control, error});
        // stream doubles of interest (add/remove variables if desired)
        mn.send_data({control, error, A, B, C, kp, kd, period});
        // set real world output values
        myrio.update_output();
        // wait timer and then get elapsed time
        t = timer.wait();

    }
    // write the header
    csv_write_row(filepath, header);    
    // append the data
    csv_append_rows(filepath, data);

    // return 0 to indicate success
    return 0;
}