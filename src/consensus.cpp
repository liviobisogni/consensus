//*****************************************************************************
//***********************         CONSENSUS.CPP         ***********************
//***********************     Author: Livio Bisogni     ***********************
//*********************** © 2021 Turtley & Turtles Ltd. ***********************
//*****************************************************************************

/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
                                  INSTRUCTIONS

    Please read the attached `README.md` file.
_____________________________________________________________________________*/


/*-----------------------------------------------------------------------------
-------------------------------------------------------------------------------
--------------------------------- HEADER FILES --------------------------------
-------------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include <termios.h>


/*-----------------------------------------------------------------------------
-------------------------------------------------------------------------------
------------------------------- GLOBAL CONSTANTS ------------------------------
-------------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

const int NUM_TURTLES = 5;          // turtles number; modify it
                                    // accordingly to "consensus.launch"
const double DT         = 0.00001;  // time step
const double KP_LINEAR  = 0.5;
const double KP_ANGULAR = 4.0;
const double TOLERANCE  = 0.001;           // linear error tolerance; once
                                           // linear error falls below this
                                           // threshold, the considered turtle
                                           // stops
const double CONSENSUS_TOLERANCE = 0.001;  // consensus tolerance; once
                                           // consensus error falls below this
                                           // threshold, the consensus algorithm
                                           // stops running
const int KEY_ESC = 27;                    // ASCII code equivalence
const int SECOND  = 1000000;  // number of microseconds in one second


/*-----------------------------------------------------------------------------
-------------------------------------------------------------------------------
------------------------------- GLOBAL VARIABLES ------------------------------
-------------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

// Poses
turtlesim::Pose currentPose[NUM_TURTLES + 1];  // NUM_TURTLES actual poses
turtlesim::Pose centroid[NUM_TURTLES + 1];     // NUM_TURTLES centroid poses

bool stop = false;  // flag to stop the considered turtle (1, ... , NUM_TURTLES)
                    // when it's set to 'true'
bool check[NUM_TURTLES + 1];  // used to check that all the turtles reach their
                              // goal poses
double u_x[NUM_TURTLES + 1];
double u_y[NUM_TURTLES + 1];
double x[NUM_TURTLES + 1];
double y[NUM_TURTLES + 1];
double x_temp[NUM_TURTLES + 1];
double y_temp[NUM_TURTLES + 1];
double x_old[NUM_TURTLES + 1];
double y_old[NUM_TURTLES + 1];


/*-----------------------------------------------------------------------------
-------------------------------------------------------------------------------
----------------------- CALLBACK & FUNCTION DEFINITIONS -----------------------
-------------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    GETCURRENTPOSE_CALLBACK:        Get i-th turtle current pose
_____________________________________________________________________________*/

void getCurrentPose_Callback(const turtlesim::Pose::ConstPtr &msg, int i)
{
    currentPose[i].x     = msg->x;
    currentPose[i].y     = msg->y;
    currentPose[i].theta = msg->theta;
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    GETCH:      Get pressed key character; non-blocking function.
                Code adapted from:

                https://answers.ros.org/question/63491/keyboard-key-pressed/
_____________________________________________________________________________*/

char getch()
{
    fd_set         set;
    struct timeval timeout;
    int            rv;
    char           buff     = 0;
    int            len      = 1;
    int            filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec  = 0;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
        ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN]  = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
        ROS_ERROR("tcsetattr ICANON");

    if (rv == -1)
        ROS_ERROR("select");
    else if (rv == 0) {
        // ROS_INFO("no_key_pressed"); // DEBUG
    } else
        read(filedesc, &buff, len);

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR("tcsetattr ~ICANON");
    return (buff);
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    EXECUTECONSENSUS:       Rendez-vous: k-th iteration for the i-th turtle.
                            It's gonna be iterated till:
                            consensusError <= CONSENSUS_TOLERANCE
_____________________________________________________________________________*/

void executeConsensus(int i, int k)
{
    u_x[i] = 0;
    u_y[i] = 0;

    // Consensus protocol
    for (int j = 1; j <= NUM_TURTLES; j++) {
        u_x[i] = u_x[i] - (x[i] - x[j]);
        u_y[i] = u_y[i] - (y[i] - y[j]);
    }

    // Integrate
    x_temp[i] = x_old[i] + u_x[i] * k * DT;
    y_temp[i] = y_old[i] + u_y[i] * k * DT;

    // Print various stuff (mainly for debugging)
    printf("i: %i\n", i);
    printf("x_old[%i]\t= %f\n", i, x_old[i]);
    printf("y_old[%i]\t= %f\n", i, y_old[i]);
    printf("x_new[%i]\t= %f\n", i,
           x_temp[i]);  // x_temp is the new x proposed
    printf("y_new[%i]\t= %f\n", i,
           y_temp[i]);  // y_temp is the new y proposed

    // Temporary variables (now, "temps are the new old")
    x_old[i] = x_temp[i];
    y_old[i] = y_temp[i];

    // usleep(3 * SECOND); // debug only
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    GETANGULARERROR:        Get angular error between direction of the turtle
                            and direction to the desired pose
_____________________________________________________________________________*/

double getAngularError(turtlesim::Pose current_pose, turtlesim::Pose goal_pose)
{
    // Create linear error vector
    double E_x = goal_pose.x - current_pose.x;  // Error along X component
    double E_y = goal_pose.y - current_pose.y;  // Error along Y component

    // Get desired angle
    double desired_theta = atan2(E_y, E_x);

    // Compute angular error
    double E_theta = desired_theta - current_pose.theta;

    return E_theta;
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    GET LINEAR ERROR:       Get linear error from the turtles perspective
_____________________________________________________________________________*/

double getLinearError(turtlesim::Pose current_pose, turtlesim::Pose goal_pose)
{
    // create error vector
    double E_x = goal_pose.x - current_pose.x;  // Error along X component
    double E_y = goal_pose.y - current_pose.y;  // Error along Y component

    // double E_lin = sqrt(pow(E_x, 2) + pow(E_y, 2));
    double E_lin = hypot(E_x, E_y);
    return E_lin;
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    INITIALIZEARRAYTOFALSE:     Set every element of the given logical array to
                                'false'
_____________________________________________________________________________*/

void initializeArrayToFalse(bool array[], int arrayLength)
{
    for (int i = 0; i < arrayLength; i++) {
        array[i] = false;
    }
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    CHECKARRAY:     Check every element of a given logical array is 'true'
_____________________________________________________________________________*/

bool checkArray(bool array[], int arrayLength)
{
    for (int i = 0; i < arrayLength; i++) {
        if (array[i] == false)
            return false;  // At least one element is 'false' ...
    }
    return true;  // ... otherwise they're all 'true'
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    MAXABSDIFF:     Return the maximum absolute difference between any two
                    elements of the array, except for the former
_____________________________________________________________________________*/

double maxAbsDiff(double array[], int arrayLength)
{
    // For storing the minimum and the maximum elements of the array
    double minElement = array[1];
    double maxElement = array[1];

    for (int i = 2; i < arrayLength; i++) {
        minElement = fmin(minElement, array[i]);
        maxElement = fmax(maxElement, array[i]);
    }
    return (maxElement - minElement);
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    COMPUTECENTROIDS:       Compute the centroid(s)
_____________________________________________________________________________*/

void computeCentroids()
{
    for (int i = 1; i <= NUM_TURTLES; i++) {
        centroid[i].x     = x[i];
        centroid[i].y     = y[i];
        centroid[i].theta = 0;
        printf("CENTROID(%i):\t(%f, %f)\n", i, centroid[i].x,
               centroid[i].y);  // DEBUG
    }
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    COMPUTECENTROIDTHEORETICAL:     Compute the theoretical centroid (based on
                                    the turtles initial positions)
_____________________________________________________________________________*/

void computeCentroidTheoretical()
{
    turtlesim::Pose centroidTheoretical;  // theoretical centroid pose

    centroidTheoretical.x = 0;
    centroidTheoretical.y = 0;
    for (int i = 1; i <= NUM_TURTLES; i++) {
        centroidTheoretical.x = (centroidTheoretical.x + currentPose[i].x);
        centroidTheoretical.y = (centroidTheoretical.y + currentPose[i].y);
    }
    centroidTheoretical.x = centroidTheoretical.x / NUM_TURTLES;
    centroidTheoretical.y = centroidTheoretical.y / NUM_TURTLES;

    printf("\n");
    printf("Theoretical centroid coordinates:\t(%f, %f)\n",
           centroidTheoretical.x, centroidTheoretical.y);
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    INIT:       Various initializations
_____________________________________________________________________________*/

void init(ros::NodeHandle nh)
{
    // Useful to give enough time to acquire the current turtles poses
    if (ros::ok() && nh.ok()) {
        usleep(2 * SECOND);
        ros::spinOnce();
        usleep(2 * SECOND);
    }

    printf("\n");
    for (int i = 1; i <= NUM_TURTLES; i++) {
        printf("Turtle %i initial position:\t(%f, %f)\n", i, currentPose[i].x,
               currentPose[i].y);
    }

    usleep(4 * SECOND);

    for (int i = 1; i <= NUM_TURTLES; i++) {
        x[i] = currentPose[i].x;
        y[i] = currentPose[i].y;

        x_old[i] = x[i];
        y_old[i] = y[i];

        x_temp[i] = x[i];
        x_temp[i] = y[i];
    }
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    MANAGECONSENSUSACHIEVED:        After consensus is achieved
_____________________________________________________________________________*/

void manageConsensusAchieved()
{
    printf("\n\n");

    for (int i = 1; i <= NUM_TURTLES; i++)
        printf("Turtle %i final position:\t(%f, %f)\n", i, currentPose[i].x,
               currentPose[i].y);

    usleep(1 * SECOND);
    printf("\n");
    printf("Consensus finally achieved! :)\n");

    usleep(3 * SECOND);
    printf("\n");
    printf("Shutting down the node ...\n");

    usleep(1 * SECOND);
    printf("\n");
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    ACHIEVECONSENSUS:       For achieving consensus
_____________________________________________________________________________*/

void achieveConsensus()
{
    double consensusError_x, consensusError_y, consensusError;

    consensusError_x = maxAbsDiff(x, NUM_TURTLES + 1);
    consensusError_y = maxAbsDiff(y, NUM_TURTLES + 1);
    consensusError   = fmax(consensusError_x, consensusError_y);

    // printf("\n");
    // for (int i = 1; i <= NUM_TURTLES; i++) {
    //  printf("x[%i] = %f\n", i, x[i]);
    // }
    // for (int i = 1; i <= NUM_TURTLES; i++) {
    //  printf("y[%i] = %f\n", i, y[i]);
    // }
    printf("Consensus Error:\t%f\n\n", consensusError);

    int k = 0;  // iteration index

    while (consensusError > CONSENSUS_TOLERANCE) {
        printf("***************************\n");
        printf("\tk = %i\n", k);
        for (int i = 1; i <= NUM_TURTLES; i++) {
            executeConsensus(i, k);
        }
        for (int i = 1; i <= NUM_TURTLES; i++) {
            x[i] = x_temp[i];
            y[i] = y_temp[i];
        }
        k++;

        for (int j = 2; j <= NUM_TURTLES; j++) {
            consensusError_x = maxAbsDiff(x, NUM_TURTLES + 1);
            consensusError_y = maxAbsDiff(y, NUM_TURTLES + 1);
            consensusError   = fmax(consensusError_x, consensusError_y);
            // consensusError = sqrt(pow(x[0] - x[j], 2) + pow(y[0] - y[j], 2));
        }

        // printf("\n");
        // for (int i = 1; i <= NUM_TURTLES; i++) {
        //  printf("x[%i] = %f\n", i, x[i]);
        // }
        // for (int i = 1; i <= NUM_TURTLES; i++) {
        //  printf("y[%i] = %f\n", i, y[i]);
        // }
        printf("\nConsensus Error:\t%f\n\n\n", consensusError);
    }
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    MOVETURTLES:        Set turtles velocities with a proportional controller,
                        then stop them
_____________________________________________________________________________*/

void moveTurtles(ros::Publisher turtleVel_pub[])
{
    double               linearError  = 0;
    double               angularError = 0;
    geometry_msgs::Twist cmdVel[NUM_TURTLES + 1];  // Turtles velocities

    for (int i = 1; i <= NUM_TURTLES; i++) {
        char str[30] = "cmdVel_";
        char index[4];
        sprintf(index, "%d", i);
        strcat(str, index);

        linearError  = getLinearError(currentPose[i], centroid[i]);
        angularError = getAngularError(currentPose[i], centroid[i]);
        if (fabs(linearError) > TOLERANCE) {
            if (linearError > 0)
                cmdVel[i].linear.x = KP_LINEAR * linearError;
            else
                cmdVel[i].linear.x = 0;
            cmdVel[i].angular.z = KP_ANGULAR * angularError;
        } else {
            // Stop i-th turtle
            cmdVel[i].linear.x  = 0;
            cmdVel[i].angular.z = 0;
            check[i]            = true;
        }
    }

    // Publish the controls
    for (int i = 1; i <= NUM_TURTLES; i++)
        turtleVel_pub[i].publish(cmdVel[i]);
}


/*‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
    MAIN:       Da main
_____________________________________________________________________________*/

int main(int argc, char **argv)
{
    // Node creation
    ros::init(argc, argv, "consensus");
    ros::NodeHandle nh;

    // Subscribers for reading turtles current poses
    ros::Subscriber currentPose_sub[NUM_TURTLES + 1];
    for (int i = 1; i <= NUM_TURTLES; i++) {
        char str[30] = "/turtle";
        char index[4];
        sprintf(index, "%d", i);
        strcat(str, index);
        strcat(str, "/pose");

        currentPose_sub[i] = nh.subscribe<turtlesim::Pose>(
            str, 100, boost::bind(getCurrentPose_Callback, _1, i));
    }

    // Publishers for reading turtles velocities
    ros::Publisher turtleVel_pub[NUM_TURTLES + 1];
    for (int i = 1; i <= NUM_TURTLES; i++) {
        char str[30] = "/turtle";
        char index[4];
        sprintf(index, "%d", i);
        strcat(str, index);
        strcat(str, "/cmd_vel");

        turtleVel_pub[i] = nh.advertise<geometry_msgs::Twist>(str, 100);
    }

    init(nh);

    achieveConsensus();

    computeCentroids();
    computeCentroidTheoretical();

    usleep(2 * SECOND);
    printf("\nMoving all %i turtles ...\n", NUM_TURTLES);
    usleep(1 * SECOND);

    initializeArrayToFalse(check, NUM_TURTLES + 1);
    check[0] = true;
    int c    = 0;  // key pressed (ASCII code)

    // Node frequency
    ros::Rate loop_rate(10);  // Frequency to run loops to (10 Hz)

    // Execute until node and channel are ok, and esc key is not pressed
    while (ros::ok() && nh.ok() && c != KEY_ESC) {
        ros::spinOnce();
        
        if (stop == false) {
            moveTurtles(turtleVel_pub);
        } else {
            manageConsensusAchieved();
            ros::shutdown();
        }

        // Once linearError <= TOLERANCE for each turtle (turtle = 1, ... ,
        // NUM_TURTLES), stop the entire process
        if (checkArray(check, NUM_TURTLES + 1))
            stop = true;

        c = getch();  // read character (non-blocking)

        loop_rate.sleep();
    }

    return 0;
}
