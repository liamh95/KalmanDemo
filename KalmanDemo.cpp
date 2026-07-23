#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <random>
#include <fstream>
#include <iomanip>
#include "KalmanFilter.h"


constexpr double PROCESS_NOISE_STD_DEV = 0.3; // m / s^2
//constexpr double DISTANCE_TOLERANCE {2.5}; // m
constexpr double GATE_CHI_SQUARE_THRESHOLD {9.210}; // 99% confidence, 2 degrees of freedom (our  measurements are 2-dimensional)
constexpr double GPS_LOCKOUT {2.0}; // s


const Eigen::Matrix2d IMU_COV = Eigen::Matrix2d::Identity() * 0.25;
const Eigen::Matrix2d GPS_COV = Eigen::Matrix2d::Identity() * 1.21;
const Eigen::Matrix2d LID_COV = Eigen::Matrix2d::Identity() * 0.3;


// Measurement matrix H_k (sometimes C_k)
const Eigen::Matrix<double, 2, 4> IMU_MATRIX {
    {0.0, 0.0, 1.0, 0.0},
    {0.0, 0.0, 0.0, 1.0}
};

const Eigen::Matrix<double, 2, 4> GPS_MATRIX {
    {1.0, 0.0, 0.0, 0.0},
    {0.0, 1.0, 0.0, 0.0}
};

const Eigen::Matrix<double, 2, 4> LID_MATRIX {
    {1.0, 0.0, 0.0, 0.0},
    {0.0, 1.0, 0.0, 0.0}
};

Eigen::VectorXd sampleStdNormal(Eigen::Index n, std::mt19937& gen){
    Eigen::VectorXd ret(n);
    std::normal_distribution<double> dist(0.0, 1.0);
    for(Eigen::Index i = 0; i < n; i++){
        ret(i) = dist(gen);
    }
    return ret;
}

// would this be a class in an ideal world since L * L^T = R should be invariant?
struct Sensor{
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;
    Eigen::MatrixXd L; // L * L^T = R, Cholesky.
    Eigen::VectorXd measure(const Eigen::Vector4d& x, std::mt19937& gen) const{
        return H * x + L * sampleStdNormal(H.rows(), gen);
    }
};


struct DataFrame{
    double time; // seconds
    Eigen::Vector2d imu_vel; // m/s
    Eigen::Vector2d gps_pos; // m
    Eigen::Vector2d lidar_pos; // m

    bool gps_available = true;
    bool lidar_available = true;
};


// truth doesn't have time stamps. probably nbd since we're always using it along with frames
struct SimData{
    std::vector<DataFrame> frames;
    std::vector<Eigen::Vector4d> truth;
};


// Drive straight ahead at 55mph = 24.59 m/s (no noise)
constexpr double STRAIGHT_VELOCITY{24.59}; // m/s
Eigen::Vector4d straightLine(const double t){
    return {0.0, STRAIGHT_VELOCITY * t, 0, STRAIGHT_VELOCITY};
}

// 60ft = 18.29m turn radius at 25mph = 11.18m/s
constexpr double TURN_RADIUS {18.29}; // m
constexpr double ANGULAR_VELOCITY {11.18 / TURN_RADIUS};
Eigen::Vector4d circle(const double t){
    double a = ANGULAR_VELOCITY * t;
    return {TURN_RADIUS * std::cos(a), TURN_RADIUS * std::sin(a), -1 * TURN_RADIUS * ANGULAR_VELOCITY * std::sin(a), TURN_RADIUS * ANGULAR_VELOCITY * std::cos(a)};
}


// one minute of simulation
constexpr double SIM_DT {0.1}; // 10Hz seems like a reasonable sensor polling rate
constexpr double SIM_MAX_T {60.0};
// Model dropped sensor availability and bad values with exponential distribution
// Therefore, number of failures in ia minute is Poisson
constexpr double MEAN_GPS_UNAVAIL_PER_MIN       {6.1};
constexpr double MEAN_GPS_GARBAGE_PER_MIN       {5.8};
constexpr double MEAN_LIDAR_UNAVAIL_PER_MIN     {4.0};
constexpr double MEAN_LIDAR_GARBAGE_PER_MIN     {3.4};

constexpr double LAMBDA_GPS_UNAVAIL       {MEAN_GPS_UNAVAIL_PER_MIN / 60.0};
constexpr double LAMBDA_GPS_GARBAGE       {MEAN_GPS_GARBAGE_PER_MIN / 60.0};
constexpr double LAMBDA_LIDAR_UNAVAIL     {MEAN_LIDAR_UNAVAIL_PER_MIN / 60.0};
constexpr double LAMBDA_LIDAR_GARBAGE     {MEAN_LIDAR_GARBAGE_PER_MIN / 60.0};

// TODO: add an optional parameter for process noise
// Garbage sensor data takes state and adds a big Gaussian to it
SimData simulate(std::function<Eigen::Vector4d(double)> truth_at, const unsigned int random_seed){

    std::vector<DataFrame> frames;
    std::vector<Eigen::Vector4d> truth;

    // Could have more or fewer frames if we allow random time steps
    frames.reserve(600);
    truth.reserve(600);

    std::mt19937 gen(random_seed);
    //TODO: make this more modular
    std::exponential_distribution<double> dist_gps_unavail(LAMBDA_GPS_UNAVAIL);
    std::exponential_distribution<double> dist_gps_garbage(LAMBDA_GPS_GARBAGE);
    std::exponential_distribution<double> dist_lidar_unavail(LAMBDA_LIDAR_UNAVAIL);
    std::exponential_distribution<double> dist_lidar_garbage(LAMBDA_LIDAR_GARBAGE);


    // Sensors
    Sensor imu       {IMU_MATRIX, IMU_COV, Eigen::LLT<Eigen::MatrixXd>(IMU_COV).matrixL()};
    Sensor gps       {GPS_MATRIX, GPS_COV, Eigen::LLT<Eigen::MatrixXd>(GPS_COV).matrixL()};
    Sensor lidar     {LID_MATRIX, LID_COV, Eigen::LLT<Eigen::MatrixXd>(LID_COV).matrixL()};

    double t {0.0};
    double t_next_gps_unavail =       dist_gps_unavail(gen);
    double t_next_gps_garbage =       dist_gps_garbage(gen);
    double t_next_lidar_unavail =     dist_lidar_unavail(gen);
    double t_next_lidar_garbage =     dist_lidar_garbage(gen);

    int step {0};

    while (t < SIM_MAX_T){
        // Truth
        Eigen::Vector4d true_state = truth_at(t);
        // Would add process noise here
        truth.push_back(true_state);

        // Measurement
        DataFrame frame;
        frame.time = t;
        Eigen::Vector2d imu_vel =       imu.measure(true_state, gen);
        Eigen::Vector2d gps_pos =       gps.measure(true_state, gen);
        Eigen::Vector2d lidar_pos =     lidar.measure(true_state, gen);

        if(t > t_next_gps_unavail){
            frame.gps_available = false;
            t_next_gps_unavail += dist_gps_unavail(gen);
        }

        if(t > t_next_gps_garbage){
            t_next_gps_garbage += dist_gps_garbage(gen);
            gps_pos += 3 * sampleStdNormal(2, gen);
        }

        if(t > t_next_lidar_unavail){
            frame.lidar_available = false;
            t_next_lidar_unavail += dist_lidar_unavail(gen);
        }

        if(t > t_next_lidar_garbage){
            t_next_lidar_garbage += dist_lidar_garbage(gen);
            lidar_pos += 3 * sampleStdNormal(2, gen);
        }

        frame.imu_vel = imu_vel;
        frame.gps_pos = gps_pos;
        frame.lidar_pos = lidar_pos;

        frames.push_back(frame);

        step++;
        // avoids float error accumulation compared to t += SIM_DT
        t = step * SIM_DT; // TODO: add some randomness (Gaussian) to this too for funsies?
    }

    return SimData {frames, truth};
}

void writeLine(std::ofstream& file, const DataFrame& frame, const Eigen::Vector4d& true_trajectory, const Eigen::Vector4d& estimate, const bool gps_used, const bool lidar_used){
    file << frame.time << ","
         << true_trajectory(0) << ","
         << true_trajectory(1) << ","
         << true_trajectory(2) << ","
         << true_trajectory(3) << ","
         << estimate(0) << ","
         << estimate(1) << ","
         << estimate(2) << ","
         << estimate(3) << ","
         << frame.imu_vel(0) << ","
         << frame.imu_vel(1) << ","
         << frame.gps_available << ","
         << gps_used << ","
         << frame.gps_pos(0) << ","
         << frame.gps_pos(1) << ","
         << frame.lidar_available << ","
         << lidar_used << ","
         << frame.lidar_pos(0) << ","
         << frame.lidar_pos(1) << "\n";
}

int main(){

    double ignore_gps_until = -1.0;

    bool gps_available = true;
    bool lidar_available = true;
    bool gps_valid = true;
    bool lidar_valid = true;

    bool use_gps = true;
    bool use_lidar = true;

    SimData sim_data = simulate(straightLine, 0);



    // Assume first frame contains initial position and velocity
    std::vector<DataFrame> data = sim_data.frames;

    if(data.size() == 0){
        std::cout << "No data!" << std::endl;

        return -1;
    }

    std::ofstream csv_file("straight_line_no_proc_noise.csv");
    csv_file << std::setprecision(9);
    csv_file << "t,"
             << "x_true,"
             << "y_true,"
             << "vx_true,"
             << "vy_true,"
             << "x_est,"
             << "y_est,"
             << "vx_est,"
             << "vy_est,"
             << "imu_vx,"
             << "imu_vy,"
             << "gps_avail,"
             << "gps_used,"
             << "gps_x,"
             << "gps_y,"
             << "lidar_avail,"
             << "lidar_used,"
             << "lidar_x,"
             << "lidar_y\n";

    Eigen::Vector4d x0 = Eigen::Vector4d::Zero();
    Eigen::Matrix4d P0 = Eigen::Matrix4d::Identity() * 1e6;

    KalmanFilter filter(x0, P0, PROCESS_NOISE_STD_DEV);

    // Process first frame without gating
    double prev_t = data[0].time;
    filter.update(data[0].imu_vel, IMU_MATRIX, IMU_COV);
    if (data[0].gps_available) filter.update(data[0].gps_pos, GPS_MATRIX, GPS_COV);
    if (data[0].lidar_available) filter.update(data[0].lidar_pos, LID_MATRIX, LID_COV);
    Eigen::Vector2d position_estimate = filter.position();
    writeLine(csv_file, data[0], sim_data.truth[0], filter.state(), data[0].gps_available, data[0].lidar_available);



    // drop first frame since we just processed it
    for(std::size_t i = 1; i < data.size(); i++){
        const double dt = data[i].time - prev_t;
        prev_t = data[i].time;

        filter.predict(dt);

        // Dropout logic
        // We include a sensor in our fusion if and only if it is available and valid
        // The sensor is available if the sensor tells us it is (we read this from the frame)
        // The sensor is valid if its distance from our dead reckoning position is within some tolerance (this part im fuzzy on)

        // Extract position and velocity from state vector
        position_estimate = filter.position();

        gps_available = data[i].gps_available;
        lidar_available = data[i].lidar_available;

        // Compute this regardless since it determines lockout
        gps_valid = filter.gateAccepts(data[i].gps_pos, GPS_MATRIX, GPS_COV, GATE_CHI_SQUARE_THRESHOLD);
        if (gps_available && !gps_valid){
            ignore_gps_until = data[i].time + GPS_LOCKOUT;
        }
        // only compute validity if available. faster this way
        if (lidar_available)
            lidar_valid = filter.gateAccepts(data[i].lidar_pos, LID_MATRIX, LID_COV, GATE_CHI_SQUARE_THRESHOLD);
        
        use_gps = gps_valid && gps_available && (data[i].time > ignore_gps_until);
        use_lidar = lidar_valid && lidar_available;


        // Update with IMU
        filter.update(data[i].imu_vel, IMU_MATRIX, IMU_COV);

        // Update with GPS
        if (use_gps)
            filter.update(data[i].gps_pos, GPS_MATRIX, GPS_COV);

        // Update with LIDAR
        if (use_lidar)
            filter.update(data[i].lidar_pos, LID_MATRIX, LID_COV);

        writeLine(csv_file, data[i], sim_data.truth[i], filter.state(), use_gps, use_lidar);
    }

    csv_file.close();

    return 0;
}
