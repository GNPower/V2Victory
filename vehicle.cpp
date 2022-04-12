#include <math.h>
#include <cmath>
#include <iostream>
#include <random>
#include "vehicle.hpp"

#ifdef EMS_OVERRIDE_ENABLE
//need to account for k_veh_type
#ifdef PLATOONING_ENABLE
//Platooning + EMS
vehicle::vehicle(unsigned int id, double max_speed, y_veh_type veh_type, double front_pos, double set_speed, double pos_x, double pos_y, double speed, double heading, bool override, bool platooning, double driver_accel_req)
#else
//EMS
vehicle::vehicle(unsigned int id, double max_speed, y_veh_type veh_type, double front_pos, double set_speed, double pos_x, double pos_y, double speed, double heading, bool override, double driver_accel_req)
#endif
#else
#ifdef PLATOONING_ENABLE
//Platooning
vehicle::vehicle(unsigned int id, double max_speed, double front_pos, double set_speed, double pos_x, double pos_y, double speed, double heading, bool platooning, double driver_accel_req)
#else
//None
vehicle::vehicle(unsigned int id, double max_speed, double front_pos, double set_speed, double pos_x, double pos_y, double speed, double heading, double driver_accel_req)
#endif
#endif
{
    //Setup Inputs
    k_id = id;
    k_max_speed = max_speed;
    k_front_pos = front_pos;

    #ifdef EMS_OVERRIDE_ENABLE
    k_veh_type = veh_type;
    #endif
    
    p_set_speed = set_speed;
    validate_set_speed();
    
    #ifdef EMS_OVERRIDE_ENABLE
    if (k_veh_type == EMERGENCY)
    {
        p_override = override;
    }
    else
    {
        p_override = false;
    }
    #endif

    #ifdef PLATOONING_ENABLE
    p_platooning = platooning;
    #endif

    //NOTE: all position updates after initialization are from the GPS & IMU or sim updates methods, this is just for something to start
    m_ego_pos_x = pos_x;
    m_ego_pos_y = pos_y;
    m_ego_speed = speed;
    m_ego_heading = heading;
    ego_updated = true;

    //setup kalman filter
    #ifdef KALMAN_ENABLE
    X_pred = Eigen::VectorXd(4);
    P_pred = Eigen::MatrixXd(4, 4);
    C = Eigen::MatrixXd(3, 4);
    R = Eigen::MatrixXd(3, 3);
    X_pred << 0, 0, 0, 0;
    P_pred.setIdentity();
    P_pred = P_pred*P_PRED_INIT;
    C << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0;
    R << pow(ERROR_POS, 2), 0, 0, 0, pow(ERROR_POS, 2), 0, 0, 0, pow(ERROR_SPEED, 2); 
    //do first iteration for P_pred
    Eigen::MatrixXd A;
    A = Eigen::MatrixXd(4, 4);
    double ch = cos(m_ego_heading*M_PI/180);
    double sh = sin(m_ego_heading*M_PI/180);
    A.row(0) << 1, 0, TIMESTEP*ch, pow(TIMESTEP, 2)*ch/2;
    A.row(1) << 0, 1, TIMESTEP*sh, pow(TIMESTEP, 2)*sh/2;
    A.row(2) << 0, 0, 1, TIMESTEP;
    A.row(3) << 0, 0, 0, 1;

    Eigen::MatrixXd Q;
    Q = Eigen::MatrixXd(4, 4);
    Q.row(0) << pow(TIMESTEP, 4)*ch*ch/4, 0, pow(TIMESTEP, 3)*ch/2, pow(TIMESTEP, 2)*ch/2;
    Q.row(1) << 0, pow(TIMESTEP, 4)*sh*sh/4, pow(TIMESTEP, 3)*sh/2, pow(TIMESTEP, 2)*sh/2;
    Q.row(2) << pow(TIMESTEP, 3)*ch/2, pow(TIMESTEP, 3)*sh/2, pow(TIMESTEP, 2), TIMESTEP;
    Q.row(3) << pow(TIMESTEP, 2)*ch/2, pow(TIMESTEP, 2)*sh/2, TIMESTEP, 1;
    
    P_pred = A*P_pred*A.transpose() + Q;
    #endif

    m_driver_accel_req = driver_accel_req;

    //initialize vehicle arrays
    init_veh_data();

    //initialize intersection arrays
    init_int_data();

    //initialize controlled variables
    init_outputs();

    //initialize lead vehicle info
    init_lead_veh();

    //initialize relevant intersection info
    init_rel_int();
}

double vehicle::get_accel_req(void)
{
    return c_accel_req;
}

bool vehicle::get_driver_alerts(void)
{
    return c_driver_alert;
}

#ifdef SIM
s_published_vehicle_data vehicle::publish(void)
#else
void vehicle::publish(void)
#endif
{
    #ifdef SIM
    //just output the data
    s_published_vehicle_data veh_data;
    veh_data.veh_id = k_id;
    veh_data.veh_pos_x = coasted_ego_pos_x;
    veh_data.veh_pos_y = coasted_ego_pos_y;
    veh_data.veh_speed = coasted_ego_speed;
    veh_data.veh_heading = m_ego_heading;

    #ifdef EMS_OVERRIDE_ENABLE
    veh_data.veh_type = k_veh_type;
    veh_data.override = p_override;
    #endif

    #ifdef PLATOONING_ENABLE
    veh_data.platooning = p_platooning;
    veh_data.platoon_set_speed = c_platoon_set_speed;
    #endif

    //setup crc
    veh_data.veh_crc = get_veh_crc(veh_data);

    return veh_data;

    #else
    //ROS setup here
    #endif
}

#ifdef PLATOONING_ENABLE
void vehicle::platooning_callback(bool platooning)
{
    p_platooning = platooning;
}
#endif

#ifdef EMS_OVERRIDE_ENABLE
void vehicle::override_callback(bool override)
{
    if (override && k_veh_type == EMERGENCY)
    {
        p_override = true;
    }
    else
    {
        p_override = false;
    }
}
#endif

void vehicle::ego_data_callback(double pos_x, double pos_y, double speed, double heading)
{
    m_ego_pos_x = pos_x;
    m_ego_pos_y = pos_y;
    m_ego_speed = speed;
    m_ego_heading = heading;
    ego_updated = true;
}

void vehicle::driver_req_callback(double driver_accel_req)
{
    m_driver_accel_req = driver_accel_req;
}

void vehicle::set_speed_callback(double set_speed)
{
    p_set_speed = set_speed;
    validate_set_speed();
}

#ifdef SIM
void vehicle::new_vehicle_callback(s_published_vehicle_data data)
#else
void vehicle::new_vehicle_callback(const std_msgs::String::ConstPtr& msg)
#endif
{
    #ifndef SIM
    //actually parse data, put into s_vehicle_data struct, then do below
    #endif

    if (num_new_vehicles < MAX_VEHICLES)
    {
        //check crc
        unsigned int calc_crc = get_veh_crc(data);
        if (calc_crc == data.veh_crc)
        {
            new_vehicles[num_new_vehicles].veh_id = data.veh_id;
            new_vehicles[num_new_vehicles].veh_pos_x = data.veh_pos_x;
            new_vehicles[num_new_vehicles].veh_pos_y = data.veh_pos_y;
            new_vehicles[num_new_vehicles].veh_speed = data.veh_speed;
            new_vehicles[num_new_vehicles].veh_heading = data.veh_heading;
            new_vehicles[num_new_vehicles].time_since_update = 0;

            #ifdef PLATOONING_ENABLE
            new_vehicles[num_new_vehicles].platooning = data.platooning;
            new_vehicles[num_new_vehicles].platoon_set_speed = data.platoon_set_speed;
            #endif

            num_new_vehicles++;
        }
        #ifdef DEBUG
        else
        {
            std::cout << "CRC FAIL! Calculated: " << calc_crc << "\tReceived: " << data.veh_crc << "\n";
        }
        #endif
    }
}

#ifdef SIM
void vehicle::new_int_callback(s_published_int_data data)
#else
void vehicle::new_int_callback(const std_msgs::String::ConstPtr& msg)
#endif
{
    #ifndef SIM
    //actually parse data, put into s_vehicle_data struct, then do below
    #endif

    if (num_new_ints < MAX_INTS)
    {
        //check crc
        unsigned int calc_crc = get_int_crc(data);
        if (calc_crc == data.int_crc)
        {
            new_ints[num_new_ints].int_id = data.int_id;
            new_ints[num_new_ints].int_length = data.int_length;
            
            new_ints[num_new_ints].int_1_state = data.int_1_state;
            new_ints[num_new_ints].int_1_next_state = data.int_1_next_state;
            new_ints[num_new_ints].int_1_time_to_switch = data.int_1_time_to_switch;
            new_ints[num_new_ints].int_2_state = data.int_2_state;
            new_ints[num_new_ints].int_2_next_state = data.int_2_next_state;
            new_ints[num_new_ints].int_2_time_to_switch = data.int_2_time_to_switch;
            new_ints[num_new_ints].time_since_update = 0;

            #ifdef STOP_SIGN_ENABLE
            new_ints[num_new_ints].int_type = data.int_type;
            new_ints[num_new_ints].int_next_veh_id = data.int_next_veh_id;
            #endif
            
            //copied from intersection, same logic
            //entrance 0 is forward from center by dir 1 (heading is opposite of dir 1)
            //entrance 1 is backward from center by dir 1 (heading is same as dir 1)
            //entrance 2 is forward from center by dir 2 (heading is opposite of dir 2)
            //entrance 3 is backward from center by dir 2 (heading is same as dir 2)
            //all the entrance headings are backwards of these since those are angle of entrance through intersection
            double x, y, angle;
            for (int i = 0; i < 4; i++)
            {
                if (i < 2) //Entrance Direction #1
                {   
                    angle = data.ent_dir_1*M_PI/180;
                }
                else //Entrance Direction #2
                {
                    angle = data.ent_dir_2*M_PI/180;
                }
                double angle_2 = angle + M_PI/180*90;
                double x_delta = data.int_length/2*cos(angle) + data.int_length/4*cos(angle_2); //x is forward, angle is CCW from North
                double y_delta = data.int_length/2*sin(angle) + data.int_length/4*sin(angle_2); //y is side, angle is CCW from North

                if (i == 0 || i == 2)
                {
                    //0 deg
                    x = data.int_pos_x + x_delta;
                    y = data.int_pos_y + y_delta;
                }
                else
                {
                    //180 deg
                    x = data.int_pos_x - x_delta;
                    y = data.int_pos_y - y_delta;
                }

                new_ints[num_new_ints].int_ent_pos_x[i] = x;
                new_ints[num_new_ints].int_ent_pos_y[i] = y;
            }

            //setup entrance headings
            if (data.ent_dir_1 >= 180)
            {
                new_ints[num_new_ints].int_ent_headings[0] = data.ent_dir_1 - 180;
            }
            else 
            {
                new_ints[num_new_ints].int_ent_headings[0] = data.ent_dir_1 + 180;
            }

            new_ints[num_new_ints].int_ent_headings[1] = data.ent_dir_1;

            if (data.ent_dir_2 >= 180)
            {
                new_ints[num_new_ints].int_ent_headings[2] = data.ent_dir_2 - 180;
            }
            else 
            {
                new_ints[num_new_ints].int_ent_headings[2] = data.ent_dir_2 + 180;
            }

            new_ints[num_new_ints].int_ent_headings[3] = data.ent_dir_2;
            num_new_ints++;
        }
        #ifdef DEBUG
        else
        {
            std::cout << "CRC FAIL! Calculated: " << calc_crc << "\tReceived: " << data.int_crc << "\n";
        }
        #endif
    }
}

void vehicle::update(double dt)
{
    //coast/update ego data
    update_ego_data(dt);

    //first update vehicle and intersection data with new ones
    update_veh_data(dt);
    update_int_data(dt);

    //next find lead vehicle
    update_lead_veh();

    //find relevant intersection
    #ifdef DRIVER
    update_rel_int(dt);
    #else
    update_rel_int();
    #endif

    //now have all the data needed, can update accel req
    update_accel_req();

    #ifdef PLATOONING_ENABLE
    //lastly update platooning info (if enabled)
    update_platooning();
    #endif
}

#ifdef PLATOONING_ENABLE
void vehicle::update_platooning(void)
{
    if (lead_platooning)
    {
        c_platoon_set_speed = lead_platoon_set_speed;
    }
    else
    {
        c_platoon_set_speed = p_set_speed; //we are head of platoong
    }
}
#endif

void vehicle::update_ego_data(double dt)
{
    //operate under assumption callback for ego data is run before
    if (ego_updated)
    {
        ego_updated = false; //reset so if updated again without callback will coast
        
        #ifdef KALMAN_ENABLE
        //measure
        Eigen::MatrixXd X_meas;
        X_meas = Eigen::VectorXd(3);
        X_meas << m_ego_pos_x, m_ego_pos_y, m_ego_speed;

        //update K
        Eigen::MatrixXd K, K_p1;
        K_p1 = C*P_pred*C.transpose() + R;
        K = P_pred*C.transpose()*K_p1.inverse();

        //estimate
        Eigen::MatrixXd X_est;
        X_est = X_pred + K*(X_meas - C*X_pred);
        X_est(2) = fmax(X_est(2), 0);

        //update P
        Eigen::MatrixXd I, P_p1;
        I = Eigen::MatrixXd(4, 4);
        I.setIdentity();
        P_p1 = I - K*C;
        P = P_p1*P_pred*P_p1.transpose() + K*R*K.transpose();

        //predict
        Eigen::MatrixXd A;
        A = Eigen::MatrixXd(4, 4);
        double ch = cos(m_ego_heading*M_PI/180);
        double sh = sin(m_ego_heading*M_PI/180);
        A.row(0) << 1, 0, TIMESTEP*ch, pow(TIMESTEP, 2)*ch/2;
        A.row(1) << 0, 1, TIMESTEP*sh, pow(TIMESTEP, 2)*sh/2;
        A.row(2) << 0, 0, 1, TIMESTEP;
        A.row(3) << 0, 0, 0, 1;
        X_pred = A*X_est;
        X_pred(2) = fmax(X_pred(2), 0);

        Eigen::MatrixXd Q;
        Q = Eigen::MatrixXd(4, 4);
        Q.row(0) << pow(TIMESTEP, 4)*ch*ch/4, 0, pow(TIMESTEP, 3)*ch/2, pow(TIMESTEP, 2)*ch/2;
        Q.row(1) << 0, pow(TIMESTEP, 4)*sh*sh/4, pow(TIMESTEP, 3)*sh/2, pow(TIMESTEP, 2)*sh/2;
        Q.row(2) << pow(TIMESTEP, 3)*ch/2, pow(TIMESTEP, 3)*sh/2, pow(TIMESTEP, 2), TIMESTEP;
        Q.row(3) << pow(TIMESTEP, 2)*ch/2, pow(TIMESTEP, 2)*sh/2, TIMESTEP, 1;

        P_pred = A*P*A.transpose() + Q;

        coasted_ego_pos_x = X_est(0);
        coasted_ego_pos_y = X_est(1);
        coasted_ego_speed = X_est(2);
        
        #ifdef DEBUG
        std::cout << "update_ego_data: x = " << X_est(0) << "\ty = " << X_est(1) << "\tv = " << X_est(2) << "\n";
        #endif
        #else
        coasted_ego_pos_x = m_ego_pos_x;
        coasted_ego_pos_y = m_ego_pos_y;
        coasted_ego_speed = m_ego_speed;

        #endif
    }
    else
    {
        //cannot assume constant vel, since accelerating
        //work backwards, first update speed
        //dont directly update coasted_ego_speed since can use to get new speed
        double new_speed = coasted_ego_speed + dt*c_accel_req;
        coasted_ego_pos_x = coasted_ego_pos_x + dt*(coasted_ego_speed + new_speed)/2*cos(M_PI/180*m_ego_heading);
        coasted_ego_pos_y = coasted_ego_pos_y + dt*(coasted_ego_speed + new_speed)/2*sin(M_PI/180*m_ego_heading);
        coasted_ego_speed = new_speed;
    }
}

void vehicle::update_accel_req(void)
{
    #ifdef DRIVER
    //modelling human driver
    update_sys_accel_req();
    c_accel_req = system_accel_req;
    #else
    //first calculate system's accel request
    update_sys_accel_req();

    //check if any driver overrides apply
    if (m_driver_accel_req == 0)
    {
        //no driver input, take system's
        c_accel_req = system_accel_req;
        #ifdef DEBUG
        std::cout << "update_accel_req: No Driver Input\n";
        #endif
    }
    else if (m_driver_accel_req > 0)
    {
        //driver wants to accelerate
        c_accel_req = fmax(m_driver_accel_req, system_accel_req);
        #ifdef DEBUG
        std::cout << "update_accel_req: Driver Accel\n";
        #endif
    }
    else
    {
        //driver wants to decelerate
        c_accel_req = fmin(m_driver_accel_req, system_accel_req);
        #ifdef DEBUG
        std::cout << "update_accel_req: Driver Decel\n";
        #endif
    }
    #endif
}

#ifdef DRIVER
double vehicle::get_driver_int_accel_req(void)
{
    double int_accel_req;
    //use last state to account for driver delay
    //get a perceived distance
    std::default_random_engine generator;
    std::normal_distribution<double> dist_error(8.6, 13.4);

    double int_ent_pct = 1 - dist_error(generator)/100; //according to study, average distance is -8.6% of actual
    int_ent_pct = fmax(int_ent_pct, 0.1);
    double stop_dist_pct = 1 - dist_error(generator)/100;
    stop_dist_pct = fmax(stop_dist_pct, 0.1);


    if (last_state == RED)
    {
        //Red Light, stop at int entrance
        double braking_dist = pow(coasted_ego_speed, 2) / (2*MAX_DECEL) + STOP_SIGN_DISTANCE; //stop sign dist added as safety factor
        double stop_dist = int_ent_dist - k_front_pos; //stop dist/2 to allow error
        if (stop_dist*int_ent_pct > braking_dist*stop_dist_pct)
        {
            //far away, check what accel can be requested without breaking stop dist
            int_accel_req = 2*((stop_dist*int_ent_pct - braking_dist*stop_dist_pct) - coasted_ego_speed*TIMESTEP)/pow(TIMESTEP, 2);
        }
        else
        {
            //too close, need to stop at thing
            int_accel_req = -pow(coasted_ego_speed, 2)/(2*stop_dist*int_ent_pct);
        }

        //int_accel_req = -pow(coasted_ego_speed, 2)/(2*int_ent_dist*int_ent_pct);
    }
    else if (last_state == YELLOW)
    {
        //Yellow Light, see if we should continue or stop
        //only continue if we cannot stop in time
        //if continue, go for set speed
        //if stop, go for int entrance
        double stopping_dist = -pow(coasted_ego_speed, 2)/(2*MAX_DECEL)*stop_dist_pct;
        if (stopping_dist > int_ent_dist*int_ent_pct)
        {
            //cannot stop in time, continue
            int_accel_req = p_set_speed - coasted_ego_speed;
        }
        else
        {
            //can stop in time, stop at entrance
            int_accel_req = -pow(coasted_ego_speed, 2)/(2*int_ent_dist*int_ent_pct);
        }
    }
    else {
        std::cout << "LS GREEN\n";
        //Green Light, go for set speed
        int_accel_req = p_set_speed - coasted_ego_speed;
    }
    return int_accel_req;
}
#endif

void vehicle::update_sys_accel_req(void)
{
    //takes in the ego vehicle data, lead vehicle, relevant intersection, set speed, platooning mode (if enabled), to determine accel 

    if (lead_exists && approaching_int)
    {
        #ifdef DEBUG
        std::cout << "update_sys_accel_req: Lead + Intersection\n";
        #endif

        #ifdef DRIVER
        double int_accel_req = get_driver_int_accel_req();
        #else
        double int_accel_req = get_int_accel_req();
        #endif
        
        double lead_accel_req = get_lead_accel_req();

        #ifdef DEBUG
        std::cout << "update_sys_accel_req: int_accel_req = " << int_accel_req << "\tlead_accel_req = " << lead_accel_req << "\n";
        #endif

        system_accel_req = fmin(int_accel_req, lead_accel_req);
    }
    else if (lead_exists)
    {
        #ifdef DEBUG
        std::cout << "update_sys_accel_req: Lead\n";
        #endif

        //lead vehicle, no intersection
        system_accel_req = get_lead_accel_req();
    }
    else if (approaching_int)
    {
        #ifdef DEBUG
        std::cout << "update_sys_accel_req: Intersection\n";
        #endif

        //intersection, no lead vehicle
        #ifdef DRIVER
        system_accel_req = get_driver_int_accel_req();
        #else
        system_accel_req = get_int_accel_req();
        #endif
    }
    else
    {
        #ifdef DEBUG
        std::cout << "update_sys_accel_req: None\n";
        #endif

        //no lead vehicle or intersection, go to set speed
        system_accel_req = p_set_speed - coasted_ego_speed;
    }
    
    //saturate requests between valid ranges
    if (system_accel_req > MAX_ACCEL)
    {
        system_accel_req = MAX_ACCEL;
    }
    else if (system_accel_req < -MAX_DECEL)
    {
        system_accel_req = -MAX_DECEL;
    }
}

double vehicle::get_lead_accel_req(void)
{
    double lead_accel_req;
    //get desired gap from lead vehicle
    double desired_lead_gap;
    static double iError = 0;

    #ifdef PLATOONING_ENABLE
    if (p_platooning && lead_platooning)
    {
        desired_lead_gap = k_front_pos + PLATOONING_GAP;
    }
    else
    {
        desired_lead_gap = k_front_pos + fmax(LEAD_GAP_DIST, LEAD_GAP_TIME*m_ego_speed);
    }
    #else
    desired_lead_gap = k_front_pos + fmax(LEAD_GAP_DIST, LEAD_GAP_TIME*m_ego_speed);
    #endif

    #ifdef DEBUG
    std::cout << "get_lead_accel_req: desired_lead_gap = " << desired_lead_gap << "\n";
    #endif
    
    double dist_error = abs(lead_rel_pos_x) - desired_lead_gap;
    #ifdef DEBUG
    std::cout << "get_lead_accel_req: dist_error = " << dist_error << "\n";
    #endif
    //since using relative stuff need to use rel_vel_x
    double set_speed;
    //////////////////////////////////// PUT PLATOONING CATCH UP CODE HERE
    #ifdef PLATOONING_ENABLE
    if (p_platooning && lead_platooning)
    {
        if (lead_platoon_set_speed < p_set_speed)
        {
            c_driver_alert = true;
            std::cout << "platoon set speed below vehicle's set speed!\n using platoon's set speed...\n";
        }
        set_speed = lead_platoon_set_speed;
    }
    else
    {
        set_speed = p_set_speed;
    }
    #else
    set_speed = p_set_speed;
    #endif
    //////////////////////////////////// PUT PLATOONING CATCH UP CODE HERE

    double desired_speed;

    // iError = iError + dist_error*0.0001;
    iError = dist_error*0.1;
    std::cout << "iError = " << iError << "\n";

    if(abs(dist_error) <= 0.01)
    {
        // Do nothing
        desired_speed = coasted_ego_speed;

        #ifdef DEBUG
        std::cout << "get_lead_accel_req: dist_error flag within [-0.01,0.01]\n";
        #endif
    }
    else if(dist_error < -0.01)
    {
        /*
        If dist_error -ve, then we are too close
        Speed should be desired (set_speed) if we are at desired lead gap
        Speed should be 0 if we are on top of the car
        Simply scale linearly
        */
        desired_speed =  (coasted_ego_speed + lead_rel_vel_x)*(1 + dist_error/desired_lead_gap) + iError;
        //dist_error -ve so ends up being 1 - fraction, so lower than set speed

        #ifdef DEBUG
        std::cout << "get_lead_accel_req: dist_error flag less than -0.01, desired_speed = " << desired_speed << "\n";
        #endif
    }
    else
    {
        // If dist_error +ve, then we are too far
        // Speed should be speed of lead vehicle if we are at desired lead gap and set_speed if we are sufficiently far away enough
        // Sufficiently far enough is desired_lead_gap
        // Scale linearly
        // desired_speed = (coasted_ego_speed + lead_rel_vel_x) + (set_speed - (coasted_ego_speed + lead_rel_vel_x))/(2*desired_lead_gap);

        //
        #ifdef PLATOONING_ENABLE
        if (p_platooning && lead_platooning)
        {
            std::cout << "Using platooning catchup code\n";
            desired_speed = fmin(1.15*set_speed, set_speed + 0.15*set_speed*(dist_error/desired_lead_gap)) + iError;
            // desired_speed = set_speed;
        }
        else
        {
            desired_speed = set_speed;
            std::cout << "Not using platooning catchup code\n";
        }
        #else
            desired_speed = set_speed;
            std::cout << "Platooning not defined\n";
        #endif
        //

        #ifdef DEBUG
        std::cout << "get_lead_accel_req: dist_error flag greater than 0.01, desired_speed = " << desired_speed << "\n";
        #endif
    }

    // Convert desired_speed and current speed to acceleration
    // a = (v2-v1)/t
    // Arbitrarily choose t = 2.5s
    lead_accel_req = fmin((desired_speed - coasted_ego_speed)/2.5, MAX_ACCEL);

    #ifdef DEBUG
    std::cout << "get_lead_accel_req: desired_lead_gap = " << desired_lead_gap << "\tdist_error = " << dist_error << "\n";
    std::cout << "get_lead_accel_req: set_speed = " << set_speed << "\tcoasted_ego_speed = " << coasted_ego_speed << "\n";
    std::cout << "get_lead_accel_req: lead_accel_req = " << lead_accel_req << "\n";
    #endif

    return lead_accel_req;
}

double vehicle::get_int_accel_req(void)
{
    double int_accel_req;
    #ifdef STOP_SIGN_ENABLE
    if (int_type == SIGNAL && int_state == RED)
    #else
    if (int_state == RED)
    #endif
    {
        //traffic signal red, check if turning green soon
        if (int_next_state == GREEN)
        {
            #ifdef DEBUG
            std::cout << "get_int_accel_req: RED->GREEN\n";
            #endif
            //should be green, but need to account for ems override/dynamic timing
            //try to arrive at intersection with min accel
            //using: delta_s = vi*delta_t + a/2*delta_t^2
            //add safety factor to tts to ensure not entering on red light
            double safe_tts = int_time_to_switch + TTS_SAFETY_FACTOR;
            double make_light_req = (int_ent_dist - k_front_pos - coasted_ego_speed*safe_tts)*2/pow(safe_tts, 2);

            //need to ensure we dont go over set speed, so compare vf with set speed 
            double vf = coasted_ego_speed + make_light_req*int_time_to_switch;

            #ifdef DEBUG
            std::cout << "get_int_accel_req: safe_tts = " << safe_tts << "\tmake_light_req = " << make_light_req << "\tvf = " << vf << "\n";
            #endif
            if (vf > p_set_speed || make_light_req > MAX_ACCEL)
            {
                //need to reduce req
                //using: vf^2 = vi^2 + 2*a*delta_s where vf is set_speed
                int_accel_req = (pow(p_set_speed, 2) - pow(coasted_ego_speed, 2))/(2*(int_ent_dist - k_front_pos));
            }
            else
            {
                //request is below set speed at entrance, ok to use
                int_accel_req = make_light_req;
            }
        }
        else
        {
            #ifdef DEBUG
            std::cout << "get_int_accel_req: RED->RED/YELLOW\n";
            #endif
            //next state is red/yellow, just try to approach and stop at intersection
            //using: vf^2 = vi^2 + 2*a*delta_s where vf^2 is 0 and solving for a
            int_accel_req = -pow(coasted_ego_speed, 2)/(2*(int_ent_dist - k_front_pos));
        }
    }
    #ifdef STOP_SIGN_ENABLE
    else if (int_type == SIGNAL)
    #else
    else
    #endif
    {
        #ifdef DEBUG
        std::cout << "get_int_accel_req: GREEN/YELLOW\n";
        #endif
        //traffic signal green or yellow, check if can make through int or should slow down
        double time_to_red = int_time_to_switch;
        switch(int_next_state)
        {
            case GREEN:
                time_to_red += MIN_RED_GREEN_TIME;
                break;
            case YELLOW:
                time_to_red += MIN_YELLOW;
                break;
            //for red dont increment
        }

        #ifdef DEBUG
        std::cout << "get_int_accel_req: time_to_red = " << time_to_red << "\n";
        #endif

        //now have time til red, see if we can exit intersection
        double exit_light_req = (int_exit_dist - k_front_pos - coasted_ego_speed*time_to_red)*2/pow(time_to_red, 2);
        //now check if we go over set speed to reach it
        double vf = coasted_ego_speed + time_to_red*exit_light_req;
        if (vf > p_set_speed || exit_light_req > MAX_ACCEL)
        {
            //can't make it through intersection, need to stop before it
            int_accel_req = -pow(coasted_ego_speed, 2)/(2*(int_ent_dist - k_front_pos));
        }
        else
        {
            //this will barely make it through light, use set speed
            int_accel_req = p_set_speed - coasted_ego_speed;
        }
    }
    #ifdef STOP_SIGN_ENABLE
    else
    {
        //stop sign
        if (int_next_id == k_id)
        {
            //go (was at intersection)
            int_accel_req = p_set_speed - coasted_ego_speed;
        }
        else
        {
            //stop at intersection
            double braking_dist = pow(coasted_ego_speed, 2) / (2*MAX_DECEL) + STOP_SIGN_DISTANCE; //stop sign dist added as safety factor
            double stop_dist = int_ent_dist - k_front_pos; //stop dist/2 to allow error
            if (stop_dist > braking_dist)
            {
                //far away, check what accel can be requested without breaking stop dist
                int_accel_req = 2*((stop_dist - braking_dist) - coasted_ego_speed*TIMESTEP)/pow(TIMESTEP, 2);
            }
            else
            {
                //too close, need to stop at thing
                int_accel_req = -pow(coasted_ego_speed, 2)/(2*stop_dist);
            }
        }
    }
    #endif

    #ifdef DEBUG
    std::cout << "get_int_accel_req: int_accel_req = " << int_accel_req << "\n";
    #endif

    return int_accel_req;
}

void vehicle::update_veh_data(double dt)
{
    //first update existing tracks and delete ones that have coasted too long
    for (int i = 0; i < num_vehicles; i++)
    {
        bool match_found = false;
        unsigned int existing_id = m_vehicle_data[i].veh_id;
        for (int j = 0; j < num_new_vehicles; j++)
        {
            if (existing_id == new_vehicles[j].veh_id)
            {
                match_found = true;
                m_vehicle_data[i] = new_vehicles[j];
                break;
            }
        }
        if (!match_found)
        {
            //no update, increase time since last update
            m_vehicle_data[i].time_since_update += dt;

            //check if coasted too long
            if (m_vehicle_data[i].time_since_update > MAX_COASTING)
            {
                //delete, coasted too long
                for (int j = i; j < (num_vehicles - 1); j++)
                {
                    m_vehicle_data[j] = m_vehicle_data[j + 1];
                }
                //reset last vehicle
                m_vehicle_data[num_vehicles - 1].veh_id = 0;
                m_vehicle_data[num_vehicles - 1].veh_pos_x = 0;
                m_vehicle_data[num_vehicles - 1].veh_pos_y = 0;
                m_vehicle_data[num_vehicles - 1].veh_speed = 0;
                m_vehicle_data[num_vehicles - 1].veh_heading = 0;
                m_vehicle_data[num_vehicles - 1].time_since_update = 0;

                #ifdef PLATOONING_ENABLE
                m_vehicle_data[num_vehicles - 1].platooning = false;
                m_vehicle_data[num_vehicles - 1].platoon_set_speed = 0;
                #endif

                num_vehicles--;
                i--; //need to check new m_vehicle_data[i] since shifted over
            }
        }
    }

    //now add new vehicles
    for (int i = 0; i < num_new_vehicles; i++)
    {
        bool match_found = false;
        unsigned int new_id = new_vehicles[i].veh_id;
        for (int j = 0; j < num_vehicles; j++)
        {
            if (new_id == m_vehicle_data[j].veh_id)
            {
                match_found = true;
                break;
            }
        }
        if (!match_found && num_vehicles < MAX_VEHICLES)
        {
            m_vehicle_data[num_vehicles] = new_vehicles[i];
            num_vehicles++;
        }
    }

    //now that new vehicles are added, can reset queue
    reset_new_veh();
}

void vehicle::reset_new_veh(void)
{
    num_new_vehicles = 0;
    for (int i = 0; i < MAX_VEHICLES; i++)
    {
        new_vehicles[i].veh_id = 0;
        new_vehicles[i].veh_pos_x = 0;
        new_vehicles[i].veh_pos_y = 0;
        new_vehicles[i].veh_speed = 0;
        new_vehicles[i].veh_heading = 0;
        new_vehicles[i].time_since_update = 0;
        
        #ifdef PLATOONING_ENABLE
        new_vehicles[i].platooning = false;
        new_vehicles[i].platoon_set_speed = 0;
        #endif
    }
}

void vehicle::update_int_data(double dt)
{
    //first update existing ints, delete ones that have coasted too long
    for (int i = 0; i < num_ints; i++)
    {
        bool match_found = false;
        unsigned int existing_id = m_int_data[i].int_id;
        for (int j = 0; j < num_new_ints; j++)
        {
            if (existing_id == new_ints[j].int_id)
            {
                //update
                match_found = true;
                m_int_data[i] = new_ints[j];
                break;
            }
        }
        if (!match_found)
        {
            //no update, increase time since last update
            m_int_data[i].time_since_update += dt;
            if (m_int_data[i].time_since_update > MAX_COASTING)
            {
                for (int j = i; j < (num_vehicles - 1); j++)
                {
                    m_int_data[j] = m_int_data[j + 1];
                }
                //reset last int
                m_int_data[num_ints - 1].int_id = 0;

                for (int j = 0; j < 4; j++)
                {
                    m_int_data[num_ints - 1].int_ent_pos_x[j] = 0;
                    m_int_data[num_ints - 1].int_ent_pos_y[j] = 0;
                    m_int_data[num_ints - 1].int_ent_headings[j] = 0;
                }
                m_int_data[num_ints - 1].int_length = 0;

                //initialize to GREEN, not to be used since num_ints = 0
                m_int_data[num_ints - 1].int_1_state = GREEN;
                m_int_data[num_ints - 1].int_1_next_state = GREEN;
                m_int_data[num_ints - 1].int_1_time_to_switch = 0;
                m_int_data[num_ints - 1].int_2_state = GREEN;
                m_int_data[num_ints - 1].int_2_next_state = GREEN;
                m_int_data[num_ints - 1].int_2_time_to_switch = 0;

                #ifdef STOP_SIGN_ENABLE
                m_int_data[num_ints - 1].int_type = SIGNAL;
                m_int_data[num_ints - 1].int_next_veh_id = 0;
                #endif

                num_ints--;
                i--;
            }
        }
    }

    //now add new ints
    for (int i = 0; i < num_new_ints; i++)
    {
        bool match_found = false;
        unsigned int new_id = new_ints[i].int_id;
        for (int j = 0; j < num_ints; j++)
        {
            if (new_id == m_int_data[j].int_id)
            {
                match_found = true;
                break;
            }
        }
        if (!match_found && num_ints < MAX_INTS)
        {
            m_int_data[num_ints] = new_ints[i];
            num_ints++;
        }
    }
    //now that new ints are added, can reset queue
    reset_new_int();
}

void vehicle::reset_new_int(void)
{
    num_new_ints = 0;
    for (int i = 0; i < MAX_INTS; i++)
    {
        new_ints[i].int_id = 0;

        for (int j = 0; j < 4; j++)
        {
            new_ints[i].int_ent_pos_x[j] = 0;
            new_ints[i].int_ent_pos_y[j] = 0;
            new_ints[i].int_ent_headings[j] = 0;
        }
        new_ints[i].int_length = 0;

        //initialize to GREEN, not to be used since num_ints = 0
        new_ints[i].int_1_state = GREEN;
        new_ints[i].int_1_next_state = GREEN;
        new_ints[i].int_1_time_to_switch = 0;
        new_ints[i].int_2_state = GREEN;
        new_ints[i].int_2_next_state = GREEN;
        new_ints[i].int_2_time_to_switch = 0;

        #ifdef STOP_SIGN_ENABLE
        new_ints[i].int_type = SIGNAL;
        new_ints[i].int_next_veh_id = 0;
        #endif
    }
}

void vehicle::update_lead_veh(void)
{
    //need to find closest vehicle (if any) in path
    //since we have cars as points but in reality have width, need to account for that when searching

    bool lead_found = false;
    double lead_dist;
    s_vehicle_data lead_vehicle;

    for (int i = 0; i < num_vehicles; i++)
    {
        //first check if in path
        //coast x and y of track
        double coasted_x = m_vehicle_data[i].veh_pos_x + m_vehicle_data[i].time_since_update*m_vehicle_data[i].veh_speed*cos(M_PI/180*m_vehicle_data[i].veh_heading);
        double coasted_y = m_vehicle_data[i].veh_pos_y + m_vehicle_data[i].time_since_update*m_vehicle_data[i].veh_speed*sin(M_PI/180*m_vehicle_data[i].veh_heading);
        
        double x1, x2, y1, y2, angle;
        angle = m_ego_heading + 90.0;
        x1 = coasted_ego_pos_x + INPATH_WIDTH*cos(M_PI/180*angle);
        x2 = coasted_ego_pos_x - INPATH_WIDTH*cos(M_PI/180*angle);
        y1 = coasted_ego_pos_y + INPATH_WIDTH*sin(M_PI/180*angle);
        y2 = coasted_ego_pos_y - INPATH_WIDTH*sin(M_PI/180*angle);

        bool in_path_y, in_path_x;
        double x1_dist = coasted_x - x1;
        double x2_dist = coasted_x - x2;
        double y1_dist = coasted_y - y1;
        double y2_dist = coasted_y - y2;

        if (m_ego_heading == 0 || m_ego_heading == 180)
        {
            #ifdef DEBUG
            std::cout << "update_lead_veh: Up/Down\n";
            #endif
            //going straight up or down, y dist will never change
            //first see if y_dist within ranges
            if ((coasted_y >= y1 && coasted_y <= y2) || (coasted_y >= y2 && coasted_y <= y1))
            {
                in_path_y = true;
            }
            else
            {
                in_path_y = false;
            }

            //now see if we are approaching or going away from object
            if (m_ego_heading == 0 && (x1_dist >= 0 || x2_dist >= 0)) //going up
            {
                in_path_x = true;
            }
            else if (m_ego_heading == 180 && (x1_dist <= 0 || x2_dist <= 0)) //going down
            {
                in_path_x = true;
            }
            else
            {
                in_path_x = false;
            }
        }
        else if (m_ego_heading == 90 || m_ego_heading == 270)
        {
            #ifdef DEBUG
            std::cout << "update_lead_veh: Left/Right\n";
            #endif
            //going straight left or right, x dist will never change
            //first see if x_dist within ranges
            if ((coasted_x >= x1 && coasted_x <= x2) || (coasted_x >= x2 && coasted_x <= x1))
            {
                in_path_x = true;
            }
            else
            {
                in_path_x = false;
            }

            //now see if we are approaching or going away from object
            if (m_ego_heading == 90 && (y1_dist >= 0 || y2_dist >= 0)) //going up
            {
                in_path_y = true;
            }
            else if (m_ego_heading == 270 && (y1_dist <= 0 || y2_dist <= 0)) //going down
            {
                in_path_y = true;
            }
            else
            {
                in_path_y = false;
            }
        }
        else
        {
            #ifdef DEBUG
            std::cout << "update_lead_veh: Both Directions\n";
            #endif
            //no special angles to consider, both dists will change
            //start with x
            double h1_sol = x1_dist/cos(M_PI/180*m_ego_heading);
            double h2_sol = x2_dist/cos(M_PI/180*m_ego_heading);

            //check positive (forward)
            if (h1_sol >= 0 || h2_sol >= 0)
            {
                in_path_x = true;
            }
            else
            {
                in_path_x = false;
            }

            //see where y would be
            double y1_sol = y1 + h1_sol*sin(M_PI/180*m_ego_heading);
            double y2_sol = y2 + h2_sol*sin(M_PI/180*m_ego_heading);

            //now see if in range for y
            if ((y1_sol >= coasted_y && coasted_y <= y2_sol) || (y2_sol >= coasted_y && coasted_y <= y1_sol))
            {
                in_path_y = true;
            }
            else
            {
                in_path_y = false;
            }
        }

        #ifdef DEBUG
        std::cout << "update_lead_veh: in_path_x = " << in_path_x << "\tin_path_y = " << in_path_y << "\n";
        #endif

        //now see if our coasted_y is between the sol_y1 and sol_y2
        if (in_path_x && in_path_y)
        {
            //in path, get distance
            double dist_x = coasted_x - coasted_ego_pos_x;
            double dist_y = coasted_y - coasted_ego_pos_y;
            double dist = sqrt(pow(dist_x, 2) + pow(dist_y, 2)); //dont need to square since using for comparison
            
            if (!lead_found)
            {
                lead_found = true;
                lead_dist = dist;
                lead_vehicle = m_vehicle_data[i];
            }
            else if (lead_found && lead_dist > dist)
            {
                lead_dist = dist;
                lead_vehicle = m_vehicle_data[i];
            }
        }
    }  

    //now have lead, just need to convert to ego's coords (pos x and vel x)
    if (lead_found)
    {
        lead_exists = true;

        //need to get rel pos x and rel vel x
        double x_diff = lead_vehicle.veh_pos_x - coasted_ego_pos_x;
        double y_diff = lead_vehicle.veh_pos_y - coasted_ego_pos_y;
        double diff = pow(x_diff, 2) + pow(y_diff, 2);

        //need to check these calculations
        lead_rel_pos_x = sqrt(diff)*cos(atan(y_diff/x_diff) - M_PI/180*m_ego_heading);

        double heading_diff = lead_vehicle.veh_heading - m_ego_heading;
        lead_rel_vel_x = lead_vehicle.veh_speed*cos(M_PI/180*heading_diff) - coasted_ego_speed; 

        #ifdef PLATOONING_ENABLE
        lead_platooning = lead_vehicle.platooning;
        lead_platoon_set_speed = lead_vehicle.platoon_set_speed;
        #endif
    }
    else
    {
        lead_exists = false;
        lead_rel_pos_x = 0;
        lead_rel_vel_x = 0;

        #ifdef PLATOONING_ENABLE
        lead_platooning = false;
        lead_platoon_set_speed = 0;
        #endif
    }

    #ifdef DEBUG
    std::cout << "update_lead_veh: lead_exists = " << lead_exists << "\tlead_rel_pos_x = " << lead_rel_pos_x << "\tlead_rel_vel_x = " << lead_rel_vel_x << "\n";
    #endif
}

#ifdef DRIVER
void vehicle::update_rel_int(double dt)
#else
void vehicle::update_rel_int(void)
#endif
{
    bool int_found = false;
    double closest_int_dist;
    double closest_int_length;
    double int_update_time;

    for (int i = 0; i < num_ints; i++)
    {
        //check each entrance
        for (int j = 0; j < 4; j++)
        {
            double heading_diff = abs(m_int_data[i].int_ent_headings[j] - m_ego_heading);
            if (heading_diff > 180)
            {
                heading_diff = 360 - heading_diff;
            }

            if (heading_diff < INLANE_HEADING)
            {
                double x_dist = m_int_data[i].int_ent_pos_x[j] - coasted_ego_pos_x;
                double y_dist = m_int_data[i].int_ent_pos_y[j] - coasted_ego_pos_y;
                double dist = sqrt(pow(x_dist, 2) + pow(y_dist, 2)); //dont need to square since just for comparing

                //need to see if approaching or leaving
                //assume const vel and see if time to reach is positive
                double x_time, y_time;
                if (abs(x_dist) < INPATH_WIDTH)
                {
                    x_time = 0;
                }
                else
                {
                    x_time = x_dist/cos(M_PI/180*m_ego_heading);
                }
                
                if (abs(y_dist) < INPATH_WIDTH)
                {
                    y_time = 0;
                }
                else
                {
                    y_time = y_dist/sin(M_PI/180*m_ego_heading);
                }
                 
                bool pos_times = x_time >= 0 && y_time >= 0;

                if (!int_found && pos_times)
                {
                    int_found = true;
                    closest_int_dist = dist;
                    closest_int_length = m_int_data[i].int_length;
                    int_update_time = m_int_data[i].time_since_update;
                    #ifdef STOP_SIGN_ENABLE
                    int_type = m_int_data[i].int_type;
                    int_next_id = m_int_data[i].int_next_veh_id;
                    #endif

                    if (j < 2)
                    {
                        //Direction #1
                        int_state = m_int_data[i].int_1_state;
                        int_next_state = m_int_data[i].int_1_next_state;
                        int_time_to_switch = m_int_data[i].int_1_time_to_switch;
                    }
                    else
                    {
                        //Direction #2
                        int_state = m_int_data[i].int_2_state;
                        int_next_state = m_int_data[i].int_2_next_state;
                        int_time_to_switch = m_int_data[i].int_2_time_to_switch;
                    }
                    
                }
                else if (int_found && pos_times && closest_int_dist > dist)
                {
                    closest_int_dist = dist;
                    closest_int_length = m_int_data[i].int_length;
                    int_update_time = m_int_data[i].time_since_update;
                    #ifdef STOP_SIGN_ENABLE
                    int_type = m_int_data[i].int_type;
                    int_next_id = m_int_data[i].int_next_veh_id;
                    #endif

                    if (j < 2)
                    {
                        //Direction #1
                        int_state = m_int_data[i].int_1_state;
                        int_next_state = m_int_data[i].int_1_next_state;
                        int_time_to_switch = m_int_data[i].int_1_time_to_switch;
                    }
                    else
                    {
                        //Direction #2
                        int_state = m_int_data[i].int_2_state;
                        int_next_state = m_int_data[i].int_2_next_state;
                        int_time_to_switch = m_int_data[i].int_2_time_to_switch;
                    }
                }
            }
        }
    }
    approaching_int = int_found;
    if (int_found)
    {
        int_ent_dist = closest_int_dist;
        int_exit_dist = closest_int_dist + closest_int_length;
        int_time_to_switch -= int_update_time;
        #ifdef STOP_SIGN_ENABLE
        if (int_type == SIGNAL && int_time_to_switch < 0) //only care if signaled light
        #else
        if (int_time_to_switch < 0)
        #endif
        {
            //light switched based on last received data, need to account for
            switch (int_next_state)
            {
                //since int_time_to_switch is negative this removes the extra time from dt
                case GREEN:
                    int_state = GREEN;
                    int_next_state = YELLOW;
                    int_time_to_switch = int_time_to_switch + MIN_RED_GREEN_TIME;
                    break;
                case YELLOW:
                    int_state = YELLOW;
                    int_next_state = RED;
                    int_time_to_switch = int_time_to_switch + MIN_YELLOW;
                    break;
                case RED:
                    int_state = RED;
                    int_next_state = GREEN;
                    int_time_to_switch = int_time_to_switch + MIN_RED_GREEN_TIME;
                    break;
            }
        }
    #ifdef DRIVER
    if (int_state != last_state)
    {
        time_since_switch += dt;
        if (time_since_switch >= DRIVER_REACTION)
        {
            last_state = int_state;
            time_since_switch = 0;
        }
    }
    #endif

    }
    else
    {
        int_ent_dist = 0;
        int_exit_dist = 0;
        int_state = GREEN;
        int_next_state = GREEN;
        int_time_to_switch = 0;

        #ifdef STOP_SIGN_ENABLE
        int_type = SIGNAL;
        int_next_id = 0;
        #endif
    }

    #ifdef DEBUG
    std::cout << "update_rel_int: int_found = " << int_found << "\tint_ent_dist = " << int_ent_dist << "\tint_exit_dist = " << int_exit_dist << "\n";
    std::cout << "update_rel_int: int_state = " << int_state << "\tint_next_state = " << int_next_state << "\tint_time_to_switch = " << int_time_to_switch << "\n";
    #endif

}

void vehicle::init_veh_data(void)
{
    num_vehicles = 0;
    num_new_vehicles = 0;

    for (int i = 0; i < MAX_VEHICLES; i++)
    {
        //existing vehicles
        m_vehicle_data[i].veh_id = 0;
        m_vehicle_data[i].veh_pos_x = 0;
        m_vehicle_data[i].veh_pos_y = 0;
        m_vehicle_data[i].veh_speed = 0;
        m_vehicle_data[i].veh_heading = 0;

        m_vehicle_data[i].time_since_update = 0;

        #ifdef PLATOONING_ENABLE
        m_vehicle_data[i].platooning = false;
        m_vehicle_data[i].platoon_set_speed = 0;
        #endif

        //new vehicles
        new_vehicles[i] = m_vehicle_data[i];
    }
}

void vehicle::init_int_data(void)
{
    num_ints = 0;
    num_new_ints = 0;

    for (int i = 0; i < MAX_INTS; i++)
    {
        //existing intersections
        m_int_data[i].int_id = 0;

        for (int j = 0; j < 4; j++)
        {
            m_int_data[i].int_ent_pos_x[j] = 0;
            m_int_data[i].int_ent_pos_y[j] = 0;
            m_int_data[i].int_ent_headings[j] = 0;
        }
        m_int_data[i].int_length = 0;

        //initialize to GREEN, not to be used since num_ints = 0
        m_int_data[i].int_1_state = GREEN;
        m_int_data[i].int_1_next_state = GREEN;
        m_int_data[i].int_1_time_to_switch = 0;
        m_int_data[i].int_2_state = GREEN;
        m_int_data[i].int_2_next_state = GREEN;
        m_int_data[i].int_2_time_to_switch = 0;

        #ifdef STOP_SIGN_ENABLE
        m_int_data[i].int_type = SIGNAL;
        m_int_data[i].int_next_veh_id = 0;
        #endif

        //new intersections
        new_ints[i] = m_int_data[i];
    }
}

void vehicle::init_outputs(void)
{
    system_accel_req = 0;
    c_accel_req = 0;
    c_driver_alert = false;
    c_platoon_set_speed = 0;
}

void vehicle::init_lead_veh(void)
{
    lead_exists = false;
    lead_rel_pos_x = 0;
    lead_rel_vel_x = 0;

    #ifdef PLATOONING_ENABLE
    lead_platooning = false;
    lead_platoon_set_speed = 0;
    #endif
}

void vehicle::init_rel_int(void)
{
    approaching_int = false;
    int_ent_dist = 0;
    int_exit_dist = 0;
    int_state = GREEN;
    int_next_state = GREEN;
    int_time_to_switch = 0;

    #ifdef STOP_SIGN_ENABLE
    int_type = SIGNAL;
    int_next_id = 0;
    #endif

    #ifdef DRIVER
    last_state = int_state;
    time_since_switch = DRIVER_REACTION + 1.0;
    #endif
}

void vehicle::validate_set_speed(void)
{
    if (p_set_speed < MIN_SET_SPEED)
    {
        std::cout << "Set speed below minimum! Setting to minimum...\n";
        p_set_speed = MIN_SET_SPEED;
    }
    else if (p_set_speed > k_max_speed)
    {
        std::cout << "Set speed above maximum vehicle speed! Setting to maximum...\n";
        p_set_speed = k_max_speed;
    }
}