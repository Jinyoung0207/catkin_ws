#include "formation_order.hpp"
int key_in(void)
{
    int ch;
    struct termios buf;
    struct termios save;
    tcgetattr(0,&save);
    buf = save;
    buf.c_lflag &= ~(ICANON|ECHO);
    buf.c_cc[VMIN] = 1;
    buf.c_cc[VTIME] = 0;
    tcsetattr(0, TCSAFLUSH, &buf);
    ch = getchar();
    tcsetattr(0, TCSAFLUSH, &save);
    return ch;
}

void formation_order::formation_init_set()
{
    // UGV formation init setting

    std::vector<geometry_msgs::Point> formation_vec;
    geometry_msgs::Point temp_point;
    temp_point.x = (0.0);
    temp_point.y = (0.0);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (-sqrt(3) * (scale_nor / 2));
    temp_point.y = (-(scale_nor / 2));
    formation_vec.emplace_back(temp_point);
    temp_point.x = (-sqrt(3) * (scale_nor / 2));
    temp_point.y = ((scale_nor / 2));
    formation_vec.emplace_back(temp_point);
    temp_point.x = ((-sqrt(3) * (scale_nor / 2))*2);
    temp_point.y = (-scale_nor);
    formation_vec.emplace_back(temp_point);
    temp_point.x = ((-sqrt(3) * (scale_nor / 2))*2);
    temp_point.y = (scale_nor);
    formation_vec.emplace_back(temp_point);
    ugv_formation_list.insert(std::pair<std::string,std::vector<geometry_msgs::Point>>("1",formation_vec));
    formation_vec.clear();

    temp_point.x = (0.0);
    temp_point.y = (0.0);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (-sqrt(3) * (scale_nor / 2));
    temp_point.y = (-(scale_nor / 2));
    formation_vec.emplace_back(temp_point);
    temp_point.x = (-sqrt(3) * (scale_nor / 2));
    temp_point.y = ((scale_nor / 2));
    formation_vec.emplace_back(temp_point);
    temp_point.x = (-sqrt(3) * (scale_nor));
    temp_point.y = (-scale_nor / 2);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (-sqrt(3) * (scale_nor));
    temp_point.y = (scale_nor / 2);
    formation_vec.emplace_back(temp_point);
    ugv_formation_list.insert(std::pair<std::string,std::vector<geometry_msgs::Point>>("2",formation_vec));
    formation_vec.clear();

    temp_point.x = (0.0);
    temp_point.y = (0.0);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (-scale_nor);
    temp_point.y = (0.0);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (-scale_nor * 2);
    temp_point.y = (0.0);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (-scale_nor * 3);
    temp_point.y = (0.0);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (-scale_nor * 4);
    temp_point.y = (0.0);
    formation_vec.emplace_back(temp_point);
    ugv_formation_list.insert(std::pair<std::string,std::vector<geometry_msgs::Point>>("3",formation_vec));
    formation_vec.clear();

    temp_point.x = (0.0);
    temp_point.y = (0.0);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (0.0);
    temp_point.y = (-(scale_nor));
    formation_vec.emplace_back(temp_point);
    temp_point.x = (0.0);
    temp_point.y = ((scale_nor));
    formation_vec.emplace_back(temp_point);
    temp_point.x = (0.0);
    temp_point.y = (-(scale_nor * 2));
    formation_vec.emplace_back(temp_point);
    temp_point.x = (0.0);
    temp_point.y = ((scale_nor * 2));
    formation_vec.emplace_back(temp_point);
    ugv_formation_list.insert(std::pair<std::string,std::vector<geometry_msgs::Point>>("0",formation_vec));
    formation_vec.clear();

    // UAV formation init setting
    
    temp_point.x = (0.0);
    temp_point.y = (0.0);
    temp_point.z = (uav_z_max);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (-sqrt(3) * (scale_nor / 2));
    temp_point.y = (-(scale_nor / 2));
    temp_point.z = (uav_z_max);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (-sqrt(3) * (scale_nor / 2));
    temp_point.y = ((scale_nor / 2));
    temp_point.z = (uav_z_max);
    formation_vec.emplace_back(temp_point);
    temp_point.x = ((-sqrt(3) * (scale_nor / 2))*2);
    temp_point.y = (-scale_nor);
    temp_point.z = (uav_z_max);
    formation_vec.emplace_back(temp_point);
    temp_point.x = ((-sqrt(3) * (scale_nor / 2))*2);
    temp_point.y = (scale_nor);
    temp_point.z = (uav_z_max);
    formation_vec.emplace_back(temp_point);
    uav_formation_list.insert(std::pair<std::string,std::vector<geometry_msgs::Point>>("1",formation_vec));
    formation_vec.clear();

    temp_point.x = (0.0);
    temp_point.y = (0.0);
    temp_point.z = (uav_z_max);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (-sqrt(3) * (scale_nor / 2));
    temp_point.y = (-(scale_nor / 2));
    temp_point.z = (uav_z_max);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (-sqrt(3) * (scale_nor / 2));
    temp_point.y = ((scale_nor / 2));
    temp_point.z = (uav_z_max);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (-sqrt(3) * (scale_nor));
    temp_point.y = (-scale_nor / 2);
    temp_point.z = (uav_z_max);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (-sqrt(3) * (scale_nor));
    temp_point.y = (scale_nor / 2);
    temp_point.z = (uav_z_max);
    formation_vec.emplace_back(temp_point);
    uav_formation_list.insert(std::pair<std::string,std::vector<geometry_msgs::Point>>("2",formation_vec));
    formation_vec.clear();

    temp_point.x = (0.0);
    temp_point.y = (0.0);
    temp_point.z = (uav_z_max);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (-scale_nor);
    temp_point.y = (0.0);
    temp_point.z = (uav_z_max);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (-scale_nor * 2);
    temp_point.y = (0.0);
    temp_point.z = (uav_z_max);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (-scale_nor * 3);
    temp_point.y = (0.0);
    temp_point.z = (uav_z_max);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (-scale_nor * 4);
    temp_point.y = (0.0);
    temp_point.z = (uav_z_max);
    formation_vec.emplace_back(temp_point);
    uav_formation_list.insert(std::pair<std::string,std::vector<geometry_msgs::Point>>("3",formation_vec));
    formation_vec.clear();
    
    temp_point.x = (0.0);
    temp_point.y = (0.0);
    temp_point.z = (uav_z_max);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (0.0);
    temp_point.y = (-scale_nor);
    temp_point.z = (uav_z_max + 1.0);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (0.0);
    temp_point.y = (scale_nor);
    temp_point.z = (uav_z_max + 1.0);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (0.0);
    temp_point.y = (-scale_nor * 2);
    temp_point.z = (uav_z_max + 2.0);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (0.0);
    temp_point.y = (scale_nor * 2);
    temp_point.z = (uav_z_max + 2.0);
    formation_vec.emplace_back(temp_point);
    uav_formation_list.insert(std::pair<std::string,std::vector<geometry_msgs::Point>>("4",formation_vec));
    formation_vec.clear();

    temp_point.x = (0.0);
    temp_point.y = (0.0);
    temp_point.z = (uav_z_max);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (0.0);
    temp_point.y = (-(scale_nor));
    temp_point.z = (uav_z_max);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (0.0);
    temp_point.y = ((scale_nor));
    temp_point.z = (uav_z_max);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (0.0);
    temp_point.y = (-(scale_nor * 2));
    temp_point.z = (uav_z_max);
    formation_vec.emplace_back(temp_point);
    temp_point.x = (0.0);
    temp_point.y = ((scale_nor * 2));
    temp_point.z = (uav_z_max);
    formation_vec.emplace_back(temp_point);
    uav_formation_list.insert(std::pair<std::string,std::vector<geometry_msgs::Point>>("0",formation_vec));
    formation_vec.clear();
}

void formation_order::reset()
{
    if(formation_change_end)
    {
        if(uav_key_input_in || all_key_input_in)
        {
            uav_formation_pattern = uav_formation_input;
        }
        if(ugv_key_input_in || all_key_input_in)
        {
            ugv_formation_pattern = ugv_formation_input;
        }
        
        set_key_input_in = false;
        set_fir_key_input_in = false;

        uav_key_input_in = false;
        uav_fir_key_input_in = false;

        ugv_key_input_in = false;
        ugv_fir_key_input_in = false;

        all_key_input_in = false;
        all_fir_key_input_in = false;
        
        formation_order_end = false;
        formation_set_end = false;
        formation_sampling = false;
        formation_changing = false;
        formation_change_end = false;

        choose_set_input = 0;
        ugv_formation_input = 0;
        uav_formation_input = 0;
        all_formation_input = 0;

        uav_changing_point_id_max = 0;
        ugv_changing_point_id_max = 0;

        uav_changing_point_id = 0;
        ugv_changing_point_id = 0;

        uav_assignment.clear();
        ugv_assignment.clear();

        uav_change_formation_vec.clear();
        ugv_change_formation_vec.clear();
    }

    if(formation_changing)
    {
        ++ loop_count;
    }
    
    if(formation_changing && (uav_key_input_in || all_key_input_in))
    {
        uav_formation_msg.poses.clear();
    }
    if(formation_changing && (ugv_key_input_in || all_key_input_in))
    {
        ugv_formation_msg.poses.clear();
    }

}

void formation_order::publish()
{
    if(formation_changing)
    {
        uav_formation_pub.publish(uav_formation_msg);
        ugv_formation_pub.publish(ugv_formation_msg);
    }
    
}

void formation_order::msg_set()
{
    if(formation_changing)
    {
        uav_formation_msg.header.stamp = ros::Time::now();
        uav_formation_msg.header.frame_id = frame_id;

        ugv_formation_msg.header.stamp = ros::Time::now();
        ugv_formation_msg.header.frame_id = frame_id;
    }
    
}

void formation_order::formation_reset()
{
    if(formation_change_end)
    {
        if(!uav_assignment.empty())
        {
            std::vector<geometry_msgs::Point> formation_idx;
            std::string cur_key = std::to_string(uav_formation_input);
            for(const auto id : uav_assignment)
            {
                formation_idx.emplace_back(uav_formation_list[cur_key][id]);
            }
            uav_formation_list[cur_key] = formation_idx;
        }
        if(!ugv_assignment.empty())
        {
            std::vector<geometry_msgs::Point> formation_idx;
            std::string cur_key = std::to_string(ugv_formation_input);
            for(const auto id : ugv_assignment)
            {
                formation_idx.emplace_back(ugv_formation_list[cur_key][id]);
            }
            ugv_formation_list[cur_key] = formation_idx;
        }
    }
}

void formation_order::formation_changing_point()
{
    if(formation_changing && formation_order_end)
    {
        std::cout << "==============================================" << "\n";
        if(uav_key_input_in || all_key_input_in)
        {
            
            for(int uav_num = 0; uav_num < uav_change_formation_vec.size(); ++uav_num)
            {
                geometry_msgs::Pose node;
                
                if(uav_change_formation_vec[uav_num].size() <= uav_changing_point_id)
                {
                    node.position.x = uav_change_formation_vec[uav_num][uav_change_formation_vec[uav_num].size() - 1].x;
                    node.position.y = uav_change_formation_vec[uav_num][uav_change_formation_vec[uav_num].size() - 1].y;
                    node.position.z = uav_change_formation_vec[uav_num][uav_change_formation_vec[uav_num].size() - 1].z;
                }
                else
                {
                    node.position.x = uav_change_formation_vec[uav_num][uav_changing_point_id].x;
                    node.position.y = uav_change_formation_vec[uav_num][uav_changing_point_id].y;
                    node.position.z = uav_change_formation_vec[uav_num][uav_changing_point_id].z;
                }
                
                uav_formation_msg.poses.emplace_back(node);
                
            }
            std::cout << GREEN"Changing UAV formation [" << uav_changing_point_id << "]/[" << uav_changing_point_id_max << "] ..."END << "\n";
        }
         
        if(ugv_key_input_in || all_key_input_in)
        {
            for(int ugv_num = 0; ugv_num < ugv_change_formation_vec.size(); ++ugv_num)
            {
                geometry_msgs::Pose node;
                
                if(ugv_change_formation_vec[ugv_num].size() <= ugv_changing_point_id)
                {
                    node.position.x = ugv_change_formation_vec[ugv_num][ugv_change_formation_vec[ugv_num].size() - 1].x;
                    node.position.y = ugv_change_formation_vec[ugv_num][ugv_change_formation_vec[ugv_num].size() - 1].y;
                }
                else
                {
                    node.position.x = ugv_change_formation_vec[ugv_num][ugv_changing_point_id].x;
                    node.position.y = ugv_change_formation_vec[ugv_num][ugv_changing_point_id].y;
                }
                ugv_formation_msg.poses.emplace_back(node);
            }
            std::cout << GREEN"Changing UGV formation [" << ugv_changing_point_id << "]/[" << ugv_changing_point_id_max << "] ..."END << "\n";
        }
        std::cout << std::flush;
        
        if(uav_key_input_in)
        {
            if(uav_changing_point_id_max == uav_changing_point_id)
            {
                formation_change_end = true;
            }
            else
            {
                formation_change_end = false;
            }
            ++ uav_changing_point_id;
        }
        else if(ugv_key_input_in)
        {
            if(ugv_changing_point_id_max == ugv_changing_point_id)
            {
                formation_change_end = true;
            }
            else
            {
                formation_change_end = false;
            }
            ++ ugv_changing_point_id;
        }
        else if(all_key_input_in)
        {
            if(uav_changing_point_id_max == uav_changing_point_id && ugv_changing_point_id_max == ugv_changing_point_id)
            {
                formation_change_end = true;
            }
            else
            {
                formation_change_end = false;
            }

            ++ uav_changing_point_id;
            ++ ugv_changing_point_id;
        }
    }
}

void formation_order::formation_sampling_point()
{
    if(!formation_sampling && formation_order_end)
    {
        std::cout << "Formation change root sampling ... ";
        std::string cur_uav_pattern = std::to_string(uav_formation_input);
        std::string cur_ugv_pattern = std::to_string(ugv_formation_input);
        if(uav_key_input_in || all_key_input_in)
        {
            for(int id = 0; id < uav_prev_formation.size() ; ++id)
            {
                std::vector<geometry_msgs::Point> uav_change_point_vec;

                double distance = sqrt(pow((uav_prev_formation[id].x - uav_formation_list[cur_uav_pattern][uav_assignment[id]].x),2) + 
                                       pow((uav_prev_formation[id].y - uav_formation_list[cur_uav_pattern][uav_assignment[id]].y),2) +
                                       pow((uav_prev_formation[id].z - uav_formation_list[cur_uav_pattern][uav_assignment[id]].z),2));

                int sampling_node_num = (int)std::ceil(distance / (set_vel * (1/rate)));

                for(int j = 0; j < sampling_node_num; ++j)
                {
                    geometry_msgs::Point sampling_node;
                    sampling_node.x = uav_prev_formation[id].x + ((uav_formation_list[cur_uav_pattern][uav_assignment[id]].x - uav_prev_formation[id].x) / (double)sampling_node_num) * j;
                    sampling_node.y = uav_prev_formation[id].y + ((uav_formation_list[cur_uav_pattern][uav_assignment[id]].y - uav_prev_formation[id].y) / (double)sampling_node_num) * j;
                    sampling_node.z = uav_prev_formation[id].z + ((uav_formation_list[cur_uav_pattern][uav_assignment[id]].z - uav_prev_formation[id].z) / (double)sampling_node_num) * j;
                    uav_change_point_vec.emplace_back(sampling_node);
                }
                uav_change_point_vec.emplace_back(uav_formation_list[cur_uav_pattern][uav_assignment[id]]);
                uav_change_formation_vec.emplace_back(uav_change_point_vec);
            }

            for(const auto form_vec : uav_change_formation_vec)
            {
                if(uav_changing_point_id_max < form_vec.size())
                {
                    uav_changing_point_id_max = form_vec.size();
                }
            }
        }

        if(ugv_key_input_in || all_key_input_in)
        {
            for(int id = 0; id < ugv_prev_formation.size() ; ++id)
            {
                std::vector<geometry_msgs::Point> ugv_change_point_vec;

                double distance = sqrt(pow((ugv_prev_formation[id].x - ugv_formation_list[cur_ugv_pattern][ugv_assignment[id]].x),2) + 
                                       pow((ugv_prev_formation[id].y - ugv_formation_list[cur_ugv_pattern][ugv_assignment[id]].y),2));

                int sampling_node_num = (int)std::ceil(distance / (set_vel * (1/rate)));

                for(int j = 0; j < sampling_node_num; ++j)
                {
                    geometry_msgs::Point sampling_node;
                    sampling_node.x = ugv_prev_formation[id].x + ((ugv_formation_list[cur_ugv_pattern][ugv_assignment[id]].x - ugv_prev_formation[id].x) / (double)sampling_node_num) * j;
                    sampling_node.y = ugv_prev_formation[id].y + ((ugv_formation_list[cur_ugv_pattern][ugv_assignment[id]].y - ugv_prev_formation[id].y) / (double)sampling_node_num) * j;
                    ugv_change_point_vec.emplace_back(sampling_node);
                }
                ugv_change_point_vec.emplace_back(ugv_formation_list[cur_ugv_pattern][ugv_assignment[id]]);
                ugv_change_formation_vec.emplace_back(ugv_change_point_vec);
            }

            for(const auto form_vec : ugv_change_formation_vec)
            {
                if(ugv_changing_point_id_max < form_vec.size())
                {
                    ugv_changing_point_id_max = form_vec.size();
                }
            }
        }
        if(!uav_change_formation_vec.empty() || !ugv_change_formation_vec.empty())
        {
            formation_changing = true;
            formation_sampling = true;
            if(all_key_input_in)
            {
                if(ugv_changing_point_id_max > uav_changing_point_id_max)
                {
                    uav_changing_point_id_max = ugv_changing_point_id_max;
                }
                if(ugv_changing_point_id_max < uav_changing_point_id_max)
                {
                    ugv_changing_point_id_max = uav_changing_point_id_max;
                }
            }
            std::cout << GREEN"END"END << "\n";
            std::cout << std::flush;
        }
       
    }

}

void formation_order::formation_hungarian()
{
    if(!formation_set_end && formation_order_end)
    {
        std::cout << "Caculating formation cost ... ";
        if(loop_count == 0)
        {
            uav_prev_formation = uav_formation_list["0"];
            ugv_prev_formation = ugv_formation_list["0"];
        }
        else
        {
            switch (uav_formation_pattern)
            {
            case 1:
                uav_prev_formation = uav_formation_list["1"];
                break;
            case 2:
                uav_prev_formation = uav_formation_list["2"];
                break;
            case 3:
                uav_prev_formation = uav_formation_list["3"];
                break;
            case 4:
                uav_prev_formation = uav_formation_list["4"];
                break;
            default:
                break;
            }

            switch (ugv_formation_pattern)
            {
            case 1:
                ugv_prev_formation = ugv_formation_list["1"];
                break;
            case 2:
                ugv_prev_formation = ugv_formation_list["2"];
                break;
            case 3:
                ugv_prev_formation = ugv_formation_list["3"];
                break;
            default:
                break;
            }
        }
        HungarianAlgorithm hungarian;

        std::vector<std::vector<double>> uav_cost_matrix;
        std::vector<std::vector<double>> ugv_cost_matrix;
        std::string prev_uav_pattern = std::to_string(uav_formation_pattern);
        std::string prev_ugv_pattern = std::to_string(ugv_formation_pattern);

        std::string cur_uav_pattern = std::to_string(uav_formation_input);
        std::string cur_ugv_pattern = std::to_string(ugv_formation_input);

        if(uav_key_input_in || all_key_input_in)
        {
            for(auto past_formation : uav_formation_list[prev_uav_pattern])
            {
                std::vector<double> dis_vec;

                for(const auto cur_formation : uav_formation_list[cur_uav_pattern])
                {
                    double dis = sqrt(pow(past_formation.x - cur_formation.x,2) + pow(past_formation.y - cur_formation.y,2) + pow(past_formation.z - cur_formation.z,2));
                    
                    dis_vec.emplace_back(dis);
                }
                uav_cost_matrix.emplace_back(dis_vec);
                
            }
            double cost = hungarian.Solve(uav_cost_matrix,uav_assignment);
        }
        if(ugv_key_input_in || all_key_input_in)
        {
            for(const auto past_formation : ugv_formation_list[prev_ugv_pattern])
            {
                std::vector<double> dis_vec;

                for(const auto cur_formation : ugv_formation_list[cur_ugv_pattern])
                {
                    double dis = sqrt(pow(past_formation.x - cur_formation.x,2) + pow(past_formation.y - cur_formation.y,2));
                    
                    dis_vec.emplace_back(dis);
                }
                ugv_cost_matrix.emplace_back(dis_vec);
            }
            double cost = hungarian.Solve(ugv_cost_matrix,ugv_assignment);
        }
        if(!uav_cost_matrix.empty() || !ugv_cost_matrix.empty())
        {
            std::cout << GREEN"END"END << "\n";
            std::cout << std::flush;
            formation_set_end = true;
        }
    }
}

void formation_order::input_set()
{
    if(formation_order_end)
    {
        if(all_key_input_in)
        {
            ugv_formation_input = all_formation_input;
            uav_formation_input = all_formation_input;
        }
    }
}

void formation_order::key_read()
{
    if(!set_fir_key_input_in)
    {
        std::cout << "==============================================" << "\n";
        std::cout << CYAN"Please push the button [1 ~ 3]"END << "\n";
        std::cout << "[1] : Formation new order only " << GREEN"UAV"END << "\n";
        std::cout << "[2] : Formation new order only " << GREEN"UGV"END << "\n";
        std::cout << "[3] : Formation new order " << GREEN"UGV && UAV"END << "\n";
        std::cout << "If you want exit code Please push [c]" << "\n";
        std::cout << std::flush;
        choose_set_input = key_in();
        set_fir_key_input_in = true;
        choose_set_input -= 48;
    }

    if(choose_set_input == 51)
    {
        std::cout << YELL"EXIT code ..."END << "\n";
        std::cout << "\n";
        std::cout << std::flush;

        ros::shutdown();
    }
    else if(choose_set_input != 51 && choose_set_input < 1 && choose_set_input > 3)
    {
        std::cout << "==============================================" << "\n";
        std::cout << RED"[ERROR] Please push the button [1 ~ 3]"END << "\n";
        std::cout << std::flush;
        choose_set_input = key_in();
        choose_set_input -= 48;
    }
    
    if(choose_set_input >= 1 && choose_set_input <= 3)
    {
        std::cout << "\n";
        if(choose_set_input == 1)
        {
            std::cout << GREEN"Choose UAV foramtion change"END << "\n";
        }
        else if(choose_set_input == 2)
        {
            std::cout << GREEN"Choose UGV foramtion change"END << "\n";
        }
        else if(choose_set_input == 3)
        {
            std::cout << GREEN"Choose UAV && UGV foramtion change"END << "\n";
        }
        set_key_input_in = true;
    }

    if(set_key_input_in)
    {
        std::cout << "==============================================" << "\n";
        if(choose_set_input == 1 && !uav_fir_key_input_in)
        {
            std::cout << CYAN"Please push the button [1 ~ 4]"END << "\n";
            std::cout << "┌[1]──────┬[2]──────┬[3]──────┬[4]──────┐" << "\n";
            std::cout << "│    o    │    o    │    o    │  o   o  │" << "\n";
            std::cout << "│   o o   │   o o   │    o    │   o o   │" << "\n";
            std::cout << "│  o   o  │   o o   │    o    │    o    │" << "\n";
            std::cout << "└─────────┴─────────┴─────────┴─────────┘" << "\n";
            std::cout << "If you want exit code Please push [c]" << "\n";
            std::cout << std::flush;
            uav_formation_input = key_in();
            uav_formation_input -= 48;
            uav_fir_key_input_in = true;
        }
        else if(choose_set_input == 2 && !ugv_fir_key_input_in)
        {
            std::cout << CYAN"Please push the button [1 ~ 3]"END << "\n";
            std::cout << "┌[1]──────┬[2]──────┬[3]──────┐" << "\n";
            std::cout << "│    o    │    o    │    o    │" << "\n";
            std::cout << "│   o o   │   o o   │    o    │" << "\n";
            std::cout << "│  o   o  │   o o   │    o    │" << "\n";
            std::cout << "└─────────┴─────────┴─────────┘" << "\n";
            std::cout << "If you want exit code Please push [c]" << "\n";
            std::cout << std::flush;
            ugv_formation_input = key_in();
            ugv_formation_input -= 48;
            ugv_fir_key_input_in = true;
        }
        else if(choose_set_input == 3 && !all_fir_key_input_in)
        {
            std::cout << CYAN"Please push the button [1 ~ 3]"END << "\n";
            std::cout << "┌[1]──────┬[2]──────┬[3]──────┐" << "\n";
            std::cout << "│    o    │    o    │    o    │" << "\n";
            std::cout << "│   o o   │   o o   │    o    │" << "\n";
            std::cout << "│  o   o  │   o o   │    o    │" << "\n";
            std::cout << "└─────────┴─────────┴─────────┘" << "\n";
            std::cout << "If you want exit code Please push [c]" << "\n";
            std::cout << std::flush;
            all_formation_input = key_in();
            all_formation_input -= 48;
            all_fir_key_input_in = true;
        }
        
        if (uav_formation_input == 51 || ugv_formation_input == 51 || all_formation_input == 51)
        {
            std::cout << YELL"EXIT code ..."END << "\n";
            std::cout << "\n";
            std::cout << std::flush;

            ros::shutdown();
        }
        else if(uav_formation_input != 51 && uav_formation_input < 1 && uav_formation_input > 4 && uav_fir_key_input_in)
        {
            std::cout << "==============================================" << "\n";
            std::cout << "\n";
            std::cout << RED"[ERROR] Please push the button [1 ~ 4]"END << "\n";
            uav_formation_input = key_in();
            uav_formation_input -= 48;
            uav_fir_key_input_in = true;
        }
        else if(ugv_formation_input != 51 && ugv_formation_input < 1 && ugv_formation_input > 3 && ugv_fir_key_input_in)
        {
            std::cout << "==============================================" << "\n";
            std::cout << "\n";
            std::cout << RED"[ERROR] Please push the button [1 ~ 3]"END << "\n";
            ugv_formation_input = key_in();
            ugv_formation_input -= 48;
            ugv_fir_key_input_in = true;
        }
        else if(all_formation_input != 51 && all_formation_input < 1 && all_formation_input > 3 && all_fir_key_input_in)
        {
            std::cout << "==============================================" << "\n";
            std::cout << "\n";
            std::cout << RED"[ERROR] Please push the button [1 ~ 3]"END << "\n";
            all_formation_input = key_in();
            all_formation_input -= 48;
            all_fir_key_input_in = true;
        }   
        std::cout << std::flush;
    }

    if(set_key_input_in && uav_formation_input >= 1 && uav_formation_input <= 4)
    {
        std::cout << GREEN"Formation UAV order end ... && Formation change start !!! "END << "\n";
        formation_order_end = true;
        uav_key_input_in = true;
    }
    else if(set_key_input_in && ugv_formation_input >= 1 && ugv_formation_input <= 3)
    {
        std::cout << GREEN"Formation UGV order end ... && Formation change start !!! "END << "\n";
        formation_order_end = true;
        ugv_key_input_in = true;
    }
    else if(set_key_input_in && all_formation_input >= 1 && all_formation_input <= 3)
    {
        std::cout << GREEN"Formation UAV && UGV order end ... && Formation change start !!! "END << "\n";
        formation_order_end = true;
        all_key_input_in = true;
    }
    std::cout << std::flush;
}

void formation_order::param_read()
{
    nh.getParam("/scale_nor",scale_nor);
    nh.getParam("/set_vel",set_vel);
    nh.getParam("/rate",rate);
    nh.getParam("/uav_z_max",uav_z_max);
    nh.getParam("/frame_id",frame_id);

    dt = 1/rate;

    std::cout << "==============================================" << "\n";
    std::cout << "rate      : " << rate << "\n";
    std::cout << "dt        : " << dt << "\n";
    std::cout << "scale_nor : " << scale_nor << "\n";
    std::cout << "set_vel   : " << set_vel << "\n";
    std::cout << "uav_z_max : " << uav_z_max << "\n";
    std::cout << "frame_id  : " << frame_id << "\n";
    std::cout << std::flush;
}

void formation_order::init()
{
    uav_formation_pub = nh.advertise<geometry_msgs::PoseArray>("/uav_formation_list",1);
    ugv_formation_pub = nh.advertise<geometry_msgs::PoseArray>("/ugv_formation_list",1);
}

void formation_order::run()
{
    // param_read();
    formation_init_set();
    ros::Rate r(rate);
    while(ros::ok())
    {
        ros::spinOnce();
        if(!formation_change_end && !formation_changing)
        {
            key_read();
        }
        input_set();
        formation_hungarian();
        formation_sampling_point();
        formation_changing_point();
        formation_reset();
        msg_set();
        publish();
        reset();
        r.sleep();
    }
    
}
