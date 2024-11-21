#include "formation_order.hpp"

void formation_order::UAV1_init_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    init_uav1=*msg;
}
void formation_order::UAV2_init_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    init_uav2=*msg;
}
void formation_order::UAV3_init_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    init_uav3=*msg;
}
void formation_order::UAV4_init_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    init_uav4=*msg;
}
void formation_order::UAV5_init_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    init_uav5=*msg;
}

void formation_order::UGV1_init_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    init_ugv1=*msg;
}
void formation_order::UGV2_init_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    init_ugv2=*msg;
}
void formation_order::UGV3_init_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    init_ugv3=*msg;
}
void formation_order::UGV4_init_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    init_ugv4=*msg;
}
void formation_order::UGV5_init_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    init_ugv5=*msg;
}
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
    // UGV formation setting
    std::vector<geometry_msgs::Point> formation_vec;
    geometry_msgs::Point temp_point;

    //formation 1
    temp_point.x = form1_ugv1_x * scale;
    temp_point.y = form1_ugv1_y * scale;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form1_ugv2_x * scale;
    temp_point.y = form1_ugv2_y * scale;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form1_ugv3_x * scale;
    temp_point.y = form1_ugv3_y * scale;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form1_ugv4_x * scale;
    temp_point.y = form1_ugv4_y * scale;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form1_ugv5_x * scale;
    temp_point.y = form1_ugv5_y * scale;
    formation_vec.emplace_back(temp_point);
    ugv_formation_list["1"] = formation_vec;
    formation_vec.clear();

    //formation 2
    temp_point.x = form2_ugv1_x * scale;
    temp_point.y = form2_ugv1_y * scale;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form2_ugv2_x * scale;
    temp_point.y = form2_ugv2_y * scale;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form2_ugv3_x * scale;
    temp_point.y = form2_ugv3_y * scale;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form2_ugv4_x * scale;
    temp_point.y = form2_ugv4_y * scale;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form2_ugv5_x * scale;
    temp_point.y = form2_ugv5_y * scale;
    formation_vec.emplace_back(temp_point);
    ugv_formation_list["2"] = formation_vec;
    formation_vec.clear();

    //formation 3
    temp_point.x = form3_ugv1_x * scale;
    temp_point.y = form3_ugv1_y * scale;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form3_ugv2_x * scale;
    temp_point.y = form3_ugv2_y * scale;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form3_ugv3_x * scale;
    temp_point.y = form3_ugv3_y * scale;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form3_ugv4_x * scale;
    temp_point.y = form3_ugv4_y * scale;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form3_ugv5_x * scale;
    temp_point.y = form3_ugv5_y * scale;
    formation_vec.emplace_back(temp_point);
    ugv_formation_list["3"] = formation_vec;
    formation_vec.clear();

    temp_point.x = form4_ugv1_x * scale;
    temp_point.y = form4_ugv1_y * scale;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form4_ugv2_x * scale;
    temp_point.y = form4_ugv2_y * scale;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form4_ugv3_x * scale;
    temp_point.y = form4_ugv3_y * scale;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form4_ugv4_x * scale;
    temp_point.y = form4_ugv4_y * scale;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form4_ugv5_x * scale;
    temp_point.y = form4_ugv5_y * scale;
    formation_vec.emplace_back(temp_point);
    ugv_formation_list["4"] = formation_vec;
    formation_vec.clear();

    //formation0
    if(ugv1_on)
    {
        temp_point.x = init_ugv1_x;
        temp_point.y = init_ugv1_y;
        formation_vec.emplace_back(temp_point);
    }
    else
    {
        temp_point.x = 3;
        temp_point.y = 0;
        formation_vec.emplace_back(temp_point);
    }

    if(ugv2_on)
    {
        temp_point.x = init_ugv2_x;
        temp_point.y = init_ugv2_y;
        formation_vec.emplace_back(temp_point);
        d12= double(init_ugv2.pose.position.y); 
        d13=-d12;
        d14=d12*2;
        d15=-d12*2;
    }
    else
    {
        temp_point.x = 3;
        temp_point.y = d12; 
        formation_vec.emplace_back(temp_point);
    }

    if(ugv3_on)
    {
        temp_point.x = init_ugv3_x;
        temp_point.y = init_ugv3_y;
        formation_vec.emplace_back(temp_point);
        d13= double(init_ugv3.pose.position.y); 
        d12=-d13;
        d14=-d13*2;
        d15=d13*2;
    }
    else
    {
        temp_point.x = 3;
        temp_point.y = d13; 
        formation_vec.emplace_back(temp_point);
    }
    
    if(ugv4_on)
    {
        temp_point.x = init_ugv4_x;
        temp_point.y = init_ugv4_y;
        formation_vec.emplace_back(temp_point);
        d14= double(init_ugv4.pose.position.y); 
        d12=d14/2;
        d13=-d14/2;
        d15=-d14;
    }
    else
    {
        temp_point.x = 3;
        temp_point.y = d14; 
        formation_vec.emplace_back(temp_point);
    }
    
    if(ugv5_on)
    {
        temp_point.x = init_ugv5_x;
        temp_point.y = init_ugv5_y;
        formation_vec.emplace_back(temp_point);
        d15= double(init_ugv5.pose.position.y); 
        d12=-d15/2;
        d13=d15/2;
        d14=-d15;
    }
    else
    {   
        temp_point.x = 3;
        temp_point.y = d15; 
        formation_vec.emplace_back(temp_point);
    }
    ugv_formation_list["0"] = formation_vec;
    formation_vec.clear();

    // UAV formation setting
    // formation1
    temp_point.x = form1_uav1_x * scale;
    temp_point.y = form1_uav1_y * scale;
    temp_point.z = form1_uav1_z;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form1_uav2_x * scale;
    temp_point.y = form1_uav2_y * scale;
    temp_point.z = form1_uav2_z;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form1_uav3_x * scale;
    temp_point.y = form1_uav3_y * scale;
    temp_point.z = form1_uav3_z;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form1_uav4_x * scale;
    temp_point.y = form1_uav4_y * scale;
    temp_point.z = form1_uav4_z;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form1_uav5_x * scale;
    temp_point.y = form1_uav5_y * scale;
    temp_point.z = form1_uav5_z;
    formation_vec.emplace_back(temp_point);
    // uav_formation_list.insert(std::pair<std::string,std::vector<geometry_msgs::Point>>("1",formation_vec));
    uav_formation_list["1"] = formation_vec;
    formation_vec.clear();


    // formation2
    temp_point.x = form2_uav1_x * scale;
    temp_point.y = form2_uav1_y * scale;
    temp_point.z = form2_uav1_z;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form2_uav2_x * scale;
    temp_point.y = form2_uav2_y * scale;
    temp_point.z = form2_uav2_z;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form2_uav3_x * scale;
    temp_point.y = form2_uav3_y * scale;
    temp_point.z = form2_uav3_z;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form2_uav4_x * scale;
    temp_point.y = form2_uav4_y * scale;
    temp_point.z = form2_uav4_z;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form2_uav5_x * scale;
    temp_point.y = form2_uav5_y * scale;
    temp_point.z = form2_uav5_z;
    formation_vec.emplace_back(temp_point);
    // uav_formation_list.insert(std::pair<std::string,std::vector<geometry_msgs::Point>>("2",formation_vec));
    uav_formation_list["2"] = formation_vec;
    formation_vec.clear();

    // formation3
    temp_point.x = form3_uav1_x * scale;
    temp_point.y = form3_uav1_y * scale;
    temp_point.z = form3_uav1_z;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form3_uav2_x * scale;
    temp_point.y = form3_uav2_y * scale;
    temp_point.z = form3_uav2_z;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form3_uav3_x * scale;
    temp_point.y = form3_uav3_y * scale;
    temp_point.z = form3_uav3_z;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form3_uav4_x * scale;
    temp_point.y = form3_uav4_y * scale;
    temp_point.z = form3_uav4_z;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form3_uav5_x * scale;
    temp_point.y = form3_uav5_y * scale;
    temp_point.z = form3_uav5_z;
    formation_vec.emplace_back(temp_point);
    // uav_formation_list.insert(std::pair<std::string,std::vector<geometry_msgs::Point>>("3",formation_vec));
    uav_formation_list["3"] = formation_vec;
    formation_vec.clear();
    
    // formation4
    temp_point.x = form4_uav1_x * scale;
    temp_point.y = form4_uav1_y * scale;
    temp_point.z = form4_uav1_z;   
    formation_vec.emplace_back(temp_point);
    temp_point.x = form4_uav2_x * scale;
    temp_point.y = form4_uav2_y * scale;
    temp_point.z = form4_uav2_z;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form4_uav3_x * scale;
    temp_point.y = form4_uav3_y * scale;
    temp_point.z = form4_uav3_z;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form4_uav4_x * scale;
    temp_point.y = form4_uav4_y * scale;
    temp_point.z = form4_uav4_z;
    formation_vec.emplace_back(temp_point);
    temp_point.x = form4_uav5_x * scale;
    temp_point.y = form4_uav5_y * scale;
    temp_point.z = form4_uav5_z;
    formation_vec.emplace_back(temp_point);
    // uav_formation_list.insert(std::pair<std::string,std::vector<geometry_msgs::Point>>("4",formation_vec));
    uav_formation_list["4"] = formation_vec;
    formation_vec.clear();

     // formation0
    if(uav1_on)
    {
        temp_point.x = double(init_uav1.pose.position.x);
        temp_point.y = double(init_uav1.pose.position.y);
        temp_point.z = double(init_uav1.pose.position.z);
        formation_vec.emplace_back(temp_point);
    }
    else
    {
        temp_point.x = 0;
        temp_point.y = 0;
        temp_point.z = 2;
        formation_vec.emplace_back(temp_point);
    }

    if(uav2_on)
    {
        temp_point.x = double(init_uav2.pose.position.x);
        temp_point.y = double(init_uav2.pose.position.y); 
        temp_point.z = double(init_uav2.pose.position.z); 
        formation_vec.emplace_back(temp_point);
        d12= double(init_uav2.pose.position.y); 
        d13=-d12;
        d14=d12*2;
        d15=-d12*2;
    }
    else
    {
        temp_point.x = 0;
        temp_point.y = d12; 
        temp_point.z = 2; 
        formation_vec.emplace_back(temp_point);
    }

    if(uav3_on)
    {
        temp_point.x = double(init_uav3.pose.position.x);
        temp_point.y = double(init_uav3.pose.position.y);
        temp_point.z = double(init_uav3.pose.position.z);
        formation_vec.emplace_back(temp_point);
        d13= double(init_uav3.pose.position.y); 
        d12=-d13;
        d14=-d13*2;
        d15=d13*2;
    }
    else
    {
        temp_point.x = 0;
        temp_point.y = d13; 
        temp_point.z = 2; 
        formation_vec.emplace_back(temp_point);
    }
    
    if(uav4_on)
    {
        temp_point.x = double(init_uav4.pose.position.x);
        temp_point.y = double(init_uav4.pose.position.y);
        temp_point.z = double(init_uav4.pose.position.z);
        formation_vec.emplace_back(temp_point);
        d14= double(init_uav4.pose.position.y); 
        d12=d14/2;
        d13=-d14/2;
        d15=-d14;
    }
    else
    {
        temp_point.x = 0;
        temp_point.y = d14; 
        temp_point.z = 2; 
        formation_vec.emplace_back(temp_point);
    }
    
    if(uav5_on)
    {
        temp_point.x = double(init_uav5.pose.position.x);
        temp_point.y = double(init_uav5.pose.position.y);
        temp_point.z = double(init_uav5.pose.position.z);
        formation_vec.emplace_back(temp_point);
        d15= double(init_uav5.pose.position.y); 
        d12=-d15/2;
        d13=d15/2;
        d14=-d15;
    }
    else
    {   
        temp_point.x = 0;
        temp_point.y = d15; 
        temp_point.z = 2; 
        formation_vec.emplace_back(temp_point);
    }
    // uav_formation_list.insert(std::pair<std::string,std::vector<geometry_msgs::Point>>("0",formation_vec));
    uav_formation_list["0"] = formation_vec;
    formation_vec.clear();
}


void formation_order::reset()
{
    if(uav_formation_input==1 && move==0)
    {
        if(formation_change_end && move==0)
        {
            move=1;
            set_key_input_in = true;
            set_fir_key_input_in = true;

            uav_key_input_in = true;
            uav_fir_key_input_in = true;

            formation_order_end = true;
            formation_set_end = true;
            formation_sampling = false;
            formation_changing = true;

            uav_changing_point_id_max = uav_changing_point_id_max;

            uav_changing_point_id =0;
            ugv_changing_point_id =0;

            uav_assignment.clear();

            uav_change_formation_vec.clear();
        }
        if(formation_changing)
        {
            ++ loop_count;
        }

        if(formation_changing && (uav_key_input_in || all_key_input_in))
        {
            uav_formation_msg.poses.clear();
        }
    }
    else
    {
        move=1;
        if(formation_change_end || formation_change_end_ugv )
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

            formation_order_end_ugv = false;
            formation_set_end_ugv = false;
            formation_sampling_ugv = false;
            formation_changing_ugv = false;
            formation_change_end_ugv = false;

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

        if(formation_changing || formation_changing_ugv )
        {
            ++ loop_count;
        }

        if(formation_changing && (uav_key_input_in || all_key_input_in))
        {
            uav_formation_msg.poses.clear();
        }
        if(formation_changing_ugv && (ugv_key_input_in || all_key_input_in))
        {
            ugv_formation_msg.poses.clear();
        }
        
    }

}

void formation_order::publish()
{
    if(formation_changing || formation_changing_ugv )
    {
        uav_formation_pub.publish(uav_formation_msg);
        ugv_formation_pub.publish(ugv_formation_msg);
    }
    f_num.data=uav_formation_input;
    formation_num_pub.publish(f_num);
    
}

void formation_order::msg_set()
{
    if(formation_changing || formation_changing_ugv )
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
    }
    if(formation_change_end_ugv )
    {
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
        else if(all_key_input_in)
        {
            if(uav_changing_point_id_max == uav_changing_point_id )
            {
                formation_change_end = true;
            }
            else
            {
                formation_change_end = false;
            }
            ++ uav_changing_point_id;
        }
    }

    if(formation_changing_ugv && formation_order_end_ugv)
    {
        if(ugv_key_input_in || all_key_input_in && move==1)
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
            std::cout << BLUE"Changing UGV formation [" << ugv_changing_point_id << "]/[" << ugv_changing_point_id_max << "] ..."END << "\n";
        }
        std::cout << std::flush;

        if(ugv_key_input_in)
        {
            if(ugv_changing_point_id_max == ugv_changing_point_id)
            {
                formation_change_end_ugv = true;
            }
            else
            {
                formation_change_end_ugv = false;
            }
            ++ ugv_changing_point_id;
        }
        else if(all_key_input_in)
        {
            if(ugv_changing_point_id_max == ugv_changing_point_id)
            {
                formation_change_end_ugv = true;
            }
            else
            {
                formation_change_end_ugv = false;
            }
            ++ ugv_changing_point_id;
        }
    }
}

void formation_order::formation_sampling_point()
{
    if (!formation_sampling && formation_order_end)
    {
        std::cout << GREEN "UAV Formation change root sampling ... " END;
        std::string cur_uav_pattern = std::to_string(uav_formation_input);

        uav_assignment.clear();

        for (int id = 0; id < uav_prev_formation.size(); ++id) {
            uav_assignment.push_back(id); 
        }

        if (uav_key_input_in || all_key_input_in)
        {
            if(uav_formation_input==1)
            {     
                geometry_msgs::Point sampling_node1;
                geometry_msgs::Point sampling_node2;  
                for (int id = 0; id < uav_prev_formation.size(); ++id)
                {
                    std::cout <<uav_prev_formation.size()<<std::endl;
                    std::vector<geometry_msgs::Point> uav_change_point_vec;
                    double distance = sqrt(pow((uav_prev_formation[id].x - uav_formation_list[cur_uav_pattern][uav_assignment[id]].x), 2) +
                                           pow((uav_prev_formation[id].y - uav_formation_list[cur_uav_pattern][uav_assignment[id]].y), 2) +
                                           pow((uav_prev_formation[id].z - uav_formation_list[cur_uav_pattern][uav_assignment[id]].z), 2));
                    int sampling_node_num = (int)std::ceil(distance / (set_vel * ((1 / rate))));
                    if(sampling_node_num==0)
                    {
                        sampling_node_num=1;
                    }

                    if(move==0)
                    {
                        for (int j = 0; j <sampling_node_num; ++j)
                        {
                            sampling_node1.x = uav_prev_formation[id].x + ((uav_formation_list[cur_uav_pattern][uav_assignment[id]].x - uav_prev_formation[id].x) / (double)sampling_node_num) * j;
                            sampling_node1.y = uav_prev_formation[id].y;
                            sampling_node1.z = uav_formation_list["1"][id].z ;
                            uav_change_point_vec.emplace_back(sampling_node1);
                        }
                        uav_change_point_vec.emplace_back(sampling_node1);
                        uav_change_formation_vec.emplace_back(uav_change_point_vec);
                    }
                    else if(move==1)
                    {
                        for (int j = 0; j < sampling_node_num; ++j)
                        {
                            sampling_node2.x = uav_formation_list[cur_uav_pattern][uav_assignment[id]].x ;
                            sampling_node2.y = uav_prev_formation[id].y + (( uav_formation_list[cur_uav_pattern][uav_assignment[id]].y - uav_prev_formation[id].y) / (double)sampling_node_num) * j;
                            sampling_node2.z = uav_formation_list[cur_uav_pattern][uav_assignment[id]].z ;
                            uav_change_point_vec.emplace_back(sampling_node2);
                        }
                        uav_change_point_vec.emplace_back(uav_formation_list[cur_uav_pattern][uav_assignment[id]]);
                        uav_change_formation_vec.emplace_back(uav_change_point_vec);
                    }
                }  
            }
            else
            {
                move=0;
                for (int id = 0; id < uav_prev_formation.size(); ++id)
                    {
                        std::cout <<uav_prev_formation.size()<<std::endl;

                        std::vector<geometry_msgs::Point> uav_change_point_vec;

                        double distance = sqrt(pow((uav_prev_formation[id].x - uav_formation_list[cur_uav_pattern][uav_assignment[id]].x), 2) +
                                               pow((uav_prev_formation[id].y - uav_formation_list[cur_uav_pattern][uav_assignment[id]].y), 2) +
                                               pow((uav_prev_formation[id].z - uav_formation_list[cur_uav_pattern][uav_assignment[id]].z), 2));

                        int sampling_node_num = (int)std::ceil(distance / (set_vel * ((1 / rate))));

                        for (int j = 0; j < sampling_node_num; ++j)
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
            }
            
            
            for (const auto form_vec : uav_change_formation_vec)
            {
                if (uav_changing_point_id_max < form_vec.size())
                {
                    uav_changing_point_id_max = form_vec.size();
                }
            }
        }

       
        if (!uav_change_formation_vec.empty() || !ugv_change_formation_vec.empty())
        {
            formation_changing = true;
            formation_sampling = true;
            if (all_key_input_in)
            {
                if (ugv_changing_point_id_max > uav_changing_point_id_max)
                {
                    uav_changing_point_id_max = ugv_changing_point_id_max;
                }
                if (ugv_changing_point_id_max < uav_changing_point_id_max)
                {
                    ugv_changing_point_id_max = uav_changing_point_id_max;
                }
            }
            std::cout << GREEN "END" END << "\n";
            std::cout << std::flush;
        }
    }

    if (!formation_sampling_ugv && formation_order_end_ugv)
    {
        std::cout << BLUE "UGV Formation change root sampling ... " END;
        std::string cur_ugv_pattern = std::to_string(ugv_formation_input);

        ugv_assignment.clear();

        for (int id = 0; id < ugv_prev_formation.size(); ++id) {
            ugv_assignment.push_back(id); 
        }

        if (ugv_key_input_in || all_key_input_in)
        {
            for (int id = 0; id < ugv_prev_formation.size(); ++id)
            {
                std::vector<geometry_msgs::Point> ugv_change_point_vec;

                double distance = sqrt(pow((ugv_prev_formation[id].x - ugv_formation_list[cur_ugv_pattern][ugv_assignment[id]].x), 2) +
                                       pow((ugv_prev_formation[id].y - ugv_formation_list[cur_ugv_pattern][ugv_assignment[id]].y), 2));

                int sampling_node_num = (int)std::ceil(distance / (set_vel * (1 / rate)));

                for (int j = 0; j < sampling_node_num; ++j)
                {
                    geometry_msgs::Point sampling_node;
                    sampling_node.x = ugv_prev_formation[id].x + ((ugv_formation_list[cur_ugv_pattern][ugv_assignment[id]].x - ugv_prev_formation[id].x) / (double)sampling_node_num) * j;
                    sampling_node.y = ugv_prev_formation[id].y + ((ugv_formation_list[cur_ugv_pattern][ugv_assignment[id]].y - ugv_prev_formation[id].y) / (double)sampling_node_num) * j;
                    ugv_change_point_vec.emplace_back(sampling_node);
                }
                ugv_change_point_vec.emplace_back(ugv_formation_list[cur_ugv_pattern][ugv_assignment[id]]);
                ugv_change_formation_vec.emplace_back(ugv_change_point_vec);
            }

            for (const auto form_vec : ugv_change_formation_vec)
            {
                if (ugv_changing_point_id_max < form_vec.size())
                {
                    ugv_changing_point_id_max = form_vec.size();
                }
            } 
        }
        if (!ugv_change_formation_vec.empty())
        {
            formation_changing_ugv = true;
            formation_sampling_ugv = true;
            if (all_key_input_in)
            {
                if (ugv_changing_point_id_max > uav_changing_point_id_max)
                {
                    uav_changing_point_id_max = ugv_changing_point_id_max;
                }
                if (ugv_changing_point_id_max < uav_changing_point_id_max)
                {
                    ugv_changing_point_id_max = uav_changing_point_id_max;
                }
            }
            std::cout << BLUE "END" END << "\n";
            std::cout << std::flush;
        }
    }
}


void formation_order::formation_pattern()
{
    if(!formation_set_end && formation_order_end)
    {
        std::cout << "Caculating UAV formation cost ... ";
        if(loop_count == 0)
        {
            uav_prev_formation = uav_formation_list["0"];
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
        }
        std::string prev_uav_pattern = std::to_string(uav_formation_pattern);

        std::string cur_uav_pattern = std::to_string(uav_formation_input);
    }
    if(!formation_set_end_ugv && formation_order_end_ugv)
    {
        std::cout << "Caculating UGV formation cost ... ";
        if(loop_count == 0)
        {
            ugv_prev_formation = ugv_formation_list["0"];
        }
        else
        {
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
            case 4:
                ugv_prev_formation = ugv_formation_list["4"];
                break;
            default:
                break;
            }
        }
        std::string prev_ugv_pattern = std::to_string(ugv_formation_pattern);

        std::string cur_ugv_pattern = std::to_string(ugv_formation_input);
    }
}

void formation_order::input_set()
{
    if(formation_order_end || formation_order_end_ugv)
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
        std::cout << "[2] : Formation new order only " << BLUE"UGV"END << "\n";
        std::cout << "[3] : Formation new order " << YELL"UGV && UAV"END << "\n";
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
            std::cout << BLUE"Choose UGV foramtion change"END << "\n";
        }
        else if(choose_set_input == 3)
        {
            std::cout << YELL"Choose UAV && UGV foramtion change"END << "\n";
        }
        set_key_input_in = true;
    }

    if(set_key_input_in)
    {
        std::cout << "==============================================" << "\n";
        if(choose_set_input == 1 && !uav_fir_key_input_in)
        {
            std::cout << CYAN"Please push the button [1 ~ 4]"END << "\n";
            std::cout << "┌[1.]────────┬[2.]────────┬[3.]────────┬[4.]─────────┐" << "\n";
            std::cout << "│     o      │     o      │            │             │" << "\n";
            std::cout << "│     o      │            │  o     o   │      o      │" << "\n";
            std::cout << "│     o      │ o       o  │     o      │   o     o   │" << "\n";
            std::cout << "│     o      │            │  o     o   │ o         o │" << "\n";
            std::cout << "│     o      │  o     o   │            │             │" << "\n";
            std::cout << "└────────────┴────────────┴────────────┴─────────────┘" << "\n";
            std::cout << "  [narrow]    [besiege]     [escort]       [scout]    " << "\n";
            std::cout << "If you want exit code Please push [c]" << "\n";
            std::cout << std::flush;
            uav_formation_input = key_in();
            uav_formation_input -= 48;
            uav_fir_key_input_in = true;
            move=0;
        }
        else if(choose_set_input == 2 && !ugv_fir_key_input_in)
        {
            std::cout << CYAN"Please push the button [1 ~ 4]"END << "\n";
            std::cout << "┌[1.]────────┬[2.]────────┬[3.]────────┬[4.]─────────┐" << "\n";
            std::cout << "│     x      │  x     x   │     x      │             │" << "\n";
            std::cout << "│     x      │            │  x     x   │      x      │" << "\n";
            std::cout << "│     x      │ x       x  │            │   x     x   │" << "\n";
            std::cout << "│     x      │            │  x     x   │ x         x │" << "\n";
            std::cout << "│     x      │     x      │            │             │" << "\n";
            std::cout << "└────────────┴────────────┴────────────┴─────────────┘" << "\n";
            std::cout << "  [narrow]    [besiege]     [escort]       [scout]    " << "\n";
            std::cout << "If you want exit code Please push [c]" << "\n";
            std::cout << std::flush;
            ugv_formation_input = key_in();
            ugv_formation_input -= 48;
            ugv_fir_key_input_in = true;
            move=1;
        }
        else if(choose_set_input == 3 && !all_fir_key_input_in)
        {
            std::cout << CYAN"Please push the button [1 ~ 4]"END << "\n";
            std::cout << "┌[1.]────────┬[2.]────────┬[3.]────────┬[4.]─────────┐" << "\n";
            std::cout << "│     o      │ x   o   x  │     x      │      o      │" << "\n";
            std::cout << "│     x      │            │x o     o x │   o  x  o   │" << "\n";
            std::cout << "│     o      │ o       o  │     o      │o  x     x  o│" << "\n";
            std::cout << "│     x      │ x       x  │x o     o x │ x         x │" << "\n";
            std::cout << "│     o      │  o  x  o   │            │             │" << "\n";
            std::cout << "└────────────┴────────────┴────────────┴─────────────┘" << "\n";
            std::cout << "  [narrow]    [besiege]     [escort]       [scout]    " << "\n";
            std::cout << "If you want exit code Please push [c]" << "\n";
            std::cout << std::flush;
            all_formation_input = key_in();
            all_formation_input -= 48;
            if(all_formation_input==1)
            {
                move=0;
            }
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
            std::cout << RED"[ERROR] Please push the button [1 ~ 4]"END << "\n";
            ugv_formation_input = key_in();
            ugv_formation_input -= 48;
            ugv_fir_key_input_in = true;
        }
        else if(all_formation_input != 51 && all_formation_input < 1 && all_formation_input > 3 && all_fir_key_input_in)
        {
            std::cout << "==============================================" << "\n";
            std::cout << "\n";
            std::cout << RED"[ERROR] Please push the button [1 ~ 4]"END << "\n";
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
    else if(set_key_input_in && ugv_formation_input >= 1 && ugv_formation_input <= 4)
    {
        std::cout << BLUE"Formation UGV order end ... && Formation change start !!! "END << "\n";
        formation_order_end_ugv = true;
        ugv_key_input_in = true;
    }
    else if(set_key_input_in && all_formation_input >= 1 && all_formation_input <= 4)
    {
        std::cout << YELL"Formation UAV && UGV order end ... && Formation change start !!! "END << "\n";
        formation_order_end = true;
        all_key_input_in = true;
        formation_order_end_ugv = true;
    }
    std::cout << std::flush;
}

void formation_order::param_read()
{
    nh.getParam("/scale",scale);
    nh.getParam("/scale_nor",scale_nor);
    nh.getParam("/set_vel",set_vel);
    nh.getParam("/rate",rate);
    nh.getParam("/uav_z_max",uav_z_max);
    nh.getParam("/frame_id",frame_id);

    nh.getParam("/uav1_on",uav1_on);
    nh.getParam("/uav2_on",uav2_on);
    nh.getParam("/uav3_on",uav3_on);
    nh.getParam("/uav4_on",uav4_on);
    nh.getParam("/uav5_on",uav5_on);

    nh.getParam("/ugv1_on",ugv1_on);
    nh.getParam("/ugv2_on",ugv2_on);
    nh.getParam("/ugv3_on",ugv3_on);
    nh.getParam("/ugv4_on",ugv4_on);
    nh.getParam("/ugv5_on",ugv5_on);

    //initial position
    nh.getParam("UGV/rbt1/x", init_ugv1_x);
    nh.getParam("UGV/rbt1/y", init_ugv1_y);
    nh.getParam("UGV/rbt2/x", init_ugv2_x);
    nh.getParam("UGV/rbt2/y", init_ugv2_y);
    nh.getParam("UGV/rbt3/x", init_ugv3_x);
    nh.getParam("UGV/rbt3/y", init_ugv3_y);
    nh.getParam("UGV/rbt4/x", init_ugv4_x);
    nh.getParam("UGV/rbt4/y", init_ugv4_y);
    nh.getParam("UGV/rbt5/x", init_ugv5_x);
    nh.getParam("UGV/rbt5/y", init_ugv5_y);

    //formation
    nh.getParam("UGV/formation1/rbt1/x", form1_ugv1_x);
    nh.getParam("UGV/formation1/rbt1/y", form1_ugv1_y);
    nh.getParam("UGV/formation1/rbt2/x", form1_ugv2_x);
    nh.getParam("UGV/formation1/rbt2/y", form1_ugv2_y);
    nh.getParam("UGV/formation1/rbt3/x", form1_ugv3_x);
    nh.getParam("UGV/formation1/rbt3/y", form1_ugv3_y);
    nh.getParam("UGV/formation1/rbt4/x", form1_ugv4_x);
    nh.getParam("UGV/formation1/rbt4/y", form1_ugv4_y);
    nh.getParam("UGV/formation1/rbt5/x", form1_ugv5_x);
    nh.getParam("UGV/formation1/rbt5/y", form1_ugv5_y);
    
    nh.getParam("UGV/formation2/rbt1/x", form2_ugv1_x);
    nh.getParam("UGV/formation2/rbt1/y", form2_ugv1_y);
    nh.getParam("UGV/formation2/rbt2/x", form2_ugv2_x);
    nh.getParam("UGV/formation2/rbt2/y", form2_ugv2_y);
    nh.getParam("UGV/formation2/rbt3/x", form2_ugv3_x);
    nh.getParam("UGV/formation2/rbt3/y", form2_ugv3_y);
    nh.getParam("UGV/formation2/rbt4/x", form2_ugv4_x);
    nh.getParam("UGV/formation2/rbt4/y", form2_ugv4_y);
    nh.getParam("UGV/formation2/rbt5/x", form2_ugv5_x);
    nh.getParam("UGV/formation2/rbt5/y", form2_ugv5_y);

    nh.getParam("UGV/formation3/rbt1/x", form3_ugv1_x);
    nh.getParam("UGV/formation3/rbt1/y", form3_ugv1_y);
    nh.getParam("UGV/formation3/rbt2/x", form3_ugv2_x);
    nh.getParam("UGV/formation3/rbt2/y", form3_ugv2_y);
    nh.getParam("UGV/formation3/rbt3/x", form3_ugv3_x);
    nh.getParam("UGV/formation3/rbt3/y", form3_ugv3_y);
    nh.getParam("UGV/formation3/rbt4/x", form3_ugv4_x);
    nh.getParam("UGV/formation3/rbt4/y", form3_ugv4_y);
    nh.getParam("UGV/formation3/rbt5/x", form3_ugv5_x);
    nh.getParam("UGV/formation3/rbt5/y", form3_ugv5_y);

    nh.getParam("UGV/formation4/rbt1/x", form4_ugv1_x);
    nh.getParam("UGV/formation4/rbt1/y", form4_ugv1_y);
    nh.getParam("UGV/formation4/rbt2/x", form4_ugv2_x);
    nh.getParam("UGV/formation4/rbt2/y", form4_ugv2_y);
    nh.getParam("UGV/formation4/rbt3/x", form4_ugv3_x);
    nh.getParam("UGV/formation4/rbt3/y", form4_ugv3_y);
    nh.getParam("UGV/formation4/rbt4/x", form4_ugv4_x);
    nh.getParam("UGV/formation4/rbt4/y", form4_ugv4_y);
    nh.getParam("UGV/formation4/rbt5/x", form4_ugv5_x);
    nh.getParam("UGV/formation4/rbt5/y", form4_ugv5_y);

    nh.getParam("UAV/formation1/rbt1/x", form1_uav1_x);
    nh.getParam("UAV/formation1/rbt1/y", form1_uav1_y);
    nh.getParam("UAV/formation1/rbt1/z", form1_uav1_z);
    nh.getParam("UAV/formation1/rbt2/x", form1_uav2_x);
    nh.getParam("UAV/formation1/rbt2/y", form1_uav2_y);
    nh.getParam("UAV/formation1/rbt2/z", form1_uav2_z);
    nh.getParam("UAV/formation1/rbt3/x", form1_uav3_x);
    nh.getParam("UAV/formation1/rbt3/y", form1_uav3_y);
    nh.getParam("UAV/formation1/rbt3/z", form1_uav3_z);
    nh.getParam("UAV/formation1/rbt4/x", form1_uav4_x);
    nh.getParam("UAV/formation1/rbt4/y", form1_uav4_y);
    nh.getParam("UAV/formation1/rbt4/z", form1_uav4_z);
    nh.getParam("UAV/formation1/rbt5/x", form1_uav5_x);
    nh.getParam("UAV/formation1/rbt5/y", form1_uav5_y);
    nh.getParam("UAV/formation1/rbt5/z", form1_uav5_z);

    nh.getParam("UAV/formation2/rbt1/x", form2_uav1_x);
    nh.getParam("UAV/formation2/rbt1/y", form2_uav1_y);
    nh.getParam("UAV/formation2/rbt1/z", form2_uav1_z);
    nh.getParam("UAV/formation2/rbt2/x", form2_uav2_x);
    nh.getParam("UAV/formation2/rbt2/y", form2_uav2_y);
    nh.getParam("UAV/formation2/rbt2/z", form2_uav2_z);
    nh.getParam("UAV/formation2/rbt3/x", form2_uav3_x);
    nh.getParam("UAV/formation2/rbt3/y", form2_uav3_y);
    nh.getParam("UAV/formation2/rbt3/z", form2_uav3_z);
    nh.getParam("UAV/formation2/rbt4/x", form2_uav4_x);
    nh.getParam("UAV/formation2/rbt4/y", form2_uav4_y);
    nh.getParam("UAV/formation2/rbt4/z", form2_uav4_z);
    nh.getParam("UAV/formation2/rbt5/x", form2_uav5_x);
    nh.getParam("UAV/formation2/rbt5/y", form2_uav5_y);
    nh.getParam("UAV/formation2/rbt5/z", form2_uav5_z);

    nh.getParam("UAV/formation3/rbt1/x", form3_uav1_x);
    nh.getParam("UAV/formation3/rbt1/y", form3_uav1_y);
    nh.getParam("UAV/formation3/rbt1/z", form3_uav1_z);
    nh.getParam("UAV/formation3/rbt2/x", form3_uav2_x);
    nh.getParam("UAV/formation3/rbt2/y", form3_uav2_y);
    nh.getParam("UAV/formation3/rbt2/z", form3_uav2_z);
    nh.getParam("UAV/formation3/rbt3/x", form3_uav3_x);
    nh.getParam("UAV/formation3/rbt3/y", form3_uav3_y);
    nh.getParam("UAV/formation3/rbt3/z", form3_uav3_z);
    nh.getParam("UAV/formation3/rbt4/x", form3_uav4_x);
    nh.getParam("UAV/formation3/rbt4/y", form3_uav4_y);
    nh.getParam("UAV/formation3/rbt4/z", form3_uav4_z);
    nh.getParam("UAV/formation3/rbt5/x", form3_uav5_x);
    nh.getParam("UAV/formation3/rbt5/y", form3_uav5_y);
    nh.getParam("UAV/formation3/rbt5/z", form3_uav5_z);

    nh.getParam("UAV/formation4/rbt1/x", form4_uav1_x);
    nh.getParam("UAV/formation4/rbt1/y", form4_uav1_y);
    nh.getParam("UAV/formation4/rbt1/z", form4_uav1_z);
    nh.getParam("UAV/formation4/rbt2/x", form4_uav2_x);
    nh.getParam("UAV/formation4/rbt2/y", form4_uav2_y);
    nh.getParam("UAV/formation4/rbt2/z", form4_uav2_z);
    nh.getParam("UAV/formation4/rbt3/x", form4_uav3_x);
    nh.getParam("UAV/formation4/rbt3/y", form4_uav3_y);
    nh.getParam("UAV/formation4/rbt3/z", form4_uav3_z);
    nh.getParam("UAV/formation4/rbt4/x", form4_uav4_x);
    nh.getParam("UAV/formation4/rbt4/y", form4_uav4_y);
    nh.getParam("UAV/formation4/rbt4/z", form4_uav4_z);
    nh.getParam("UAV/formation4/rbt5/x", form4_uav5_x);
    nh.getParam("UAV/formation4/rbt5/y", form4_uav5_y);
    nh.getParam("UAV/formation4/rbt5/z", form4_uav5_z); 
    
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

    formation_num_pub = nh.advertise<std_msgs::Int64>("/formation_num",1);

    UAV1_init_pose_sub = nh.subscribe("UAV1/initial_pose", 1, &formation_order::UAV1_init_pose_Callback, this);
    UAV2_init_pose_sub = nh.subscribe("UAV2/initial_pose", 1, &formation_order::UAV2_init_pose_Callback, this);
    UAV3_init_pose_sub = nh.subscribe("UAV3/initial_pose", 1, &formation_order::UAV3_init_pose_Callback, this);
    UAV4_init_pose_sub = nh.subscribe("UAV4/initial_pose", 1, &formation_order::UAV4_init_pose_Callback, this);
    UAV5_init_pose_sub = nh.subscribe("UAV5/initial_pose", 1, &formation_order::UAV5_init_pose_Callback, this);

    UGV1_init_pose_sub = nh.subscribe("UGV1/initial_pose", 1, &formation_order::UGV1_init_pose_Callback, this);
    UGV2_init_pose_sub = nh.subscribe("UGV2/initial_pose", 1, &formation_order::UGV2_init_pose_Callback, this);
    UGV3_init_pose_sub = nh.subscribe("UGV3/initial_pose", 1, &formation_order::UGV3_init_pose_Callback, this);
    UGV4_init_pose_sub = nh.subscribe("UGV4/initial_pose", 1, &formation_order::UGV4_init_pose_Callback, this);
    UGV5_init_pose_sub = nh.subscribe("UGV5/initial_pose", 1, &formation_order::UGV5_init_pose_Callback, this);
}

void formation_order::run()
{
    param_read();
    
    ros::Rate r(rate);
    while(ros::ok())
    {
        ros::spinOnce();
        
        if(wait_time<=100)
        {
            formation_init_set();
            std::cout<<"waiting initial setting for (" <<double(wait_time)/10<<"/10) sec..."<<std::endl;
            wait_time+=1;  
        }
        else if(wait_time>100)
        {
            if(!formation_change_end && !formation_changing || !formation_change_end_ugv && !formation_changing_ugv )
            {
                key_read();
            }
            input_set();
            formation_pattern();
            formation_sampling_point();
            formation_changing_point();
            formation_reset();
            msg_set();
            publish();
            reset();
        }
        
        r.sleep();
    }
    
}
