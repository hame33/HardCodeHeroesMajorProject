#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <iostream>
#include <math.h>
using namespace std;
#define step_line 0.3 //单位m,每隔0.3m设置一个导航点
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef geometry_msgs::PointStamped Point;
move_base_msgs::MoveBaseGoal goal[100];//导航点数组

//函数输入为线段的起始点和终点，以及前面已设置的导航点个数，输出为加上该直线当前已设置的导航点个数
int common_line(move_base_msgs::MoveBaseGoal origin, move_base_msgs::MoveBaseGoal end, int begin){
    int num = ceil(sqrt(pow(origin.target_pose.pose.position.x-end.target_pose.pose.position.x,2)+pow(origin.target_pose.pose.position.y-end.target_pose.pose.position.y,2))/step_line);//通过线段长度计算所需要的导航点个数，实际个数为num加一
    goal[begin] = origin;
    goal[num+begin] = end;
    int i = begin + 1;
    double yaw;
    geometry_msgs::Quaternion q;
    double x1 = origin.target_pose.pose.position.x;
    double y1 = origin.target_pose.pose.position.y;
    double x2 = end.target_pose.pose.position.x;
    double y2 = end.target_pose.pose.position.y;//提取起始点和终点的横纵坐标
    //根据线段在二维坐标轴中的具体位置，计算线段上的点的坐标以及导航需要的朝向，间距为0.3m，并将这些坐标放入goal数组
    if(x1==x2){
        if(y1>y2){
            yaw = -M_PI/2;
            q = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw);
            goal[begin].target_pose.pose.orientation = q;
            goal[num+begin].target_pose.pose.orientation = q;
            for(; i < num+begin; i++){
                goal[i].target_pose.pose.position.x = goal[i-1].target_pose.pose.position.x;
                goal[i].target_pose.pose.position.y = goal[i-1].target_pose.pose.position.y - step_line;
                goal[i].target_pose.pose.orientation = q;   
            }   
        }else{
            yaw = M_PI/2;
            q = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw);
            goal[begin].target_pose.pose.orientation = q;
            goal[num+begin].target_pose.pose.orientation = q;
            for(; i < num+begin; i++){
                goal[i].target_pose.pose.position.x = goal[i-1].target_pose.pose.position.x;
                goal[i].target_pose.pose.position.y = goal[i-1].target_pose.pose.position.y + step_line;
                goal[i].target_pose.pose.orientation = q;   
            }   
        }
    }else if(y1==y2){
        if(x1>x2){
            yaw = M_PI;
            q = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw);
            goal[begin].target_pose.pose.orientation = q;
            goal[num+begin].target_pose.pose.orientation = q;
            for(; i < num+begin; i++){
                goal[i].target_pose.pose.position.x = goal[i-1].target_pose.pose.position.x - step_line;
                goal[i].target_pose.pose.position.y = goal[i-1].target_pose.pose.position.y;
                goal[i].target_pose.pose.orientation = q;   
            }   
        }else{
            yaw = 0;
            q = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw);
            goal[begin].target_pose.pose.orientation = q;
            goal[num+begin].target_pose.pose.orientation = q;
            for(; i < num+begin; i++){
                goal[i].target_pose.pose.position.x = goal[i-1].target_pose.pose.position.x + step_line;
                goal[i].target_pose.pose.position.y = goal[i-1].target_pose.pose.position.y;
                goal[i].target_pose.pose.orientation = q;   
            }   
        }
    }else if((y2-y1)/(x2-x1)>0){
        double k = (y2-y1)/(x2-x1);
        if(x1>x2){
            yaw = -(M_PI - atan(k));
            q = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw);
            goal[begin].target_pose.pose.orientation = q;
            goal[num+begin].target_pose.pose.orientation = q;
            for(; i < num+begin; i++){
                goal[i].target_pose.pose.position.x = goal[i-1].target_pose.pose.position.x - step_line*cos(atan(k));
                goal[i].target_pose.pose.position.y = goal[i-1].target_pose.pose.position.y - step_line*sin(atan(k));
                goal[i].target_pose.pose.orientation = q;   
            }   
        }else{
            yaw = atan(k);
            q = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw);
            goal[begin].target_pose.pose.orientation = q;
            goal[num+begin].target_pose.pose.orientation = q;
            for(; i < num+begin; i++){
                goal[i].target_pose.pose.position.x = goal[i-1].target_pose.pose.position.x + step_line*cos(atan(k));
                goal[i].target_pose.pose.position.y = goal[i-1].target_pose.pose.position.y + step_line*sin(atan(k));
                goal[i].target_pose.pose.orientation = q;   
            }   
        }
    }else{
        double k = -(y2-y1)/(x2-x1);
        if(x1>x2){
            yaw = (M_PI - atan(k));
            q = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw);
            goal[begin].target_pose.pose.orientation = q;
            goal[num+begin].target_pose.pose.orientation = q;
            for(; i < num+begin; i++){
                goal[i].target_pose.pose.position.x = goal[i-1].target_pose.pose.position.x - step_line*cos(atan(k));
                goal[i].target_pose.pose.position.y = goal[i-1].target_pose.pose.position.y + step_line*sin(atan(k));
                goal[i].target_pose.pose.orientation = q;   
            }   
        }else{
            yaw = -atan(k);
            q = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw);
            goal[begin].target_pose.pose.orientation = q;
            goal[num+begin].target_pose.pose.orientation = q;
            for(; i < num+begin; i++){
                goal[i].target_pose.pose.position.x = goal[i-1].target_pose.pose.position.x + step_line*cos(atan(k));
                goal[i].target_pose.pose.position.y = goal[i-1].target_pose.pose.position.y - step_line*sin(atan(k));
                goal[i].target_pose.pose.orientation = q;   
            }   
        }
    }

    return num+1+begin;//返回已经保存的导航点的数量
}


int main(int argc, char** argv){
    ros::init(argc, argv, "send_goals_node");
    MoveBaseClient ac("move_base", true);
    //目标：小车从任意位置自主导航到a点，再从a点沿直线ab走到b点，再沿直线bc走到c点，再沿直线cd走到d点
    move_base_msgs::MoveBaseGoal a,b,c,d;
   
    a.target_pose.pose.position.x = 2;
    a.target_pose.pose.position.y = 0;
    
    b.target_pose.pose.position.x = 1;
    b.target_pose.pose.position.y = 0.5;

    c.target_pose.pose.position.x = 0;
    c.target_pose.pose.position.y = 0;

    d.target_pose.pose.position.x = 2;
    d.target_pose.pose.position.y = 0;//给出abcd四点坐标
	
	//首先common_line(a,b,0)将ab间的导航点写入goal数组，其返回值为当前导航点数，其可以作为common_line(b,c,num)的第三个参数即goal中已经设置好的导航点的个数，依次类推，得到总导航点数goal_number，并且各导航点也写入了goal
    int goal_number = common_line(c,d,common_line(b,c,common_line(a,b,0)));//将导航点写入goal数组
   
    while(!ac.waitForServer( ros::Duration( 5.0 ) )){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    ROS_INFO(" Init success!!! ");
    int i = 0;
    //按顺序发送导航点，成功后再发送下一个导航点，直到发送完所有导航点
    while(i < goal_number)
    {
        goal[i].target_pose.header.frame_id = "map";
        goal[i].target_pose.header.stamp = ros::Time::now();
        ac.sendGoal(goal[i]);
        ROS_INFO("Send NO. %d Goal !!!", i);
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("The NO. %d Goal achieved success !!!", i);
            i++ ;
        }else{ROS_WARN("The NO. %d Goal Planning Failed for some reason", i); }
    }
    return 0;
} 