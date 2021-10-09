#include <obstacle_avoidance/goal_publisher.h>

GoalPublisher::GoalPublisher()
{
    // initialise variables
    // 変数の初期化
    count = 0;
    padding = 5;
    got_map = false;
}

void GoalPublisher::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    // save map to use when generating goals
    // ゴールを選ぶ時に使えるよう、マップを格納
    got_map = true;
    map = *msg;
}

void GoalPublisher::statusCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
    // if the robot reached its last goal, send a new one
    // ロボットがゴールに到達したら、次のゴールを選ぶ
    if (msg->status.text == "Goal reached.") {
        ROS_INFO("move_base reported goal reached");
        publishRandomGoal();
    }
}

void GoalPublisher::publishRandomGoal()
{
    // check whether we received a map yet
    // マップの存在を確認
    if (got_map == false) {
        ROS_WARN("No map received yet, not publishing goal");
        return;
    }

    // declare goal message and populate header
    // ゴールのメッセージを宣言、header に情報を入れる
    geometry_msgs::PoseStamped goal;
    goal.header.seq = count;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map";

    // initialise variables for finding a valid goal
    // ロボットが行けるゴールを探す時に使われる変数の初期化
    bool valid_goal = false;
    float x = 0.0;
    float y = 0.0;
    int try_count = 1;

    while (valid_goal == false && try_count < 1000) {
        // choose random grid cell within map bounds
        // マップの範囲内から、ランダムにマスを選ぶ
        int grid_x = floor(map.info.width*(float(rand())/float(RAND_MAX)));
        int grid_y = floor(map.info.height*(float(rand())/float(RAND_MAX)));
        int index = map.info.width*grid_y + grid_x;

        // check whether chosen cell is occupied or unknown
        // 選んだマスが空いているか確認
        if (map.data[index] != -1 && map.data[index] <= 1) {
            // check surrounding cells to leave space for robot
            // 周りのマスも確認し、ロボットが行けるかどうか判断
            bool blocked = false;
            for (int i = -padding; i <= padding; i++) {
                for (int j = -padding; j <= padding; j++) {
                    index = map.info.width*(grid_y+j) + (grid_x+i);
                    if (map.data[index] > 1 || map.data[index] == -1) {
                        // mark if any of the cells are occupied
                        // 周りのマスが空いていなかった場合、blockedをtrueに設定
                        blocked = true;
                    }
                }
            }

            if (blocked == false) {
                // valid cell chosen, convert cell to map coords
                // ロボットが行けるマスを選んだ場合、マスのx,yをマップの座標に変換
                valid_goal = true;
                x = map.info.origin.position.x + grid_x*map.info.resolution;
                y = map.info.origin.position.y + grid_y*map.info.resolution;
                ROS_INFO("Publishing new goal: (%f,%f)", x, y);
            }
        }
        // keep track of attempts to avoid infinite loops
        // 無限ループにならないよう、失敗した回数を格納
        try_count++;
    }
    
    // give up and skip if no valid cells were found
    // ロボットが行けるマスが見つからない場合、エラーを出す
    if (valid_goal == false) {
        ROS_ERROR("No valid cells found, is the map filled out?");
        return;
    }

    // generate random yaw for goal and convert to quaternion
    // ランダムに角度を選び、quaternionに変換
    float yaw = 6.28*(float(rand())/float(RAND_MAX)-0.5);
    tf2::Quaternion q;
    q.setRPY(0,0,yaw);

    // set goal position and orientation
    // ゴールの座標と角度を設定
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0;
    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();
    goal.pose.orientation.w = q.w();

    pub.publish(goal);
    count++;
}

int main(int argc, char **argv)
{
    // initialise ros and node
    // ROSとNodeHandleの初期化
    ros::init(argc, argv, "goal_publisher");
    ros::NodeHandle nh("~");
    GoalPublisher goal_pub;

    // set up map subscriber, goal publisher
    // マップとロボットのステータスのsubscriberと、ゴールpublisherの初期化
    goal_pub.pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    goal_pub.map_sub = nh.subscribe("/map", 10, &GoalPublisher::mapCallback, &goal_pub);
    goal_pub.status_sub = nh.subscribe("/move_base/result", 10, &GoalPublisher::statusCallback, &goal_pub);

    // load padding parameter from launch file
    // launchファイルからpadding(ゴールの周りのマスをどこまで確認するか)を読み込む
    int padding_param;
    if (nh.getParam("padding", padding_param)) {
        ROS_INFO("Loaded padding value %d from launch file", padding_param);
        goal_pub.padding = padding_param;
    }
    else {
        ROS_ERROR("Failed to load padding parameter");
    }

    // wait for messages
    // マップとステータスのメッセージを待つ
    ros::spin();
    return 0;
}