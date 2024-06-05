#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <yaml-cpp/yaml.h>

namespace gazebo
{
    class RobotDrivePlugin : public ModelPlugin
    {
        public:
        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;
        physics::JointPtr jointFL, jointFR, jointRL, jointRR;
        std::unique_ptr<ros::NodeHandle> nh;
        ros::Subscriber cmd_vel_subscriber;

        // Parâmetros carregados do arquivo YAML
        double wheel_separation_width, wheel_separation_length, wheel_radius;

        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
            }

            this->model = _model;
            nh.reset(new ros::NodeHandle("gazebo_client"));

            // Assine o tópico cmd_vel
            cmd_vel_subscriber = nh->subscribe("/cmd_vel", 10, &RobotDrivePlugin::OnCmdVelReceived, this);

            jointFL = _model->GetJoint("fl_wheel_joint");
            jointFR = _model->GetJoint("fr_wheel_joint");
            jointRL = _model->GetJoint("bl_wheel_joint");
            jointRR = _model->GetJoint("br_wheel_joint");

            updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&RobotDrivePlugin::OnUpdate, this));

            // Carregar configurações do arquivo YAML
            YAML::Node config = YAML::LoadFile("/path/to/config.yaml");
            wheel_separation_width = config["wheel_separation_width"].as<double>();
            wheel_separation_length = config["wheel_separation_length"].as<double>();
            wheel_radius = config["wheel_radius"].as<double>();
        }

        void OnCmdVelReceived(const geometry_msgs::Twist::ConstPtr& msg)
        {
            physics::LinkPtr base_link = model->GetLink("base_link");  // Substitua "base_link" pelo nome correto da sua base
            if (base_link)
            {
                base_link->SetLinearVel(ignition::math::Vector3d(msg->linear.x, msg->linear.y, 0));
                base_link->SetAngularVel(ignition::math::Vector3d(0, 0, msg->angular.z));

                double vx = msg->linear.x;
                double vy = msg->linear.y;
                double omega = msg->angular.z;

                double front_left = -1 * (vx - vy - omega * (wheel_separation_width / 2 + wheel_separation_length / 2)) / wheel_radius;
                double front_right = (vx + vy + omega * (wheel_separation_width / 2 + wheel_separation_length / 2)) / wheel_radius;
                double rear_left = -1 * (vx + vy - omega * (wheel_separation_width / 2 + wheel_separation_length / 2)) / wheel_radius;
                double rear_right = (vx - vy + omega * (wheel_separation_width / 2 + wheel_separation_length / 2)) / wheel_radius;

                jointFL->SetVelocity(0, front_left);
                jointFR->SetVelocity(0, front_right);
                jointRL->SetVelocity(0, rear_left);
                jointRR->SetVelocity(0, rear_right);
            }
        }

        void OnUpdate()
        {
            // Este método pode ser usado para publicar odometria ou outros cálculos em cada ciclo de simulação
        }
    };

    GZ_REGISTER_MODEL_PLUGIN(RobotDrivePlugin)
}
