#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo
{
  class AutoGrabPlugin : public WorldPlugin
  {
    private: 
      physics::WorldPtr world;
      physics::ModelPtr robot;
      physics::ModelPtr block; // Bloco ArUco
      physics::LinkPtr garra1_link; // Link da garra
      event::ConnectionPtr updateConnection;
      double grabDistance;
      bool holdingBlock = false;

    public:
      AutoGrabPlugin() : WorldPlugin()
      {
        this->grabDistance = 0.05; // Distância de ativação
      }

      void Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/)
      {
        this->world = _world;

        // Obter referência ao robô e ao bloco ArUco
        this->robot = this->world->ModelByName("robot_description");
        this->block = this->world->ModelByName("aruco_model_1"); // Nome do bloco ArUco

        // Obter referência ao link da garra
        this->garra1_link = this->robot->GetLink("garra1_link");

        // Conectar ao evento de atualização
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&AutoGrabPlugin::OnUpdate, this));
      }

      void OnUpdate()
      {
        if (this->holdingBlock) return;

        // Obter posição da garra
        ignition::math::Vector3d garraPos = this->garra1_link->WorldPose().Pos();
        // Obter posição do bloco ArUco
        ignition::math::Vector3d blockPos = this->block->GetLink("link")->WorldPose().Pos();

        // Calcular distância entre a garra e o bloco ArUco
        double distance = garraPos.Distance(blockPos);

        gzmsg << "Garra do robô detectada. Distância: " << distance << std::endl;


        // Se a distância for menor que a distância de captura, mover o bloco
        if (distance < this->grabDistance && !this->holdingBlock)
        {

        gzmsg << "Bloco detectado dentro da distância de captura. Capturando bloco..." << std::endl;

          // Transportar o bloco para a posição da garra
          ignition::math::Pose3d newBlockPose(
            garraPos.X(), garraPos.Y(), garraPos.Z(), 0, 0, 0);
          this->block->SetWorldPose(newBlockPose);

          // Criar uma joint fixa entre o bloco e a garra
          auto fixedJoint = this->world->Physics()->CreateJoint("fixed", this->robot);
          fixedJoint->Load(this->garra1_link, this->block->GetLink("link"), ignition::math::Pose3d());
          fixedJoint->Init();

          this->holdingBlock = true;
        }
      }

      // Função para soltar o bloco
      void ReleaseBlock()
      {
        if (this->holdingBlock)
        {
          this->holdingBlock = false;
        }
      }
  };

  // Registrar o plugin no Gazebo
  GZ_REGISTER_WORLD_PLUGIN(AutoGrabPlugin)
}
