#include "joystick.hpp"

#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <stdio.h>

//All the transport communication goes through the Node
gazebo::transport::NodePtr node(new gazebo::transport::Node());
//One node can have multiple Publishers Subscribers
gazebo::transport::PublisherPtr pub;
gazebo::transport::SubscriberPtr  sub_alive;

std::string topic;

bool  is_parter_alive;
bool  is_restart_required;

gazebo::common::Time mtime;

void publish_Joystick_Torque(JAxis &getup_axis, gazebo::transport::PublisherPtr pub)
{
  if(getup_axis.isUpdated())
  {
    double Target = ((getup_axis.getValue()+1)/2);// 0->1

    gazebo::msgs::Int val;
    int vi = (Target*1000);
    val.set_data((::google::protobuf::int32) vi);
    std::cout << "Torque sent : " << vi << std::endl;
    pub->Publish(val);

  }
}

void alive_callback(ConstAnyPtr &msg)
{
  // Dump the message contents to stdout.
  std::cout << "Received> " << msg->string_value() << std::endl;

  is_parter_alive = true;
  mtime = gazebo::common::Time::GetWallTime();
}

void cleanup()
{

  unsigned int cbid = sub_alive->GetCallbackId();
  node->RemoveCallback(topic,cbid);
  std::cout << "Removing callback id " << cbid << std::endl;
  sub_alive->Unsubscribe();
  node->Fini();
  node.reset(new gazebo::transport::Node());//reset the pointer -- very important to allow new subscriptions
  gazebo::client::shutdown();
  std::cout << "gazebo client shutdown " << std::endl;
}

void startSequence(int _argc, char **_argv)
{


  //this waits for a connection with a gazebo server
  std::cout << "* waiting for a gazebo server" << std::endl;
  gazebo::client::setup(_argc, _argv);        //starts also the transportation
  std::cout << "* gazebo client setup" << std::endl;
  

  //Leave some time for the server to start the plugin that will advertise this topic
  //gazebo::common::Time::MSleep(1000);
  //Advertise is the only way that allow others to find this topic
  topic = "/gazebo/servos/ax12a/0";

  node->Init();
  std::cout << "* node initialised ID : " << node->GetId() << std::endl;
  sub_alive = node->Subscribe(topic,alive_callback);
  std::cout << "* Subscribed to "<< sub_alive->GetTopic() <<" callback ID : " << sub_alive->GetCallbackId() << std::endl;


  std::string torque_ref_topic = topic + "/torque_ref";
  pub = node->Advertise<gazebo::msgs::Int>(torque_ref_topic);
  std::cout << "* Topic Advertised: "<< torque_ref_topic << std::endl;

}

int main(int _argc, char **_argv)
{
  
  is_parter_alive = false;
  is_restart_required = true;

  mtime = gazebo::common::Time::GetWallTime();
  std::cout << "servo client started at : " << mtime.FormattedString() << std::endl;
  
  Joystick 	joy;
  joy.start("/dev/input/js0");
  std::cout << "* Joystick device started" << std::endl;
  
  while(true)
  {
    //TODO detect the event when the server is shutdown to restart the client
    if(is_parter_alive)
    {
      if(joy.update())
      {
        publish_Joystick_Torque(joy.getAxis(5), pub);
      }
      gazebo::common::Time::MSleep(400);
    }
    else
    {
      if(is_restart_required)
      {
        startSequence(_argc, _argv);
        is_restart_required = false;//only required when the target dies
      }
      else
      {
        gazebo::common::Time::MSleep(1000);
        std::cout << "." << std::endl;
      }
    }
    //check if still alive
    gazebo::common::Time ntime = gazebo::common::Time::GetWallTime();
    double diff = ntime.Double() - mtime.Double();
    if(diff > 4)
    {
      if(is_parter_alive)//transition from live to death
      {
        is_restart_required = true;
        is_parter_alive = false;
        std::cout << "parter not responding at : " << ntime.FormattedString() << std::endl;
        
        cleanup();
      }
    }
  }

  

}
