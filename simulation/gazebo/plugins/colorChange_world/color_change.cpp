#include "color_change.h"
using namespace gazebo;
using namespace std;


void changeColor::Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/)
{

    for(int i=0;i<5;i++)
	    cout << "Plugin says helloooo!\n" << endl;

    this->world = _world;
    this->model = this->world->ModelByName("iCub");
    cout << this->model << endl;
    this->gzNode = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gzNode->Init(this->world->Name());
    this->pub_visual = this->gzNode->Advertise<gazebo::msgs::Visual>("~/visual");
    //this->updateConnection = event::Events::ConnectPreRender(boost::bind(&changeColor::OnUpdate, this));
    this->sub_collisions = this->gzNode->Subscribe("~/collisions/", &changeColor::on_msg, this);
    this->links_list = this->model->GetLinks();
    cout << links_list.size() << endl;
    for(int i=0;i<this->links_list.size();i++)
    {
        this->was_collided.push_back(0);
    }
    cout << this->was_collided.size() << "init_done" << endl;
}

void changeColor::on_msg(ConstGzString_VPtr &_msg)
{
    vector<string> collided_vec;
    for(int i=0;i<_msg->data_size();i++)
    {
        collided_vec.push_back(_msg->data(i));
        //cout << "c: " << collided_vec[i] << endl;
    }

    for(int i=0;i<this->links_list.size();i++)
    {

        gazebo::physics::LinkPtr link = this->links_list[i];
        string i_name(link->GetName());
        bool flag = false;
        for(int j=0;j<collided_vec.size();j++){
            //cout <<i<<" "<<j<<" "<< i_name.c_str() << " " << collided_vec[j].c_str() << " " << strcmp(i_name.c_str(),collided_vec[j].c_str()) << endl;
            if(strcmp(i_name.c_str(),collided_vec[j].c_str())==0)
            {
                flag = true;
                break;
            }
        }
        msgs::Visual visMsg = link->GetVisualMessage("visual");
        //msgs::Material *materialMsg = visMsg.mutable_material();
        visMsg.set_name(link->GetScopedName());
        visMsg.set_parent_name(this->model->GetScopedName());
        //materialMsg->clear_ambient();
        //materialMsg->clear_diffuse();
        if(flag)
        {
            cout << i_name << endl;
            //gazebo::common::Color newColor(1.0,0.0,0.0,0.1);
            //msgs::Color *homColMsg = new gazebo::msgs::Color(gazebo::msgs::Convert(newColor));
            //msgs::Color *homDiffMsg = new gazebo::msgs::Color(*homColMsg);
            //materialMsg->set_allocated_ambient(homColMsg);
            //materialMsg->set_allocated_diffuse(homDiffMsg);
            msgs::Set(visMsg.mutable_material()->mutable_ambient(),ignition::math::Color::Red);
            msgs::Set(visMsg.mutable_material()->mutable_diffuse(),ignition::math::Color::Red);
            visMsg.set_transparency(0.5);
            this->was_collided[i] = 1;
            //visMsg.clear_material();
            //visMsg.set_allocated_material(&this->red_vis[i]);            
        }
        else
        {   
            visMsg.set_transparency(0);
            if(this->was_collided[i] == 1)
            {
                cout << "returning the color!" << endl;
                ignition::math::Color amb(0.0, 0.0, 0.0, 1.0);
                ignition::math::Color dif(0.4224001, 0.5056, 0.64, 1.0);
                msgs::Set(visMsg.mutable_material()->mutable_ambient(),amb);
                msgs::Set(visMsg.mutable_material()->mutable_diffuse(),dif);
                this->was_collided[i] = 0;
            }
        }
        //cout << materialMsg << endl;
        this->pub_visual->Publish(visMsg);
    }
    // if(_msg->data_size()>1)
    // {
    //     for(int i=0;i<_msg->data_size();i++)
    //     {
    //         cout << "collided " << _msg->data(i) << " " << endl;
    //     }
    // }

}


// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(changeColor)
