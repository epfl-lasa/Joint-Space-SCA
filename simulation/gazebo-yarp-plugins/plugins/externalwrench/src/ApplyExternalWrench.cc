/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia. RBCS Department.
 * Author: Jorhabib Eljaik
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "ApplyExternalWrench.hh"

namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN ( ApplyExternalWrench )

ApplyExternalWrench::ApplyExternalWrench()
{
    this->m_wrenchToApply.force.resize ( 3,0 );
    this->m_wrenchToApply.torque.resize ( 3,0 );
    this->m_wrenchToApply.duration = 0.0;
    timeIni = 0;
    m_newCommand = false;
}
ApplyExternalWrench::~ApplyExternalWrench()
{
    m_rpcThread.stop();
    this->m_updateConnection.reset();
}

void ApplyExternalWrench::UpdateChild()
{
    // Reading apply wrench command
    yarp::os::Bottle tmpBottle;

    // Copying command
    this->m_lock.lock();
    tmpBottle = this->m_rpcThread.getCmd();
    if ( tmpBottle.get( 8 ).asInt32() == 1 )
    {
        // If this is a new command
        m_newCommand = true;
        this->m_rpcThread.setNewCommandFlag( 0 );
    } else {
        m_newCommand = false;
    }
    this->m_lock.unlock();

    // initialCmdBottle will contain the initial bottle in RPCServerThread
    static yarp::os::Bottle prevCmdBottle = tmpBottle;

    // Parsing command
    this->m_linkName = tmpBottle.get ( 0 ).asString();
    for ( int i=1; i<4; i++ ) {
        m_wrenchToApply.force[i-1] = tmpBottle.get ( i ).asDouble();
    }
    for ( int i=4; i<7; i++ ) {
        m_wrenchToApply.torque[i-4] = tmpBottle.get ( i ).asDouble();
    }

    std::string fullScopeLinkName = "";
    if(this->m_subscope!="") {
      fullScopeLinkName = std::string ( this->m_modelScope + "::" + this->m_subscope + "::" + this->m_linkName );
      this->m_onLink  = m_myModel->GetLink ( fullScopeLinkName );
    } else {
      this->m_onLink  = m_myModel->GetLink ( this->m_linkName );
    }

//  This piece of code shows the full name (scoped name + link name) of every link in the current loaded model.
//     gazebo::physics::Link_V tmpLinksVector;
//     tmpLinksVector = m_myModel->GetLinks();
//     for (int i=0; i<tmpLinksVector.size(); i++)
//     yDebug() << tmpLinksVector[i]->GetName() << " ";
//     yDebug() << std::endl;

    if ( !this->m_onLink ) {
        //yError() << "ApplyWrench plugin: link named " << this->m_linkName<< " not found";
        return;
    }

    // Get wrench duration
    this->m_wrenchToApply.duration = tmpBottle.get ( 7 ).asDouble();


    // Taking duration of the applied force into account
    static bool applying_force_flag = 0;

    double time_current = yarp::os::Time::now();

    if ( m_newCommand )
    {
        timeIni = yarp::os::Time::now();
        applying_force_flag = 1;
    }

    // This has to be done during the specified duration
    if ( applying_force_flag && ( time_current - timeIni ) < m_wrenchToApply.duration ) {
#if GAZEBO_MAJOR_VERSION >= 8
        // Copying command to force and torque Vector3 variables
        ignition::math::Vector3d force ( this->m_wrenchToApply.force[0], this->m_wrenchToApply.force[1], this->m_wrenchToApply.force[2] );
        ignition::math::Vector3d torque ( this->m_wrenchToApply.torque[0], this->m_wrenchToApply.torque[1], this->m_wrenchToApply.torque[2] );
        this->m_onLink->AddForce ( force );
        this->m_onLink->AddTorque ( torque );
        ignition::math::Vector3d linkCoGPos = this->m_onLink->WorldCoGPose().Pos(); // Get link's COG position where wrench will be applied
        ignition::math::Vector3d newZ = force.Normalize(); // normalized force. I want the z axis of the cylinder's reference frame to coincide with my force vector
        ignition::math::Vector3d newX = newZ.Cross ( ignition::math::Vector3d::UnitZ );
        ignition::math::Vector3d newY = newZ.Cross ( newX );
        ignition::math::Matrix4d rotation = ignition::math::Matrix4d ( newX[0],newY[0],newZ[0],0,newX[1],newY[1],newZ[1],0,newX[2],newY[2],newZ[2],0, 0, 0, 0, 1 );
        ignition::math::Quaterniond forceOrientation = rotation.Rotation();
        ignition::math::Pose3d linkCoGPose ( linkCoGPos - rotation*ignition::math::Vector3d ( 0,0,.15 ), forceOrientation );
#else
        // Copying command to force and torque Vector3 variables
        math::Vector3 force ( this->m_wrenchToApply.force[0], this->m_wrenchToApply.force[1], this->m_wrenchToApply.force[2] );
        math::Vector3 torque ( this->m_wrenchToApply.torque[0], this->m_wrenchToApply.torque[1], this->m_wrenchToApply.torque[2] );
        this->m_onLink->AddForce ( force );
        this->m_onLink->AddTorque ( torque );
        math::Vector3 linkCoGPos = this->m_onLink->GetWorldCoGPose().pos; // Get link's COG position where wrench will be applied
        math::Vector3 newZ = force.Normalize(); // normalized force. I want the z axis of the cylinder's reference frame to coincide with my force vector
        math::Vector3 newX = newZ.Cross ( math::Vector3::UnitZ );
        math::Vector3 newY = newZ.Cross ( newX );
        math::Matrix4 rotation = math::Matrix4 ( newX[0],newY[0],newZ[0],0,newX[1],newY[1],newZ[1],0,newX[2],newY[2],newZ[2],0, 0, 0, 0, 1 );
        math::Quaternion forceOrientation = rotation.GetRotation();
        math::Pose linkCoGPose ( linkCoGPos - rotation*math::Vector3 ( 0,0,.15 ), forceOrientation );
#endif

#if GAZEBO_MAJOR_VERSION == 7
        msgs::Set ( m_visualMsg.mutable_pose(), linkCoGPose.Ign() );
#else
        msgs::Set ( m_visualMsg.mutable_pose(), linkCoGPose );
#endif
        double red = 1;
        double alpha = 0.3;
#if GAZEBO_MAJOR_VERSION >= 9
        msgs::Set(m_visualMsg.mutable_material()->mutable_ambient(), ignition::math::Color(red,0,0,alpha));
#else
        msgs::Set(m_visualMsg.mutable_material()->mutable_ambient(), common::Color(red,0,0,alpha));
#endif
        m_visualMsg.set_visible ( 0 );
        m_visPub->Publish ( m_visualMsg );
    }

    if ( applying_force_flag && ( time_current - timeIni ) > m_wrenchToApply.duration ) {
        applying_force_flag = 0;
        m_visualMsg.set_visible ( 0 );
        m_visPub->Publish ( m_visualMsg );
    }

}

void ApplyExternalWrench::Load ( physics::ModelPtr _model, sdf::ElementPtr _sdf )
{
    // Check if yarp network is active;
    if ( !this->m_yarpNet.checkNetwork() ) {
        yError ( "ERROR Yarp Network was not found active in ApplyExternalWrench plugin" );
        return;
    }

    // What is the parent name??
    this->m_modelScope = _model->GetScopedName();

    // Copy the pointer to the model to access later from UpdateChild
    this->m_myModel = _model;

    bool configuration_loaded = false;

    // Read robot name
    if ( _sdf->HasElement ( "robotNamefromConfigFile" ) ) {
        std::string iniRobotName      = _sdf->Get<std::string> ( "robotNamefromConfigFile" );
        std::string iniRobotNamePath =  gazebo::common::SystemPaths::Instance()->FindFileURI ( iniRobotName );

        if ( iniRobotNamePath != "" && this->m_iniParams.fromConfigFile ( iniRobotNamePath.c_str() ) ) {
            yarp::os::Value robotNameParam = m_iniParams.find ( "gazeboYarpPluginsRobotName" );
            this->robotName = robotNameParam.asString();
            m_rpcThread.setRobotName  ( robotName );
            m_rpcThread.setScopedName ( this->m_modelScope );
            gazebo::physics::Link_V links = _model->GetLinks();
            std::string defaultLink = links[0]->GetName();
            m_rpcThread.setDefaultLink(defaultLink);
	    this->m_subscope = retrieveSubscope(links, m_modelScope);
            configuration_loaded = true;
        } else {
            yError ( "ERROR trying to get robot configuration file" );
            return;
        }
    } else {
            this->robotName = _model->GetName();
            m_rpcThread.setRobotName ( robotName );
            m_rpcThread.setScopedName ( this->m_modelScope );
            gazebo::physics::Link_V links = _model->GetLinks();
            m_rpcThread.setDefaultLink(links.at(0)->GetName());
            configuration_loaded = true;
    }

    // Starting RPC thread to read desired wrench to be applied
    if ( !m_rpcThread.start() ) {
        yError ( "ERROR: rpcThread did not start correctly" );
    }


    /// ############ TRYING TO MODIFY VISUALS #######################################################


    this->m_node = transport::NodePtr ( new gazebo::transport::Node() );

#if GAZEBO_MAJOR_VERSION >= 8
    std::string worldName = _model->GetWorld()->Name();
#else
    std::string worldName = _model->GetWorld()->GetName();
#endif

    this->m_node->Init ( worldName );
    m_visPub = this->m_node->Advertise<msgs::Visual> ( "~/visual", 10 );

    // Set the visual's name. This should be unique.
    m_visualMsg.set_name ( "__CYLINDER_VISUAL__" );

    // Set the visual's parent. This visual will be attached to the parent
    m_visualMsg.set_parent_name ( _model->GetScopedName() );

    // Create a cylinder
    msgs::Geometry *geomMsg = m_visualMsg.mutable_geometry();
    geomMsg->set_type ( msgs::Geometry::CYLINDER );
    geomMsg->mutable_cylinder()->set_radius ( 0.01 );
    geomMsg->mutable_cylinder()->set_length ( .30 );

    // Don't cast shadows
    m_visualMsg.set_cast_shadows ( false );

    /// ############ END: TRYING TO MODIFY VISUALS #####################################################


    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->m_updateConnection = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &ApplyExternalWrench::UpdateChild, this ) );
}

std::string ApplyExternalWrench::retrieveSubscope ( gazebo::physics::Link_V& v , std::string scope)
{
    std::string tmpName = v[0]->GetName();
    std::size_t found = tmpName.find_first_of(":");
    if(found!=std::string::npos)
      tmpName = tmpName.substr (0, found);
    else
      tmpName = "";
    return tmpName;
}


}


// ############ RPCServerThread class ###############

void RPCServerThread::setRobotName ( std::string robotName )
{
    this->m_robotName = robotName;
}

void RPCServerThread::setScopedName ( std::string scopedName )
{
    this->m_scopedName = scopedName;
}

void RPCServerThread::setDurationBuffer ( double d )
{
    this->m_durationBuffer = d;
}

double RPCServerThread::getDurationBuffer()
{
    return this->m_durationBuffer;
}

void RPCServerThread::setDefaultLink(const std::string &defaultLink)
{
    this->m_defaultLink = defaultLink;
}

bool RPCServerThread::threadInit()
{

    if ( !m_rpcPort.open ( std::string ( "/"+m_robotName + "/applyExternalWrench/rpc:i" ).c_str() ) ) {
        yError ( "ERROR opening RPC port /applyExternalWrench" );
        return false;
    }
    // Default link on which wrenches are applied
    //m_cmd.addString ( this->m_scopedName + "::l_arm" );
    m_cmd.addString ( this->m_defaultLink );
    m_cmd.addFloat64 ( 0 ); // Force  coord. x
    m_cmd.addFloat64 ( 0 ); // Force  coord. y
    m_cmd.addFloat64 ( 0 ); // Force  coord. z
    m_cmd.addFloat64 ( 0 ); // Torque coord. x
    m_cmd.addFloat64 ( 0 ); // Torque coord. y
    m_cmd.addFloat64 ( 0 ); // Torque coord. z
    m_cmd.addFloat64 ( 0 ); // Wrench duration

    this->m_durationBuffer = m_cmd.get ( 7 ).asDouble();

    return true;
}
void RPCServerThread::run()
{
    while ( !isStopping() ) {
        yarp::os::Bottle command;
        m_rpcPort.read ( command,true );
        if ( command.get ( 0 ).asString() == "help" ) {
            this->m_reply.addVocab ( yarp::os::Vocab::encode ( "many" ) );
            this->m_reply.addString ( "Insert a command with the following format:" );
            this->m_reply.addString ( "[link] [force] [torque] [duration]" );
            this->m_reply.addString ( "e.g. chest 10 0 0 0 0 0 1");
            this->m_reply.addString ( "[link]:     (string) Link ID of the robot as specified in robot's SDF" );
            this->m_reply.addString ( "[force]:    (double x, y, z) Force components in N w.r.t. world reference frame" );
            this->m_reply.addString ( "[torque]:   (double x, y, z) Torque components in N.m w.r.t world reference frame" );
            this->m_reply.addString ( "[duration]: (double) Duration of the applied force in seconds" );
            this->m_reply.addString ( "Note: The reference frame is the base/root robot frame with x pointing backwards and z upwards.");
            this->m_rpcPort.reply ( this->m_reply );
        } else {
            if ( command.get ( 0 ).isString() \
                    && ( command.get ( 1 ).isDouble() || command.get ( 1 ).isInt() )  && ( command.get ( 2 ).isDouble() || command.get ( 2 ).isInt() ) && ( command.get ( 3 ).isDouble() || command.get ( 3 ).isInt() ) \
                    && ( command.get ( 4 ).isDouble() || command.get ( 4 ).isInt() )  && ( command.get ( 5 ).isDouble() || command.get ( 5 ).isInt() ) && ( command.get ( 6 ).isDouble() || command.get ( 6 ).isInt() )  \
                    && ( command.get ( 7 ).isDouble() || command.get ( 7 ).isInt() ) ) {
                this->m_reply.addString ( "[ACK] Correct command format" );
                this->m_rpcPort.reply ( m_reply );
                m_lock.lock();
                // new-command flag
                command.addInt32(1);
                m_cmd = command;
                m_lock.unlock();
            } else {
                this->m_reply.clear();
                this->m_reply.addString ( "ERROR: Incorrect command format" );
                this->m_rpcPort.reply ( this->m_reply );
            }
        }
        m_reply.clear();
        command.clear();
    }
}
void RPCServerThread::threadRelease()
{
    yarp::os::Thread::threadRelease();
    m_rpcPort.close();
}
yarp::os::Bottle RPCServerThread::getCmd()
{
    return m_cmd;
}

void RPCServerThread::setNewCommandFlag(std::int32_t flag)
{
    m_cmd.get( 8 ) = yarp::os::Value(flag);
}

void RPCServerThread::onStop()
{
    m_rpcPort.interrupt();
}
