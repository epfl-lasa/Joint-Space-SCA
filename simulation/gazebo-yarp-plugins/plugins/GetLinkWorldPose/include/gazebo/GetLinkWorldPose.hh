#ifndef GETLINKWORLDPOSE_HH
#define GETLINKWORLDPOSE_HH

#include <iostream>
#include <string>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>

#include <yarp/os/Network.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Vocab.h>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>

class StreamServerThread: public yarp::os::Thread
{
    public:
        virtual bool        threadInit();
        virtual void        run();
        virtual void        threadRelease();

        yarp::os::Bottle    getCmd();   // to be changed to getLinkName
        void                setRobotName(std::string robotName);
        void                setScopedName(std::string scopedName);
        void                setDefaultLink(const std::string& defaultLink);
        void                setNewCommandFlag(int flag);
        virtual void        onStop();

        yarp::sig::Vector pose_value;

    private:

        // Creation of port for the CoM
        yarp::os::BufferedPort<yarp::sig::Vector> m_StreamPort;
        yarp::os::Bottle    m_cmd;
        /// \brief Mutex to lock reading and writing of _cmd
        boost::mutex        m_lock;
        std::string         m_robotName;
        std::string         m_scopedName;
        std::string         m_defaultLink;
     
};


namespace gazebo
{
    class GetLinkWorldPose : public ModelPlugin
    {

        public:
            GetLinkWorldPose();
            virtual ~GetLinkWorldPose();

            std::string retrieveSubscope(gazebo::physics::Link_V& v, std::string  scope);           
            std::string          robotName;
            double               timeIni;           

        protected:

            // Inherited
            void Load ( physics::ModelPtr _model, sdf::ElementPtr _sdf );
            // Inherited
            virtual void UpdateChild();
            

        private:
            yarp::os::Network       m_yarpNet;

            StreamServerThread      m_StreamThread;

            yarp::os::Property      m_iniParams;

            physics::ModelPtr       m_myModel;
            /// \brief Link for which the pose will be streamed
            std::string             m_modelScope;
            std::string             m_subscope;
            std::string             m_linkName;
            /// \brief Link the plugin is attached to
            physics::LinkPtr        m_onLink;
            /// \brief Mutex to lock access
            boost::mutex            m_lock;

            /// \brief Pointer to the update event connection
            event::ConnectionPtr    m_updateConnection;
              
    };

}



#endif  //GETLINKWORLDPOSE_HH
