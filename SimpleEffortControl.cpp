#include<tum_ics_ur10_controller_tutorial/SimpleEffortControl.h>

#include<tum_ics_ur_robot_msgs/ControlData.h>


namespace tum_ics_ur_robot_lli{
namespace RobotControllers{

SimpleEffortControl::SimpleEffortControl(double weight, const QString &name):
    ControlEffort(name,SPLINE_TYPE,JOINT_SPACE,weight),
    m_startFlag(false),
    m_Kp(Matrix6d::Zero()),
    m_Kd(Matrix6d::Zero()),
    m_Ki(Matrix6d::Zero()),
    m_goal(Vector6d::Zero()),
    m_totalTime(100.0),
    tdd(0),
    m_mode(false),
    m_DeltaQ(Vector6d::Zero()),
    m_DeltaQ_1(Vector6d::Zero()),
    m_DeltaQp(Vector6d::Zero()),
    m_DeltaQp_1(Vector6d::Zero()),
    m_iDeltaQ(Vector6d::Zero()),
    m_iDeltaQ_1(Vector6d::Zero())
{
    pubCtrlData=n.advertise<tum_ics_ur_robot_msgs::ControlData>("SimpleEffortCtrlData",100);

    m_controlPeriod=0.002; //set the control period to the standard 2 ms

    m_controlPeriod_2=m_controlPeriod/2.0;

    ROS_INFO_STREAM("SimpleEffortCtrl Control Period: "<<m_controlPeriod<<" ("<<m_controlPeriod_2<<")");

}

SimpleEffortControl::~SimpleEffortControl()
{

}

void SimpleEffortControl::setQInit(const JointState& qinit)
{
    m_qInit=qinit;
}
void SimpleEffortControl::setQHome(const JointState& qhome)
{
    m_qHome=qhome;
}
void SimpleEffortControl::setQPark(const JointState& qpark)
{
    m_qPark=qpark;
}


bool SimpleEffortControl::init()
{
    std::string ns="~simple_effort_ctrl";
    std::stringstream s;

    if (!ros::param::has(ns))
    {
        s<<"SimpleEffortControl init(): Control gains not defined --"<<ns<<"--, did you load them in the rosparam server??";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }


    VDouble p;


    /////D GAINS

    s<<ns<<"/gains_d";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF)
    {
        s.str("");
        s<<"SimpleEffortControl init(): Wrong number of d_gains --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF;i++)
    {
        m_Kd(i,i)=p[i];
    }

    ROS_WARN_STREAM("Kd: \n"<<m_Kd);

    /////P GAINS
    s.str("");
    s<<ns<<"/gains_p";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF)
    {
        s.str("");
        s<<"SimpleEffortControl init(): Wrong number of p_gains --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF;i++)
    {
        m_Kp(i,i)=p[i]/m_Kd(i,i)*0.5;
    }

    ROS_WARN_STREAM("Kp: \n"<<m_Kp);


    /////GOAL
    s.str("");
    s<<ns<<"/goal";
    ros::param::get(s.str(),p);

    if(p.size()<STD_DOF)
    {
        s.str("");
        s<<"SimpleEffortControl init(): Wrong number of joint goals --"<<p.size()<<"--";
        m_error=true;
        m_errorString=s.str().c_str();
        return false;
    }
    for(int i=0;i<STD_DOF;i++)
    {
        m_goal(i)=p[i];
    }
    m_totalTime=p[STD_DOF];

    if(!(m_totalTime>0))
    {
        m_totalTime=100.0;
    }

    ROS_WARN_STREAM("Goal [DEG]: \n"<<m_goal.transpose());
    ROS_WARN_STREAM("Total Time [s]: "<<m_totalTime);


    m_goal=DEG2RAD(m_goal);

    ROS_WARN_STREAM("Goal [RAD]: \n"<<m_goal.transpose());


}
bool SimpleEffortControl::start()
{

}
Vector6d SimpleEffortControl::update(const RobotTime &time, const JointState &current)
{
    if(!m_startFlag)
    {
        m_qStart=current.q;
        m_startFlag=true;
    }


    Vector6d tau;

    tau.setZero();


    tum_ics_ur_robot_msgs::ControlData msg;
    msg.header.stamp=ros::Time::now();
    msg.time=time.tD();

    update_J(current.q);
    bool isSingular=ifSingular();//runqiu:if meet singularity, return to park position and wait for user command. Waiting for completion
    ROS_INFO_STREAM("Singular:" << isSingular);

    if(!m_mode) // m_mode=false : joint-space control  //---------- Joint-Space -----------
    {
        VVector6d vQd;

        vQd=getJointPVT5(m_qStart,m_goal,time.tD(),m_totalTime*5);

        m_DeltaQ=current.q-vQd[0];
        m_DeltaQp=current.qp-vQd[1];

        JointState js_r;

        js_r=current;
        js_r.qp=vQd[1]-m_Kp*m_DeltaQ;
        js_r.qpp=vQd[2]-m_Kp*m_DeltaQp;

        Vector6d Sq=current.qp-js_r.qp;


        tau=-m_Kd*Sq;

        for(int i=0;i<STD_DOF;i++)
        {
            msg.q[i]=current.q(i);
            msg.qp[i]=current.qp(i);
            msg.qpp[i]=current.qpp(i);

            msg.qd[i]=vQd[0](i);
            msg.qpd[i]=vQd[1](i);

            msg.Dq[i]=m_DeltaQ(i);
            msg.Dqp[i]=m_DeltaQp(i);

            msg.torques[i]=current.tau(i);
        }        

        ROS_INFO_STREAM("(Joint-Space)\n" << "\n~m_goal: \n" <<m_goal << "\n~current.q: \n" << current.q << "\n~(m_goal-current.q).squaredNorm() \n" << (m_goal-current.q).squaredNorm());

        if( ((m_goal-current.q).squaredNorm()<0.1)  && (!isSingular) )
        {
            m_mode = true;  // m_mode=true : operational-space control

            tau=-m_Kd*Sq;


            g_tau = tau;
            tdd=0;
        }
    }else{  //---------- Operational-Space -----------

        tau = g_tau;
        
        ///update_J(current.q);
        Matrix3d R6_0=getRef(current.q);
        Vector3d angles=R2EulerA(R6_0);
        Vector3d T6_0=getTef(current.q);
        Vector3d cX;
        cX<<T6_0;
        ROS_INFO_STREAM("~~cX \n" <<cX);

        Matrix<double, 6, 3> Jp;
        Jp=getJp(current.q,current.qp);
        //ROS_INFO_STREAM("~~Jp \n" <<Jp);

        Vector3d Xdp;
        Xdp<<0,0,0;
        //ROS_INFO_STREAM("~~Xdp \n" <<Xdp);

        Vector3d Xd;
        Xd << 0.5,0.5,0.5;//-0.17273,0.62781, 0.92249,1.919, 1.412, 1.920;//getXdFromVision();//runqiu:waiting for completion
        ///broadcastarget(Xd);
        //ROS_INFO_STREAM("~~Xd \n" <<Xd);

        Vector3d Xrp;
        Vector3d Xdn;
        Xdn=TrajGen(cX,Xd);
        Xrp=Xdp-m_Kp.topLeftCorner(3,3)*(cX-Xdn);
        ROS_INFO_STREAM("~~Xd, Xdn || " << Xd <<" , "<< Xdn);

        ROS_INFO_STREAM("~~Xrp \n" <<Xrp);

        Matrix3d Jinv;
        Matrix6d Eye_6;
        Eye_6<<1,0,0,0,0,0,0,
               0,1,0,0,0,0,0,
               0,0,1,0,0,0,0,
               0,0,0,1,0,0,0,
               0,0,0,0,1,0,0,
               0,0,0,0,0,1,0,
               0,0,0,0,0,0,1; 

        Matrix3d Eye_3;
        Eye_3<<1,0,0,
               0,1,0,
               0,0,1;

        ///Jinv=m_Jef.inverse();
        Jinv=m_Jef.topRows<3>();
        Jinv=Jinv.transpose()*(Jinv*Jinv.transpose()+0.01*Eye_3).inverse();
        ROS_INFO_STREAM("~~m_Jef \n" <<m_Jef);
        ROS_INFO_STREAM("~~Jinv \n" <<Jinv);
        Vector3d Qrp;
        Qrp=Jinv*Xrp;
        //ROS_INFO_STREAM("~~Qrp" <<Qrp);

        Vector3d Xdpp;
        Xdpp<<0,0,0;

        Vector3d Xrpp;
        Xrpp=Xdpp-m_Kp.topLeftCorner(3,3)*(m_Jef.topRows<3>()*current.qp.topRows<3>()-Xdp);
        //ROS_INFO_STREAM("~~Xrpp" <<Xrpp);

        Vector3d Qrpp;
        Qrpp=Jinv*(Xrpp-Jp.topRows<3>()*Qrp);
        //ROS_INFO_STREAM("~~Qrpp" <<Qrpp);

        Vector3d EoMr;
        //EoMr=getEoMr(current.q,current.qp,Qrp,Qrpp);
        //ROS_INFO_STREAM("~~EoMr1" <<EoMr);
        //EoMr=EoMr+0.1*current.qp;//runqiu:friction
        //ROS_INFO_STREAM("~~EoMr2" <<EoMr);
        Vector3d Sq;
        Sq=current.qp.topRows<3>()-Jinv*Xrp;
        //ROS_INFO_STREAM("~~Sq" <<Sq);

        Vector3d tempTau;
        tempTau=-m_Kd.topLeftCorner(3,3)*0.1*Sq;
        tau.topRows<3>()<<tempTau;

        update_G(current.q);
        if(tdd<100){
          tau=tau+m_G*tanh(tdd/100);
          tdd++;
        }
        ROS_INFO_STREAM("~~m_G \n" <<m_G);
        ROS_INFO_STREAM("~~tau \n" <<tau);

        ROS_INFO_STREAM("(Operational-Space)\n" << "\n~Xd: \n" << Xd << "\n~current.q: \n" << current.q << "\n~(Xd-current.q.topRows<3>()).squaredNorm() \n" << (Xd-current.q.topRows<3>()).squaredNorm());

        if( isSingular )
        {
            m_mode = false;  // m_mode=true : operational-space control
            tdd=0;
        }
    }
    pubCtrlData.publish(msg);

    return tau;

}
bool SimpleEffortControl::stop()
{

}

void SimpleEffortControl::broadcastarget(Vector3d tar_msg){
    br = new tf::TransformBroadcaster();
    tf::Transform transform;

    transform.setOrigin( tf::Vector3( tar_msg(0), tar_msg(1),tar_msg(2) ));
    transform.setRotation( tf::Quaternion(0,0,0,1) );
    br->sendTransform( tf:: StampedTransform(transform, ros::Time::now(),"base_link", "target_point"));
    ROS_INFO_STREAM("TFTFTFTFTF \n");
}

void SimpleEffortControl::update_J(Vector6d currentQ){
  double q1,q2,q3;
  q1=currentQ(0);
  q2=currentQ(1);
  q3=currentQ(2);

  m_Jef << (2581*cos(q1))/10000 + (6127*cos(q2)*sin(q1))/10000 + (sqrt(6802261)*sqrt(20000000)*cos(q3 + atan(1157/5716))*cos(q2)*sin(q1))/20000000 - (sqrt(6802261)*sqrt(20000000)*sin(q3 + atan(1157/5716))*sin(q1)*sin(q2))/20000000, cos(q1)*((6127*sin(q2))/10000 + (sqrt(6802261)*sqrt(20000000)*sin(q2 + q3 + atan(1157/5716)))/20000000), (sqrt(6802261)*sqrt(20000000)*sin(q2 + q3 + atan(1157/5716))*cos(q1))/20000000,
  (2581*sin(q1))/10000 - (6127*cos(q1)*cos(q2))/10000 - (sqrt(6802261)*sqrt(20000000)*cos(q3 + atan(1157/5716))*cos(q1)*cos(q2))/20000000 + (sqrt(6802261)*sqrt(20000000)*sin(q3 + atan(1157/5716))*cos(q1)*sin(q2))/20000000, sin(q1)*((6127*sin(q2))/10000 + (sqrt(6802261)*sqrt(20000000)*sin(q2 + q3 + atan(1157/5716)))/20000000), (sqrt(6802261)*sqrt(20000000)*sin(q2 + q3 + atan(1157/5716))*sin(q1))/20000000,
  0, - (6127*cos(q2))/10000 - (sqrt(6802261)*sqrt(20000000)*cos(q2 + q3 + atan(1157/5716)))/20000000,        -(sqrt(6802261)*sqrt(20000000)*cos(q2 + q3 + atan(1157/5716)))/20000000,
  0, sin(q1), sin(q1),
  0, -cos(q1), -cos(q1),
  1, 0, 0;
}

void SimpleEffortControl::update_G(Vector6d currentQ){
  double q1,q2,q3;
  q1=currentQ(0);
  q2=currentQ(1);
  q3=currentQ(2);

  m_G << 0,
        (580027041*sin(q2 + q3))/200000000 - (7685174601*cos(q2 + q3))/200000000 - (403731531*cos(q2))/5000000,
        (580027041*sin(q2 + q3))/200000000 - (7685174601*cos(q2 + q3))/200000000,
        (580027041*sin(q2 + q3))/200000000,
        -(59019903*sin(q2 + q3))/100000000,
        0;
}

Matrix3d SimpleEffortControl::getRef(Vector6d currentQ){
  Matrix3d R6_0;
  double q1,q2,q3;
  q1=currentQ(0);
  q2=currentQ(1);
  q3=currentQ(2);
  R6_0 << cos(q2 + q3 + atan(1157/5716))*cos(q1), -sin(q2 + q3 + atan(1157/5716))*cos(q1),  sin(q1),
          cos(q2 + q3 + atan(1157/5716))*sin(q1), -sin(q2 + q3 + atan(1157/5716))*sin(q1), -cos(q1),
          sin(q2 + q3 + atan(1157/5716)), cos(q2 + q3 + atan(1157/5716)), 0;
  return R6_0;

}

Vector3d SimpleEffortControl::getTef(Vector6d currentQ){
  Vector3d T3_0;
  double q1,q2,q3;
  q1=currentQ(0);
  q2=currentQ(1);
  q3=currentQ(2);
  T3_0 << (2581*sin(q1))/10000 - (6127*cos(q1)*cos(q2))/10000 - (sqrt(6802261)*sqrt(20000000)*cos(q3 + atan(1157/5716))*cos(q1)*cos(q2))/20000000 + (sqrt(6802261)*sqrt(20000000)*sin(q3 + atan(1157/5716))*cos(q1)*sin(q2))/20000000,
          (sqrt(6802261)*sqrt(20000000)*sin(q3 + atan(1157/5716))*sin(q1)*sin(q2))/20000000 - (6127*cos(q2)*sin(q1))/10000 - (sqrt(6802261)*sqrt(20000000)*cos(q3 + atan(1157/5716))*cos(q2)*sin(q1))/20000000 - (2581*cos(q1))/10000,
          16/125 - (sqrt(6802261)*sqrt(20000000)*sin(q2 + q3 + atan(1157/5716)))/20000000 - (6127*sin(q2))/10000;
  return T3_0;
}

Vector3d SimpleEffortControl::R2EulerA(Matrix3d R){
  Vector3d angles;
  double d;
  d=pow((1-pow(R(3,1),2)),0.5);
  angles(0)=atan2(R(2,1),R(1,1));
  angles(1)=atan2(-R(3,1),d);
  angles(2)=atan2(R(3,2),R(3,3));
  return angles;
}

Matrix<double, 6, 3> SimpleEffortControl::getJp(Vector6d currentQ,Vector6d currentQp){
  Matrix<double, 6, 3> Jp;
  double q1,q2,q3;
  q1=currentQ(0);
  q2=currentQ(1);
  q3=currentQ(2);

  double q1p,q2p,q3p;
  q1p=currentQp(0);
  q2p=currentQp(1);
  q3p=currentQp(2);

  Jp << -q1p*((2581*sin(q1))/10000 - (6127*cos(q1)*cos(q2))/10000 - (sqrt(6802261)*sqrt(20000000)*cos(q3 + atan(1157/5716))*cos(q1)*cos(q2))/20000000 + (sqrt(6802261)*sqrt(20000000)*sin(q3 + atan(1157/5716))*cos(q1)*sin(q2))/20000000) - q2p*sin(q1)*((6127*sin(q2))/10000 + (sqrt(6802261)*sqrt(20000000)*sin(q2 + q3 + atan(1157/5716)))/20000000) - (sqrt(6802261)*sqrt(20000000)*q3p*sin(q2 + q3 + atan(1157/5716))*sin(q1))/20000000, q2p*cos(q1)*((6127*cos(q2))/10000 + (sqrt(6802261)*sqrt(20000000)*cos(q2 + q3 + atan(1157/5716)))/20000000) - q1p*sin(q1)*((6127*sin(q2))/10000 + (sqrt(6802261)*sqrt(20000000)*sin(q2 + q3 + atan(1157/5716)))/20000000) + (sqrt(6802261)*sqrt(20000000)*q3p*cos(q2 + q3 + atan(1157/5716))*cos(q1))/20000000, (sqrt(6802261)*sqrt(20000000)*(q2p*cos(q2 + q3 + atan(1157/5716))*cos(q1) + q3p*cos(q2 + q3 + atan(1157/5716))*cos(q1) - q1p*sin(q2 + q3 + atan(1157/5716))*sin(q1)))/20000000,
      q3p*((sqrt(6802261)*sqrt(20000000)*cos(q3 + atan(1157/5716))*cos(q1)*sin(q2))/20000000 + (sqrt(6802261)*sqrt(20000000)*sin(q3 + atan(1157/5716))*cos(q1)*cos(q2))/20000000) + q2p*((6127*cos(q1)*sin(q2))/10000 + (sqrt(6802261)*sqrt(20000000)*cos(q3 + atan(1157/5716))*cos(q1)*sin(q2))/20000000 + (sqrt(6802261)*sqrt(20000000)*sin(q3 + atan(1157/5716))*cos(q1)*cos(q2))/20000000) + q1p*((2581*cos(q1))/10000 + (6127*cos(q2)*sin(q1))/10000 + (sqrt(6802261)*sqrt(20000000)*cos(q3 + atan(1157/5716))*cos(q2)*sin(q1))/20000000 - (sqrt(6802261)*sqrt(20000000)*sin(q3 + atan(1157/5716))*sin(q1)*sin(q2))/20000000), q2p*sin(q1)*((6127*cos(q2))/10000 + (sqrt(6802261)*sqrt(20000000)*cos(q2 + q3 + atan(1157/5716)))/20000000) + q1p*cos(q1)*((6127*sin(q2))/10000 + (sqrt(6802261)*sqrt(20000000)*sin(q2 + q3 + atan(1157/5716)))/20000000) + (sqrt(6802261)*sqrt(20000000)*q3p*cos(q2 + q3 + atan(1157/5716))*sin(q1))/20000000, (sqrt(6802261)*sqrt(20000000)*(q1p*sin(q2 + q3 + atan(1157/5716))*cos(q1) + q2p*cos(q2 + q3 + atan(1157/5716))*sin(q1) + q3p*cos(q2 + q3 + atan(1157/5716))*sin(q1)))/20000000,
      0,q2p*((6127*sin(q2))/10000 + (sqrt(6802261)*sqrt(20000000)*sin(q2 + q3 + atan(1157/5716)))/20000000) + (sqrt(6802261)*sqrt(20000000)*q3p*sin(q2 + q3 + atan(1157/5716)))/20000000,                                                                                             (sqrt(6802261)*sqrt(20000000)*sin(q2 + q3 + atan(1157/5716))*(q2p + q3p))/20000000,
      0,q1p*cos(q1),q1p*cos(q1),
      0,q1p*sin(q1),q1p*sin(q1),
      0,0,0;
  return Jp;

}

Vector3d SimpleEffortControl::getEoMr(Vector6d currentQ,Vector6d currentQp,Vector6d currentQrp,Vector6d currentQrpp){
  Vector3d EoMr;
  double q1,q2,q3;
  q1=currentQ(0);
  q2=currentQ(1);
  q3=currentQ(2);
  double q1p,q2p,q3p;
  q1p=currentQp(0);
  q2p=currentQp(1);
  q3p=currentQp(2);
  double q1rp,q2rp,q3rp;
  q1rp=currentQrp(0);
  q2rp=currentQrp(1);
  q3rp=currentQrp(2);
  double q1rpp,q2rpp,q3rpp;
  q1rpp=currentQrpp(0);
  q2rpp=currentQrpp(1);
  q3rpp=currentQrpp(2);

  EoMr << (43980597*q1rpp)/100000000 + (1877129*q1rpp*cos(2*q2))/8000000 + (q1rpp*sin(2*q2))/50 + (11812969*q1rpp*cos(2*q2 + 2*q3))/200000000 + (q1rpp*sin(2*q2 + 2*q3))/50 + (q2rpp*cos(q2 + q3))/50 + (q3rpp*cos(q2 + q3))/50 + (20311*q2rpp*sin(q2 + q3))/500000 + (20311*q3rpp*sin(q2 + q3))/500000 + (21058499*q1rpp*cos(q3))/100000000 + (q2rpp*cos(q2))/50 + (51361*q2rpp*sin(q2))/500000 + (21058499*q1rpp*cos(2*q2 + q3))/100000000 + (51361*q2p*q2rp*cos(q2))/500000 - (q2p*q2rp*sin(q2))/50 - (21058499*q1p*q3rp*sin(q3))/200000000 - (21058499*q3p*q1rp*sin(q3))/200000000 - (21058499*q1p*q2rp*sin(2*q2 + q3))/100000000 - (21058499*q2p*q1rp*sin(2*q2 + q3))/100000000 - (21058499*q1p*q3rp*sin(2*q2 + q3))/200000000 - (21058499*q3p*q1rp*sin(2*q2 + q3))/200000000 + (q1p*q2rp*cos(2*q2))/50 + (q2p*q1rp*cos(2*q2))/50 - (1877129*q1p*q2rp*sin(2*q2))/8000000 - (1877129*q2p*q1rp*sin(2*q2))/8000000 + (q1p*q2rp*cos(2*q2 + 2*q3))/50 + (q2p*q1rp*cos(2*q2 + 2*q3))/50 + (q1p*q3rp*cos(2*q2 + 2*q3))/50 + (q3p*q1rp*cos(2*q2 + 2*q3))/50 - (11812969*q1p*q2rp*sin(2*q2 + 2*q3))/200000000 - (11812969*q2p*q1rp*sin(2*q2 + 2*q3))/200000000 - (11812969*q1p*q3rp*sin(2*q2 + 2*q3))/200000000 - (11812969*q3p*q1rp*sin(2*q2 + 2*q3))/200000000 + (20311*q2p*q2rp*cos(q2 + q3))/500000 + (20311*q2p*q3rp*cos(q2 + q3))/500000 + (20311*q3p*q2rp*cos(q2 + q3))/500000 + (20311*q3p*q3rp*cos(q2 + q3))/500000 - (q2p*q2rp*sin(q2 + q3))/50 - (q2p*q3rp*sin(q2 + q3))/50 - (q3p*q2rp*sin(q2 + q3))/50 - (q3p*q3rp*sin(q2 + q3))/50,
      (33370597*q2rpp)/50000000 + (15812969*q3rpp)/100000000 - (3371697*cos(q2 + q3))/1000000 - (9016371*cos(q2))/1000000 + (q1rpp*cos(q2 + q3))/50 + (20311*q1rpp*sin(q2 + q3))/500000 + (q1rpp*cos(q2))/50 + (21058499*q2rpp*cos(q3))/50000000 + (21058499*q3rpp*cos(q3))/100000000 + (51361*q1rpp*sin(q2))/500000 - (21058499*q2p*q3rp*sin(q3))/100000000 - (21058499*q3p*q2rp*sin(q3))/100000000 - (21058499*q3p*q3rp*sin(q3))/100000000 + (21058499*q1p*q1rp*sin(2*q2 + q3))/100000000 - (q1p*q1rp*cos(2*q2))/50 + (1877129*q1p*q1rp*sin(2*q2))/8000000 - (q1p*q1rp*cos(2*q2 + 2*q3))/50 + (11812969*q1p*q1rp*sin(2*q2 + 2*q3))/200000000,
      (15812969*q2rpp)/100000000 + (15812969*q3rpp)/100000000 - (3371697*cos(q2 + q3))/1000000 + (q1rpp*cos(q2 + q3))/50 + (20311*q1rpp*sin(q2 + q3))/500000 + (21058499*q2rpp*cos(q3))/100000000 + (21058499*q1p*q1rp*sin(q3))/200000000 + (21058499*q2p*q2rp*sin(q3))/100000000 + (21058499*q1p*q1rp*sin(2*q2 + q3))/200000000 - (q1p*q1rp*cos(2*q2 + 2*q3))/50 + (11812969*q1p*q1rp*sin(2*q2 + 2*q3))/200000000;
  return EoMr;
}


bool SimpleEffortControl::ifSingular(){
  double w;
  Matrix3d temp;
  temp=m_Jef.topRows<3>();
  ROS_INFO_STREAM("~~Jef: \n" << m_Jef);
  //w=abs(temp.determinant());
  //ROS_INFO_STREAM("\n~~det_Jef = " << w);

  Matrix3d Jinv;
  Matrix3d Eye_3;
  Eye_3<<1,0,0,
         0,1,0,
         0,0,1;

  ///Jinv=m_Jef.inverse();
  Jinv=m_Jef.topRows<3>();
  Jinv=Jinv.transpose()*(Jinv*Jinv.transpose()+0.1*Eye_3).inverse();
  ROS_INFO_STREAM("\n~~Jinv = \n" << Jinv);
  ROS_INFO_STREAM("\n~~Jinv.squaredNorm() = " << Jinv.squaredNorm());
  bool isSingular;
  if(Jinv.squaredNorm()<0.001)//(w<=0.00001)
    isSingular=true;
  else
    isSingular=false;
  return isSingular;
}

Vector3d SimpleEffortControl::TrajGen(Vector3d X, Vector3d Xd){
    double dt=0.0002;
    Matrix6d eq;
             eq<< 1,0,0,0,0,0,
                  0,1,0,0,0,0,
                  0,0,2,0,0,0,
                  1,dt,pow(dt,2),pow(dt,3),pow(dt,4),pow(dt,5),
                  0,1,2*dt,3*pow(dt,2),4*pow(dt,3),5*pow(dt,4),
                  0,0,2,6*dt,12*pow(dt,2),20*pow(dt,3);
    Vector6d condi1;
    Vector6d condi2;
    Vector6d condi3;
    Vector6d a_X;
    Vector6d a_Y;
    Vector6d a_Z;
    condi1<<X(0),0,0,Xd(0),0,0;
    condi2<<X(1),0,0,Xd(1),0,0;
    condi3<<X(2),0,0,Xd(2),0,0;
    a_X=eq.inverse()*condi1;
    a_Y=eq.inverse()*condi2;
    a_Z=eq.inverse()*condi3;
    dt /= 100;
    Vector6d temp;
    temp<<1,dt,pow(dt,2),pow(dt,3),pow(dt,4),pow(dt,5);
    Vector3d Xdn;
    Xdn(0)=temp.adjoint()*a_X;
    Xdn(1)=temp.adjoint()*a_Y;
    Xdn(2)=temp.adjoint()*a_Z;
    ROS_INFO_STREAM("~~a_X\n" <<a_X);
    return Xdn;

}

}
}

