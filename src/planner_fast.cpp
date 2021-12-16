#include <planner_fast.h>

void TrajectoryGenerator_fast::read_param (const ros::NodeHandle *nh_priv){
// { nh_(*nh_priv);
    // cout << "mk2" << endl;
  config.loadParameters(*nh_priv);}
// generate the trajectory
void TrajectoryGenerator_fast::replan_traj(double MAXVEL,Vector3d &start,Vector3d &vi,Vector3d &ai,MatrixXd &waypoints,MatrixXd &cd_c,VectorXd &cd_r, vec_Vec3f *obs_pointer, dynobs_tmp *dynobs, double plan_t, bool full_trip)
{  dynobs_pointer = dynobs;
   
  //  cout << "1" << endl;

   cout << "waypoints:\n" <<waypoints << endl;
   cout << "vi, ai: \n" << vi << endl << ai <<endl;
   if (!if_config || full_trip)
   {wPs.clear();
    if (waypoints.cols() < 3)
   {
    wPs.emplace_back(start.transpose());
    wPs.emplace_back(((start.transpose()+waypoints.col(1).transpose())/2));
    wPs.emplace_back(waypoints.col(1).transpose());}
    else{
      wPs.emplace_back(start.transpose());
      for (uint i = 1; i<waypoints.cols(); i++)
      {
        wPs.emplace_back(waypoints.col(i).transpose());
      }}
   cout << "received obs points:" << obs_pointer->size()<<endl << obs_pointer <<endl;

   gen_polyhedrons(obs_pointer);}
   Matrix3d iniState,finState; 
  //  Vector3d final_va = Vector3d::Zero(3);
   iniState.col(0) = start;
   iniState.col(1) = vi;
   iniState.col(2) = ai;
   finState.col(0) = wPs.back();
   finState.col(1) = Vector3d::Zero(3);
   finState.col(2) = Vector3d::Zero(3);
  // check_wps_in_polyH();
   Traj_opt(iniState,finState,plan_t);
     }

void TrajectoryGenerator_fast::check_wps_in_polyH (void)
{

 Vector3d pos;
 Polyhedron3D poly;
 int i =0;
 for (uint j =0; j<2;j++)  // check if neccessary to update corridor
 {
  //cout<<"(in func) check traj safe:\n"<<j<<endl<<total_t<<endl;
  pos = wPs[i];
  if (i==0)
  {
  poly = decompPolys[0];}
  else{poly = decompPolys.back();}
  Vec3f pt;
  pt[0] = pos(0);
  pt[1] = pos(1);
  pt[2] = pos(2);
  
  bool if_inside = poly.inside(pt);
  if (!if_inside)
  {
  cout << "The "<<i<<" th waypoint is not in SFC"<<endl;}
  else{cout << "The "<<i<<" th waypoint is in SFC"<<endl;}
  i = wPs.size()-1;
   }

 }


bool TrajectoryGenerator_fast::check_polyH_safe (const double start_t, const MatrixXd &waypoints, Vector3d &start, vec_Vec3f *obs_pointer, dynobs_tmp *dynobs, double plan_t)
{
dynobs_pointer = dynobs;
 double start_t1 = plan_t - start_t;
 check_sfc_ind = 0;
 Vector3d pos;
 double dt = 0.1;
 bool old_plhd_safe = true;
 Polyhedron3D poly = decompPolys[check_sfc_ind];
 for (uint i =0; i< waypoints.cols(); i++)  // check if neccessary to update corridor
 {
  //cout<<"(in func) check traj safe:\n"<<j<<endl<<total_t<<endl;
  pos = waypoints.col(i);

  Vec3f pt;
  pt[0] = pos(0);
  pt[1] = pos(1);
  pt[2] = pos(2);
  
  // check_sfc_ind = 0;
  
  if(!poly.inside(pt)){
      check_sfc_ind +=1;
  if (check_sfc_ind > hPolys.size()-1)
  {
   break;}
      poly = decompPolys[check_sfc_ind];
      if(!poly.inside(pt)){
      old_plhd_safe = false;
      break;}
  }
   }
//  vector<Vector3d> out_points;
//  vector<Vector3d> out_centers;

   wPs.clear();
    if (waypoints.cols() < 3)
   {
    wPs.emplace_back(start.transpose());
    wPs.emplace_back(((start.transpose()+waypoints.col(1).transpose())/2));
    wPs.emplace_back(waypoints.col(1).transpose());}
    else{
      wPs.emplace_back(start.transpose());
      for (uint i = 1; i<waypoints.cols(); i++)
      {
        wPs.emplace_back(waypoints.col(i).transpose());
      }}
if (old_plhd_safe)
{ cout<<"old corridor is safe for new waypoints!"<<endl;
  return true;}
  cout<<"old corridor is not safe for new waypoints!"<<endl;
gen_polyhedrons(obs_pointer);
if_config = true;
total_t = traj.getTotalDuration();
 if (start_t1+dt > total_t)
 { cout<< "traj timeout!"<<endl;
   return false;
 }
//  cout<< "mk1"<<endl;
 check_sfc_ind = 0;
 poly = decompPolys[check_sfc_ind];
//  cout<< "mk2"<<endl;
 for (double j = start_t1+dt; j<total_t; j+=dt)
 {
  //cout<<"(in func) check traj safe:\n"<<j<<endl<<total_t<<endl;
  pos = traj.getPos(j);

  Vec3f pt;
  pt[0] = pos(0);
  pt[1] = pos(1);
  pt[2] = pos(2);
  if (!dyn_safe_check(pos,j+start_t))
  {cout<< "dyn check fail!"<<endl;
    return false;}
  if(!poly.inside(pt)){
      check_sfc_ind +=1;
      if (check_sfc_ind > hPolys.size()-1)
      { cout<< "old traj SFC check fail!--1"<<endl;
        return false;}
      poly = decompPolys[check_sfc_ind];
      if(!poly.inside(pt)){
      cout<< "old traj SFC check fail!--2"<<endl;
      return false;}
  }
   }
   cout<<"old traj all in new corridor!"<<endl;
  return true;
 }

inline bool TrajectoryGenerator_fast::dyn_safe_check(Vector3d pt, double check_t)
// { return true; // for rosbag tests!!!
  {
  if (dynobs_pointer->dyn_number>0)
  {
  double t_base = check_t - dynobs_pointer->time_stamp;
  Vector3d ct_center;
  for (int j =0; j<dynobs_pointer->dyn_number; j++)
  {
    ct_center = dynobs_pointer->centers[j] + t_base*dynobs_pointer->vels[j];
    if ((((ct_center - pt).cwiseAbs() - dynobs_pointer->obs_sizes[j]*0.5).array()<0).all())
    {return false;}
  }}
  else if (dynobs_pointer->ball_number>0)
  {
  t_base = check_t - dynobs_pointer->ball_time_stamp;
  for (int j =0; j<dynobs_pointer->ball_number; j++)
  {
    ct_center = dynobs_pointer->ballpos[j] + t_base*dynobs_pointer->ballvel[j] + 0.5*t_base*t_base*dynobs_pointer->ballacc[j];
    if ((((ct_center - pt).cwiseAbs() - dynobs_pointer->ball_sizes[j]*0.5).array()<0).all())
    {return false;}
  }}
   return true;
}
void TrajectoryGenerator_fast::Traj_opt(const MatrixXd &iniState, const MatrixXd &finState, double plan_t)
 
{
    chrono::high_resolution_clock::time_point tic = chrono::high_resolution_clock::now();
    SE3GCOPTER nonlinOpt;
    // Trajectory traj;
           cout << "received dynamic obs number:" << dynobs_pointer->dyn_number << endl << dynobs_pointer<<endl<<"dyn_timestamp:"<<dynobs_pointer->time_stamp<<endl<<plan_t<<endl;
    ROS_INFO("Begin to optimize the traj~");
    cout << "final state in use:" << endl << finState.col(0) << endl;
    if (!nonlinOpt.setup(config.rho, config.totalT, iniState, finState, hPolys, INFINITY,
                            config.qdIntervals, config.horizHalfLen, config.vertHalfLen,
                            config.safeMargin, config.velMax, config.thrustAccMin, config.thrustAccMax,
                            config.bodyRateMax, config.gravAcc, config.penaltyPVTB, config.useC2Diffeo, plan_t,dynobs_pointer))
    {
        ROS_INFO("gcopter initialize fail!");
        return;
    }
    double finalObj = nonlinOpt.optimize(traj, config.optRelTol);
    
    chrono::high_resolution_clock::time_point toc = chrono::high_resolution_clock::now();
    double compTime = chrono::duration_cast<chrono::microseconds>(toc - tic).count() * 1.0e-3;
    
    printf("finished!!!\n");
    cout << "Optimization time usage: " << compTime << " ms" << endl;
    cout << "Final cost: " << finalObj << endl;
    cout << "Maximum Vel: " << traj.getMaxVelRate() << endl;
    cout << "Maximum Acc: " << traj.getMaxAccRate() << endl;
    cout << "Total traj Duration: " << traj.getTotalDuration() << endl;
}

void TrajectoryGenerator_fast::gen_polyhedrons(vec_Vec3f *obs_pointer)
{   chrono::high_resolution_clock::time_point tic = chrono::high_resolution_clock::now();
    cout << "gen polyhedrons" <<endl;
    EllipsoidDecomp3D decomp_util;
   // cout << "mk00:" <<obs_pointer->size()<<endl;
    decomp_util.set_obs(*obs_pointer);
    //cout << "mk0" <<endl;
    decomp_util.set_local_bbox(Eigen::Vector3d(config.polyhedronBox(0),
                                               config.polyhedronBox(1),
                                               config.polyhedronBox(2)));
    decompPolys.clear();
    //vec_E<Ellipsoid3D> ellips; 
    // cout << "mk1" <<endl;
    for(uint i = 0;i<wPs.size()-1;){
         //find the farest unblocked point
        vec_Vec3f line;
        line.push_back(wPs[i]);
        line.push_back(wPs[i+1]);
        decomp_util.dilate(line);
        Polyhedron3D poly = decomp_util.get_polyhedrons()[0];
        //Ellipsoid3D ellip = decomp_util.get_ellipsoids()[0];
        decompPolys.push_back(poly);
       // ellips.push_back(ellip);
        //find the nearest one to the boundry of poly.
        cout << "mk22 " << i<<endl<<wPs.size()<<endl;
        if (i+2 < wPs.size())
        {
        uint j;
        for(j=i+2;j<wPs.size();j++){
            Vec3f pt;
            pt[0] = wPs[j](0);
            pt[1] = wPs[j](1);
            pt[2] = wPs[j](2);
            if(!poly.inside(pt)){
                break;
            }
        }
        j--;
        if(j>=wPs.size()-1){
          if (decompPolys.size()==1)
          {j = wPs.size()-2;}
          else{
          break;}
          break;
        }
        i = j;

        // int wp;
        // wp = round((1*i+4*j)/5);
        // i = wp;
        }
        else{i+=1;}
    }
    cout << "Number of polyhedrons: " << decompPolys.size() << endl;
    for (size_t i = 0; i < decompPolys.size(); i++)
    {
        decompPolys[i].add(Plane3D(Eigen::Vector3d(0.0, 0.0, config.mapHeight),
                                          Eigen::Vector3d(0.0, 0.0, 1.0)));
        decompPolys[i].add(Plane3D(Eigen::Vector3d(0.0, 0.0, 0.0),
                                          Eigen::Vector3d(0.0, 0.0, -1.0)));
    }
    // visualization.visualizePolyH(decompPolys);   
    hPolys.clear();
    Eigen::MatrixXd current_poly;
    for (uint i = 0; i < decompPolys.size(); i++)
    {
        vec_E<Plane3D> current_hyperplanes = decompPolys[i].hyperplanes();
        current_poly.resize(6, current_hyperplanes.size());
        for (uint j = 0; j < current_hyperplanes.size(); j++)
        {
            current_poly.col(j) << current_hyperplanes[j].n_, current_hyperplanes[j].p_;
            //outside
        }
        hPolys.push_back(current_poly);
    }
    
    double compTime = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
    cout << "polyhedrons finished! time cost (ms)： " <<compTime<<endl;
    }

bool TrajectoryGenerator_fast::get_new_wps(Trajectory traj, const MatrixXd &cd_c, const VectorXd &cd_r)
{
 total_t = traj.getTotalDuration();
 if (total_t > 20)
 {return false;}
 if (wPs.size() <3)
 { 
   return true;}
 check_sfc_ind = 0;
 Vector3d pos;
 double dt = 0.1;
 vector<Vector3d> out_points;
 vector<Vector3d> out_centers;
 int count = 0, count_tt = 0;
 for (int j =1; j*dt<total_t; j++)
 {
   
// if (check_sfc_ind > cd_c.rows()-1)
//   {if (count !=0)
//    { MatrixXd Moutpts = Map<MatrixXd>(out_points[0].data(),out_points.size(),3);
//  out_centers.emplace_back(Moutpts.colwise().mean());
//  vector<Vector3d> out_points;
//    }
//   break;}
  pos = traj.getPos(j*dt);
  
 // cout<<"j:"<<j<<endl;
  if (total_t >dt+j*dt && safe_check(pos,cd_c,cd_r))
  {
   count +=1;
   count_tt+=1;
   out_points.emplace_back(pos);
   cout<<"unsafe pos,time:"<<pos<<endl<<j*dt<<endl << total_t<<endl;
   }
else if (count !=0)
{count = 0;
 //int osize = out_points.size();
 MatrixXd Moutpts = Map<MatrixXd>(out_points[0].data(),3,out_points.size());
 out_centers.emplace_back(Moutpts.rowwise().mean());
 out_points.clear();
 cout << "found unsafe segment!" <<endl << out_centers[out_centers.size()-1]<<endl;
// break;
}
//cout << "safe_check returned!:"<< check_sfc_ind << endl;
 }
 if (count_tt ==0)
 {return true;}
 else{
  update_wps(out_centers,cd_c,cd_r);
  return false;}
}

bool TrajectoryGenerator_fast::check_traj_safe(const MatrixXd &cd_c, const VectorXd &cd_r, const double start_t)
{
 total_t = traj.getTotalDuration();
 check_sfc_ind = 0;
 Vector3d pos;
 double dt = 0.1;
 vector<Vector3d> out_points;
 vector<Vector3d> out_centers;
 if (start_t+dt > total_t)
 { return false;
 }
 for (double j = start_t+dt; j<total_t; j+=dt)
 {
  //cout<<"(in func) check traj safe:\n"<<j<<endl<<total_t<<endl;
  pos = traj.getPos(j);
  if (check_sfc_ind > cd_c.rows()-1)
  {return false;}
  if (safe_check(pos,cd_c,cd_r))
  {

    return false;
 //  out_points.emplace_back(pos);
   }

 }

 return true;
 
 
}

// void TrajectoryGenerator_fast::update_wps( const vector<Vector3d> &out_centers, const MatrixXd &cd_c, const VectorXd &cd_r)
// {
//  vector<Vector3d> wPs_copy = wPs;
//  for (int k=0; k<out_centers.size(); k++)
//  {
//  //MatrixXd centers = VectorXd::Map(&waypoints[0],waypoints.size());
//  VectorXd dists = (cd_c.rowwise() - out_centers[k].transpose()).rowwise().norm();
//  VectorXd gap = dists - cd_r;
//  cout << "update_wps!" << gap << endl << wPs.size()<< endl;
//  ptrdiff_t min_ind;
//  double min_gap = gap.minCoeff(&min_ind);
//  Vector3d min_row = cd_c.row(min_ind);
//  Vector3d new_wp = min_row + (out_centers[k] - min_row).normalized()*(cd_r(min_ind)*0.5);
//  
//  wPs.insert(wPs.begin()+get_insert_ind(new_wp),new_wp);
//  }
//  }

void TrajectoryGenerator_fast::update_wps( const vector<Vector3d> &out_centers, const MatrixXd &cd_c, const VectorXd &cd_r)
{
  vector<Vector3d> wPs_copy = wPs;
  Vector3d new_wp;
  double a, b, c, s;
  for (uint k=0; k<out_centers.size(); k++)
  {
  //MatrixXd centers = VectorXd::Map(&waypoints[0],waypoints.size());
  int location = get_insert_ind_1(out_centers[k],cd_c, cd_r);
 // MatrixXd ck = out_centers[k];
//   a= (cd_c.row(location) - ck).norm();
//   b= (cd_c.row(location+1) - ck).norm();
  a = cd_r(location);
  b = cd_r(location+1);
  c= (cd_c.row(location) - cd_c.row(location+1)).norm();
  s= (a+b+c)/2;
  cout << "s of triangle: " << s <<endl << wPs.size()<< endl << a << endl << b << endl <<c <<endl;
  if (out_centers[k](2) < 0.2)
  {new_wp << out_centers[k](0),out_centers[k](1), 0.3;}
  else if (c < a+b)
  {
  double joint_r = sqrt(s*(s-a)*(s-b)*(s-c))*2/c;
  Vector3d ancor = cd_c.row(location) + (cd_c.row(location+1)-cd_c.row(location)).normalized()*sqrt(a*a-joint_r*joint_r);
  new_wp = ancor + (out_centers[k]-ancor).normalized()*joint_r*0.6;}
  else {
  new_wp = (cd_c.row(location) + cd_c.row(location+1))/2; }
  cout << "going to insert!" << new_wp << endl;
  bool if_insert = true;
  for (uint i =0; i<wPs.size(); i++)
  {if ((wPs[i]-new_wp).norm()<0.3)
  {if_insert = false;
   break;}}

  int insert_pos = get_insert_ind(new_wp);
  if (if_insert)
  {wPs.insert(wPs.begin()+insert_pos,new_wp);}
  }
  }

// inline int TrajectoryGenerator_fast::get_insert_ind(const Vector3d &check_c)
// {
//  for (int i=0; i<wPs.size()-1; i++)
//  {
//   if (((check_c-wPs[i]).norm() + (check_c-wPs[i+1]).norm())/(wPs[i]-wPs[i+1]).norm() < 1.01)
//   {return i+1;
//   }
//  }
//  
// }

inline int TrajectoryGenerator_fast::get_insert_ind_1(const Vector3d &check_c, const MatrixXd &cd_c, const VectorXd &cd_r)
{
 double base;
 for (uint i=0; i<cd_c.rows()-1; i++)
 {base = (cd_c.row(i)-cd_c.row(i+1)).squaredNorm();
  cout << "mark" << endl << cd_c.row(i)<<endl<<cd_c.row(i+1)<<endl<<check_c.transpose()<<endl<<base<<endl;
  if (((check_c.transpose()-cd_c.row(i)).squaredNorm() < (check_c.transpose()-cd_c.row(i+1)).squaredNorm()+ base) && ((check_c.transpose()-cd_c.row(i+1)).squaredNorm() < (check_c.transpose()-cd_c.row(i)).squaredNorm()+ base))
  {cout<<"insert position (cd_c):" << i << endl;
  return i;
  }
 }
 return 0;
}

inline int TrajectoryGenerator_fast::get_insert_ind(const Vector3d &check_c)
{
 double base;
 for (uint i=0; i<wPs.size()-1; i++)
 {base = (wPs[i]-wPs[i+1]).squaredNorm();
 cout << "mark1" << endl;
  if ((check_c-wPs[i]).squaredNorm() < (check_c-wPs[i+1]).squaredNorm()+ base && (check_c-wPs[i+1]).squaredNorm() < (check_c-wPs[i]).squaredNorm()+ base)
  {cout<<"insert position:" << i << endl;
  return i+1;
  }
 }
 return 0;
}

// inline bool TrajectoryGenerator_fast::safe_check(const Vector3d pos, const MatrixXd &cd_c, const VectorXd &cd_r)
// {
//  // cout<<"safe_check begin:"<< check_sfc_ind <<"\n pos:"<< pos <<"\n cd_c:"<<cd_c<<"\n cd_r:"<<cd_r<<endl;
//  Vector3d checkrow = cd_c.row(check_sfc_ind);
//  bool safe = ((pos - checkrow).norm() > cd_r(check_sfc_ind));
//  if (safe)  //safe = true for collision (not in SFC)
//  {
// //  cout<<"mark3"<<endl;
//   check_sfc_ind +=1;
//  if (check_sfc_ind > cd_c.rows()-1)
//   {return safe;}
//  checkrow = cd_c.row(check_sfc_ind);
//  safe = ((pos - checkrow).norm() > cd_r(check_sfc_ind));
//  //cout<<"mark2"<<endl;
//   if (safe and check_sfc_ind+1 < cd_c.rows())
//   {
// //   cout<<"mmm:\n"<<(cd_c.block(check_sfc_ind+1, 0, cd_c.rows()-check_sfc_ind-1, 3).rowwise() - pos)<<endl;
//   VectorXd gaps = (cd_c.block(check_sfc_ind+1, 0, cd_c.rows()-check_sfc_ind-1, 3).rowwise() - pos.transpose()).rowwise().norm() - cd_r.tail(cd_c.rows()-check_sfc_ind-1);
//  //  cout<<"mark1"<<endl;
//    for (int k=0;k<gaps.size();k++)
//    {
//    if (gaps(k) < 0)
//    {
//   if (k != gaps.size()-1)
//    {
//    safe = false;
//    check_sfc_ind = k+check_sfc_ind+1;}
//   
//   else{check_sfc_ind -=1;}
//   break; 
//   }
//    }}
// 
//   }
//  // cout<<"safe_check end:"<< check_sfc_ind <<endl;
//  return safe;
// }

inline bool TrajectoryGenerator_fast::safe_check(const Vector3d pos, const MatrixXd &cd_c, const VectorXd &cd_r)
{
 double min_dis = ((cd_c.rowwise() - pos.transpose()).rowwise().norm() - cd_r).minCoeff();
 //cout << "minimal distance: " << min_dis <<endl;
 bool safe = (min_dis > 0 || pos(2) < 0.3);
 return safe;
}

void TrajectoryGenerator_fast::get_wPs(const MatrixXd &waypoints, const MatrixXd &cd_c, const VectorXd &cd_r, const Vector3d &start)
{

   //vector<Vector3d> wPs;
   vector<double> dists;
   wPs.emplace_back(start.transpose());
   if (waypoints.rows() > 2){ 
   double base = (waypoints.row(0) - waypoints.row(waypoints.rows()-1)).norm();
 //  cout << "mark2" << endl;
   Vector3d vec2,vec1;
   double dis;
   for (uint k = 1; k < waypoints.rows()-1; k++)
   {
   vec2 = waypoints.row(k) - waypoints.row(0);
   vec1 = waypoints.row(k) - waypoints.row(waypoints.rows()-1);
   dis = vec1.cross(vec2).norm()/base;
   dists.push_back(dis);
   cout << "mark3" <<endl <<k<< endl;
   }
 //  cout << "mark1" << endl;
  VectorXi ind;
  VectorXd sorted_vec;
  VectorXd re = VectorXd::Map(&dists[0],dists.size());
   sort_vec(re,sorted_vec,ind);
   wPs.emplace_back(waypoints.row(ind(0)+1));}

   cout << "mark2  " << waypoints.row(waypoints.rows()-1) << endl;
   
   wPs.emplace_back(waypoints.row(waypoints.rows()-1));
  // return wPs
}

void TrajectoryGenerator_fast::sort_vec(const VectorXd& vec, VectorXd& sorted_vec,  VectorXi& ind){
  ind=VectorXi::LinSpaced(vec.size(),0,vec.size()-1);//[0 1 2 3 ... N-1]
  auto rule=[vec](int i, int j)->bool{
    return vec(i)>vec(j);
  };
  sort(ind.data(),ind.data()+ind.size(),rule);

  sorted_vec.resize(vec.size());
  for(uint i=0;i<vec.size();i++){
    sorted_vec(i)=vec(ind(i));
  }
}

// allocate time for waypoints
VectorXd TrajectoryGenerator_fast::allocateTime(const MatrixXd &wayPs,
             double vel,
             double acc)
{
    int N = (int)(wayPs.cols()) - 1;
    VectorXd durations(N);
    if (N > 0)
    {
        Eigen::Vector3d p0, p1;
        double dtxyz, D, acct, accd, dcct, dccd, t1, t2, t3;

        for (int k = 0; k < N; k++)
        {
            p0 = wayPs.col(k);
            p1 = wayPs.col(k + 1);
            D = (p1 - p0).norm();       // distance

            acct = vel / acc;                   // accelerate time
            accd = (acc * acct * acct / 2);     // accelerate distance
            dcct = vel / acc;                   // de-accelerate time
            dccd = acc * dcct * dcct / 2;       // de-accelerate distance

            if (D < accd + dccd)
            {
                t1 = sqrt(acc * D) / acc;
                t2 = (acc * t1) / acc;
                dtxyz = t1 + t2;
            }
            else
            {
                t1 = acct;
                t2 = (D - accd - dccd) / vel;
                t3 = dcct;
                dtxyz = t1 + t2 + t3;
            }

            durations(k) = dtxyz;
        }
    }

    return durations;
}

// double TrajectoryGenerator_fast::generate()
// {
//     // generate&get trajectory
//     jerkOpt.generate(waypoints.block(0, 1, 3, waypGen.N - 2), ts);
//     jerkOpt.getTraj(minJerkTraj);
// 
//     // check maximum velocity
//     while ((!minJerkTraj.checkMaxVelRate(MaxVel)) || (!minJerkTraj.checkMaxAccRate(5.0)))
//     {
//         printf("maximum velocity %.2f\n", minJerkTraj.getMaxVelRate());
// 
//         // re-allocate time
//         MaxVelCal = MaxVelCal - 0.2;
//         ts = allocateTime(waypoints, MaxVelCal, 5.0);
// 
//         // generate&get trajectory
//         jerkOpt.generate(waypoints.block(0, 1, 3, waypGen.N - 2), ts);
//         jerkOpt.getTraj(minJerkTraj);
//     }
//     printf("maximum velocity %.2f\n", minJerkTraj.getMaxVelRate());
//     int maxRow;
//     
//     do
//     {   double max_time = ts.maxCoeff(&maxRow);
//          ts (maxRow) = max_time-0.2;
//         //  VectorXd grad_t = jerkOpt.getGradT();
//         // grad_t.head(1) << 0;
//         // grad_t.tail(1) << 0;
//         // ts = ts - grad_t*0.02;
//  cout<<"time locations:   "<<ts<<"\n grad_t: \n"<<endl; //grad_t<<endl;
//         // generate&get trajectory
//         jerkOpt.generate(waypoints.block(0, 1, 3, waypGen.N - 2), ts);
//         jerkOpt.getTraj(minJerkTraj);
//         printf("maximum velocity222:  %.2f\n", minJerkTraj.getMaxVelRate());
//     }
//     while  ((minJerkTraj.checkMaxVelRate(MaxVel)) && (minJerkTraj.checkMaxAccRate(5.0)));
//    
//     return minJerkTraj.getTotalDuration();
// }

void TrajectoryGenerator_fast::get_desire(double timee, Vector3d &p_d, Vector3d &v_d, Vector3d &a_d,Vector3d &p_d_yaw)
{
    p_d = traj.getPos(timee);
    v_d = traj.getVel(timee);
    a_d = traj.getAcc(timee);
    p_d_yaw = traj.getPos(clip(timee+3.0,0.0,total_t-0.01));
    // p_d_yaw = traj.getPos(total_t-0.1);
    // cout<<"pd,vd,ad,p_d_yaw: \n"<<p_d<<"\n"<<v_d<<"\n"<<a_d<<"\n"<<p_d_yaw<<endl;
}

void TrajectoryGenerator_fast::get_traj_samples(MatrixXd &sp_pos, MatrixXd &sp_vel, MatrixXd &sp_acc, double start_t)
{
  total_t = traj.getTotalDuration();
 // cout << "pub traj:" << total_t <<endl;
  start_t = min(start_t,total_t);
  int num = (total_t-start_t)/0.2;
  num = max(num,1);
  sp_pos.resize(num,3);
  sp_vel.resize(num,3);
  sp_acc.resize(num,3);
  for (int i =0; i<num; i++)
  {
  sp_pos.row(i) = traj.getPos(start_t + i*0.2);
  sp_vel.row(i) = traj.getVel(start_t + i*0.2);
  sp_acc.row(i) = traj.getAcc(start_t + i*0.2);
  }
} 
