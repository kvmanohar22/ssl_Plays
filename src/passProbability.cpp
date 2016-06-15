float receiveProbability(const krssg_ssl_msgs::BeliefState& state,int passer_id,int receiver_id,Vector2D<int> passPoint)
{
    float Prob_total,Prob_scoring,Prob_receiving;
    float pa1,pa2,pa3,pa4; 
    /*
      pa1=probability of receiving pass based on number of opponents close to ball
      pa2=probability of receiving pass based on opponents who can intercept the pass
      pa3=probability of receiving pass based on length of pass 
      pa4=probability of receiving pass based on location of passPoint
    */ 
     
      //calculating pa1
      {
        float temp=1.0000f;
        for (int id = 0; id < HomeTeam::SIZE; ++id)
        {
          Vector2D<int> ballPos(state.ballPos.x,state.ballPos.y);
          Vector2D<int> passerPos(state.homePos[passer_id].x,state.homePos[passer_id].y);
          Vector2D<int> receivePos(passPoint.x,state.homePos[receiver_id].y);

          if(Vector2D<int>::dist(ballPos,passerPos)<4*BOT_RADIUS && fabs(Vector2D<int>::angle(passerPos,receivePos)-Vector2D<int>::angle(passerPos,Vector2D<int> (state.awayPos[id].x,state.awayPos[id].y)))< PI/10 ) 
          {
            temp*=fabs(Vector2D<int>::angle(passerPos,receivePos)-Vector2D<int>::angle(passerPos,Vector2D<int> (state.awayPos[id].x,state.awayPos[id].y)))/PI;
          }
        } 
        pa1=temp+0.00001f;
      }
     
      //calculating pa2
      {
        float temp;
        for (int id = 0; id < HomeTeam::SIZE; ++id)
        {
          Vector2D<int> ballPos(state.ballPos.x,state.ballPos.y);
          Vector2D<int> passerPos(state.homePos[passer_id].x,state.homePos[passer_id].y);
          Vector2D<int> receivePos(passPoint.x, passPoint.y);
          Vector2D<int> interceptorPos(state.awayPos[id].x , state.awayPos[id].y);

          if(Vector2D<int>::dist(Vector2D<int> (state.awayPos[id].x,state.awayPos[id].y),receivePos)<6*BOT_RADIUS)
          {
            
            float distance_from_line = fabs(((passPoint.y - state.ballPos.y) * (state.awayPos[id].x)) - ((passPoint.x - state.ballPos.x) * (state.awayPos[id].y)) + (passPoint.x * state.ballPos.y) - (passPoint.y * state.ballPos.x)) / sqrt((receivePos - ballPos).absSq());
            float projected_distance = sqrt((interceptorPos - ballPos).absSq()) * cos( (Vector2D<int>::angle(passerPos,receivePos)-Vector2D<int>::angle(passerPos,Vector2D<int> (state.awayPos[id].x,state.awayPos[id].y))) * PI / 180.0);

            float prob_part_1 = (1 - (projected_distance / sqrt((ballPos - receivePos).absSq()))) + 0.01;
            float prob_part_2 = (distance_from_line / (2.83 * HALF_FIELD_MAXX)) + 0.01;

            if(prob_part_2 * prob_part_1 < temp)
              temp = prob_part_1 * prob_part_2;

          }
        } 
         pa2 = temp+0.00001f;
      }

      //calculating pa3
     {
        float maxX=1;
         Vector2D<int> passerPos(state.homePos[passer_id].x,state.homePos[passer_id].y);
         Vector2D<int> receiverPos(state.homePos[receiver_id].x,state.homePos[receiver_id].y);
         float dista=Vector2D<int>::dist(passerPos,receiverPos);
         if(dista<=maxX)
         pa3=exp(-3*pow(2*dista/maxX-1,2));
         else
         pa3=0.00001f;
    }

    //calculating pa4
    {
    //CONSTANTS ARE SET FOR HALF_FIELD_MAXX=900 AND HALF_FIELD_MAXY=600
    double prx,pry,pconst=0.7;
    int Danger_Self_Goal=-7*HALF_FIELD_MAXX/8,p_thres=7*HALF_FIELD_MAXX/8;

    //FINDING PROBABILITY IN X DIRECTION
    if(passPoint.x<Danger_Self_Goal)
      prx=0;
    else if(passPoint.x<0)
      prx=pconst*erf((passPoint.x-Danger_Self_Goal)/10.0);
    else if(passPoint.x<p_thres)
      prx=pconst+0.3*erf((passPoint.x)/10.0);
    else if(passPoint.x<HALF_FIELD_MAXX)
      prx=1-erf(passPoint.x-10);
    else prx=0;

    //FINDING PROBABILITY IN Y DIRECTION
    if(passPoint.y<-HALF_FIELD_MAXY)
      pry=0;
    else if(passPoint.y<0)
      pry=erf((passPoint.y+HALF_FIELD_MAXY)/8000.0);
    else if(passPoint.y<HALF_FIELD_MAXY)
      pry=1-erf((passPoint.y)/8000.0);
    else pry=0;

    //COMBINING RESULTS
    double pa4=sqrt(prx*pry);

  }
  Prob_receiving=pa1*pa2*pa3*pa4;

}

float shootProbability(const krssg_ssl_msgs::BeliefState& state,int passer_id,int receiver_id,Vector2D<int> passPoint)
{
     float pb1,pb2,pb3;
  /*
    pb1=probability of shot reaching faster than goalkeeper 
    pb2=wide enough theta
    pb3=not enough time for defenders to block the shot
  */

    //calculating pb1
    {
      Vector2D<int> ballPos(state.ballPos.x,state.ballPos.y);
      Vector2D<int> passerPos(state.homePos[passer_id].x,state.homePos[passer_id].y);
      Vector2D<int> receivePos(passPoint.x,passPoint.y);
      Vector2D<int> oppGoaliePos(state.awayPos[state.opp_goalie].x,state.awayPos[state.opp_goalie].y);
      float goalieReceiverDist=Vector2D<int>::dist(receivePos,Vector2D<int>(state.awayPos[state.opp_goalie].x,state.awayPos[state.opp_goalie].y) );
      float goaliePoleDist;
      if(Vector2D<int>::dist(oppGoaliePos,Vector2D<int> (OPP_GOAL_X, OPP_GOAL_MAXY ))>Vector2D<int>::dist(oppGoaliePos,Vector2D<int> (OPP_GOAL_X, OPP_GOAL_MINY )))
      {
          goaliePoleDist=Vector2D<int>::dist(oppGoaliePos,Vector2D<int> (OPP_GOAL_X, OPP_GOAL_MAXY ));
      }
      else goaliePoleDist=Vector2D<int>::dist(oppGoaliePos,Vector2D<int> (OPP_GOAL_X, OPP_GOAL_MINY ));
      pb1=goaliePoleDist/goalieReceiverDist;
    }

    //calculating pb2
    {
      float maxX=1,maxY=1;
      Vector2D<int> GoalPole;
      float theta=fabs(Vector2D<int>::angle(Vector2D<int>(maxX,maxY),GoalPole)-Vector2D<int>::angle(Vector2D<int>(maxX,maxY),Vector2D<int>(state.awayPos[state.opp_goalie].x,state.awayPos[state.opp_goalie].y)));
      float max_possible_theta=fabs(normalizeAngle(Vector2D<int>::angle(Vector2D<int>(maxX,maxY),Vector2D<int>(OPP_GOAL_X,OPP_GOAL_MAXY))-Vector2D<int>::angle(Vector2D<int>(maxX,maxY),Vector2D<int>(OPP_GOAL_X,OPP_GOAL_MINY))));
      pb2=theta/max_possible_theta;
    }

    //calculating pb3
    {
      float maxX=1,maxY=1;
      float angle_temp=fabs(normalizeAngle(Vector2D<int>::angle(Vector2D<int>(state.awayPos[0].x,state.awayPos[0].y),Vector2D<int>(maxX,maxY)) - Vector2D<int>::angle(Vector2D<int>(OPP_GOAL_X,0),Vector2D<int>(maxX,maxY))));
      float dist_temp=Vector2D<int>::dist(Vector2D<int>(maxX,maxY),Vector2D<int>(OPP_GOAL_X,0));
      float temp=dist_temp*angle_temp;
      for (int id = 0; id < HomeTeam::SIZE; ++id)
      {
        if(fabs(normalizeAngle(Vector2D<int>::angle(Vector2D<int>(state.awayPos[id].x,state.awayPos[id].y),Vector2D<int>(maxX,maxY)) - Vector2D<int>::angle(Vector2D<int>(OPP_GOAL_X,0),Vector2D<int>(maxX,maxY)))) * Vector2D<int>::dist(Vector2D<int>(maxX,maxY),Vector2D<int>(OPP_GOAL_X,0)) > temp)
        {
          temp=fabs(normalizeAngle(Vector2D<int>::angle(Vector2D<int>(state.awayPos[id].x,state.awayPos[id].y),Vector2D<int>(maxX,maxY)) - Vector2D<int>::angle(Vector2D<int>(OPP_GOAL_X,0),Vector2D<int>(maxX,maxY))) * Vector2D<int>::dist(Vector2D<int>(maxX,maxY),Vector2D<int>(OPP_GOAL_X,0)));
        }
      }
      pb3=temp/=Vector2D<int>::dist(Vector2D<int>(maxX,maxY),Vector2D<int>(OPP_GOAL_X,0));
    }
  float Prob_scoring=pb1*pb2*pb3;
  return Prob_scoring;
}
