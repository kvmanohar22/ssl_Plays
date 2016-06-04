#include "pPassTest.hpp"
#include <utility>
#include "play.hpp"
#include "krssg_ssl_msgs/BeliefState.h"
#include "tactics/tactic.h"
#include <ssl_common/config.h>
#include <ssl_common/geometry.hpp>
#include <math.h>
#include "passingPoint.cpp"
//#include "passProbability.cpp"

namespace Strategy
{
	
	PPassTest::PPassTest(const krssg_ssl_msgs::BeliefState& state) 
      : Play(state)
    {
    	      name = "PassTest";

      assert(HomeTeam::SIZE == 6); // thispassing  TestPlay is applicable for a team of 2 robots only

      PositionPlay = PLAYTYPE_YES;
      AttackPlay   = PLAYTYPE_NO;
      Tactic::Param param;
          
      //**********************selecting the bot to pass  ********************** 
        int passrer_id,receiver_id=0,marker_id;
        float maxProb=0;
        while(receiver_id<6)
       {
       	marker_id=findMarker(receiver_id);
         Vector2D<int> passPoint=findPointForPassingNaive(passrer_id,receiver_id,marker_id);
         
         if(passPoint.x==0 && passPoint.y==0)
         {
           passPoint.x=state.homePos[receiver_id].x;
           passPoint.y=state.homePos[receiver_id].y;
         } //if no valid pass point found then consider the 

         float Prob_scoring,Prob_total,Prob_receiving;
         // Prob_receiving=receiveProbability();
         // Prob_scoring= shootProbability(); 

          Prob_total=Prob_receiving*Prob_scoring;
          if(Prob_total>maxProb)
          {
            maxProb=Prob_total;
            recvrID=receiver_id;
            destPassPoint.x=passPoint.x;
            destPassPoint.y=passPoint.y;
          }

         receiver_id++;
       }

      //just for debigging 
      destPassPoint.x=OPP_GOAL_X;
      destPassPoint.y=CENTER_Y;
      printf("in Pass Test \n");
      //*******************roles for the bots************************* 
      param.PassToPointP.x=destPassPoint.x;
      param.PassToPointP.y=destPassPoint.y;
      roleList[0].push_back(std::make_pair("TPassToPoint", param));
      //roleList[0].push_back(std::make_pair("TStop", param));

      param.ReceiveP.x=destPassPoint.x;
      param.ReceiveP.y=destPassPoint.y;
      roleList[1].push_back(std::make_pair("TReceive", param));
      //roleList[1].push_back(std::make_pair("TStop", param));

      param.PositionP.x= CENTER_X;
      param.PositionP.y= CENTER_Y - 2*GAP;
      param.PositionP.finalSlope= PI/2;
      roleList[2].push_back(std::make_pair("TPosition", param));
      //roleList[2].push_back(std::make_pair("TStop", param));

      param.PositionP.x= CENTER_X - GAP;
      param.PositionP.y= CENTER_Y + GAP/2;
      param.PositionP.finalSlope= -PI/4;
      roleList[3].push_back(std::make_pair("TPosition", param));
      //roleList[3].push_back(std::make_pair("TStop", param));

      param.PositionP.x= CENTER_X - GAP;
      param.PositionP.y= CENTER_Y - GAP/2;
      param.PositionP.finalSlope= PI/4;
      roleList[4].push_back(std::make_pair("TPosition", param));
      //roleList[4].push_back(std::make_pair("TStop", param));

      param.PositionP.x= CENTER_X - GAP;
      param.PositionP.y= CENTER_Y - GAP/2;
      param.PositionP.finalSlope= PI/4;
      roleList[5].push_back(std::make_pair("TPosition", param));
      //roleList[5].push_back(std::make_pair("TStop", param));
    
      computeMaxTacticTransits();
    
    }//constructor

    int PPassTest::findMarker(int receiver_id)
    {
    	int marker_id;
    	float dist=Vector2D<int>::dist(Vector2D<int>(state.awayPos[0].x,state.awayPos[0].y),Vector2D<int>(state.homePos[receiver_id].x,state.homePos[receiver_id].y));
    	for (int id = 0; id < HomeTeam::SIZE; ++id)
    	{
    		if(Vector2D<int>::dist(Vector2D<int>(state.awayPos[id].x,state.awayPos[id].y),Vector2D<int>(state.homePos[receiver_id].x,state.homePos[receiver_id].y)) > dist)
    		{
    			marker_id=id;
    			dist=Vector2D<int>::dist(Vector2D<int>(state.awayPos[0].x,state.awayPos[0].y),Vector2D<int>(state.homePos[receiver_id].x,state.homePos[receiver_id].y));
    		}
    	}
    }

    bool PPassTest::applicable(void) const
    {
      // printf("Set position is applicable\n");
      // TODO make it more sophisticated
      return true;
    }

}// namespace strategy