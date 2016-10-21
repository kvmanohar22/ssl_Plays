#ifndef PDEFENCE_HPP
#define PDEFENCE_HPP

#include <utility>
#include "play.hpp"
#include "krssg_ssl_msgs/BeliefState.h"
#include "tactics/tactic.h"
#include <ssl_common/config.h>
#include <ssl_common/geometry.hpp>

namespace Strategy 
{
	class PDefence : public Play 
	{
	public:
		inline PDefence(const krssg_ssl_msgs::BeliefState& state)
			: Play(state)
		{
			name = "DefencePlay";

			Tactic::Param param;

			//some local variables
			float x_threshold;//this is to determine if only one primary defender is sufficient
			float dist_threshold = 1000.0f;//check nearest_opp_to_ball() function
			static int flag = 0; //this corresponds only one primary defender being used

			/*
				TODO:
				#1 calculate the x_threshold value
				#2 calculate the dist_threshold value
				#3 intercept_idx @ line no 124
				#4 threshold value as per conditions
			*/

 
			// //goalie is common for all the cases
			cout<<"goalie assigned \n";
			roleList[0].push_back(std::make_pair("TGoalie", param));

			  param.MarkBotP.awayBotID = 0;
			  roleList[1].push_back(std::make_pair("TMark", param));

			  param.InterceptP.awayBotID = 1;
			  roleList[2].push_back(std::make_pair("TPosition", param));

			  param.PositionP.x= CENTER_X - GAP;
			  param.PositionP.y= CENTER_Y + GAP/2;
			  param.PositionP.finalSlope= -PI/4;
			  roleList[3].push_back(std::make_pair("TPosition", param));
			  roleList[3].push_back(std::make_pair("TStop", param));

			  param.PositionP.x= CENTER_X - GAP;
			  param.PositionP.y= CENTER_Y - GAP/2;
			  param.PositionP.finalSlope= PI/4;
			  roleList[4].push_back(std::make_pair("TPosition", param));
			  roleList[4].push_back(std::make_pair("TStop", param));

			  param.PositionP.x= CENTER_X + GAP;
			  param.PositionP.y= CENTER_Y + GAP/2;
			  param.PositionP.finalSlope= PI/4;
			  roleList[5].push_back(std::make_pair("TPosition", param));
			  roleList[5].push_back(std::make_pair("TStop", param));

			// /*
			// 	there are 3 possible cases
			// 	#1 velocity of ball is less than a threshold value
			// 	 	AND
			// 	 	dist(nearest opp to ball, ball) < threshold value

			// 	#2 velocity of ball is less than a threshold value
			// 	 	AND
			// 	 	dist(nearest opp to ball, ball) > threshold value

			// 	#3 velocity of ball is greater than a threshold value

			// */

			//THREE MAJOR CASES
			if(((pow(state.ballVel.x, 2) + pow(state.ballVel.y, 2)) < LOW_BALL_VELOCITY_THRES_SQ)
				 && nearest_opp_to_ball(dist_threshold)){

				//POSITION THE PRIMARY DEFENDERS
				if(state.ballPos.x <= x_threshold){
					//since the open angle is less one primary defender is sufficient
					if(state.ballPos.y > 0)
						param.DefendARCP.side = 0;
					else 
						param.DefendARCP.side = 1;
				cout <<"inner primary defender tactic assigned\n";
				roleList[4].push_back(std::make_pair("TDefendARC", param));
				}
				else{
					//both the primary defenders would be required
					flag = 1;

					cout << "else condition primary defender assigned\n";
					param.DefendARCP.side = 0;
					roleList[4].push_back(std::make_pair("TDefendARC", param));

					param.DefendARCP.side = 1;
					roleList[5].push_back(std::make_pair("TDefendARC", param));
				}

			// 	//POSITION THE SECONDARY DEFENDERS
			// 	float nearest_dist = 1000000.0f;
			// 	int nearest_bot = -1;
			// 	Vector2D<float> ball_pos(state.ballPos.x, state.ballPos.y);
			// 	for(int idx = 0; idx != AwayTeam::SIZE; ++idx){
			// 		Vector2D<float> away_bot(state.awayPos[idx].x, state.awayPos[idx].y);

			// 		if(Vector2D<float>::dist(ball_pos, away_bot) < nearest_dist){
			// 			nearest_bot = idx;
			// 			nearest_dist = Vector2D<float>::dist(ball_pos, away_bot);
			// 		}
			// 	}

			// 	//count the number of opponent bots on our side
			// 	int count = 0;
			// 	std::vector<int> opp_in_our_half;

			// 	for(int idx = 0; idx != AwayTeam::SIZE; ++idx){
			// 		//make sure that we donot include the bot which is nearest to the ball
			// 		if(state.awayPos[idx].x < 0 && idx != nearest_bot){
			// 			++count;

			// 			//note down the opp. botIDs
			// 			opp_in_our_half.push_back(idx);
			// 		}
			// 	}

			// 	if(count == 1){
			// 		//go for one marker and one intercept
			// 		if(!opp_in_our_half.empty()){
			// 	    	param.MarkBotP.awayBotID = opp_in_our_half[0];
			// 		 roleList[3].push_back(std::make_pair("TMark", param));
			// 	    }
			// 		//param.InterceptP.awayBotID = opp_in_our_half[0];
			// 		//param.InterceptP.where = 1;
			// 		//roleList[4].push_back(std::make_pair("TIntercept", param));
			// 	}
			// 	else if(count == 2){
			// 		if(!opp_in_our_half.empty()){
			// 		   param.MarkBotP.awayBotID = opp_in_our_half[0];
			// 		   roleList[3].push_back(std::make_pair("TMark", param));

			// 		   param.MarkBotP.awayBotID = opp_in_our_half[1];
			// 		   roleList[4].push_back(std::make_pair("TMark", param));
			// 		}
			// 		if(!flag){
			// 		   //intercept the goal of the bot which is more likely to score the goal
			// 		   //this tactic is assigned to the extra primary defender
			// 		   int intercept_idx = 0;
			// 		   //param.InterceptP.awayBotID = opp_in_our_half[intercept_idx];
			// 		   //param.InterceptP.where = 1;
			// 		   //roleList[2].push_back(std::make_pair("TIntercept", param));
			// 		}
			// 	}
			// 	else {
			// 		   if(!opp_in_our_half.empty())
			// 		   {
			// 		   	param.MarkBotP.awayBotID = opp_in_our_half[0];
			// 		   	roleList[3].push_back(std::make_pair("TMark", param));
			// 		   }
			// 		   param.MarkBotP.awayBotID = opp_in_our_half[1];
			// 		   roleList[4].push_back(std::make_pair("TMark", param));

			// 		if(!flag){
			// 			//we'll have to even mark the other opponent instead of intercepting
			// 			param.MarkBotP.awayBotID = opp_in_our_half[2];
			// 		   roleList[2].push_back(std::make_pair("TMark", param));
			// 		}
			// 	}

			// }
			// else if((pow(state.ballVel.x, 2) + pow(state.ballVel.y, 2)) > LOW_BALL_VELOCITY_THRES_SQ){
			// 	/*
					
			// 	if our bot can reach the ball first
			// 		1 go to ball () 
			// 		2 attack supoport
			// 		2 primary defenders
			// 		1 goalie
				

			// 	*/
			// }
			// else if(((pow(state.ballVel.x, 2) + pow(state.ballVel.y, 2)) < LOW_BALL_VELOCITY_THRES_SQ)
			// 	&& !nearest_opp_to_ball(dist_threshold)){
			// 	/*
					
				

			// 	*/
			}



			// computeMaxTacticTransits();
		}//constructer

		inline ~PDefence()
		{ }

		inline bool applicable(void) const
		{
			return true;
		}

		inline Result done(void) const
		{
			//done if the ball is not on our goalie side
			//
			return NOT_TERMINATED;
		}

		//some local functions
		bool nearest_opp_to_ball(float dist_threshold){
			//this returns true if the distance to the ball from the nearest opponent bot is
			// less than threshold value
			Vector2D<float> ball_pos(state.ballPos.x, state.ballPos.y);

			for(int idx = 0; idx != AwayTeam::SIZE; ++idx){
				Vector2D<float> away_bot(state.awayPos[idx].x, state.awayPos[idx].y);

				if(Vector2D<float>::dist(ball_pos, away_bot) < dist_threshold)
					return true;
		   }
		   return true;
		}

	    void updateParam()
	    {
	    	cout<<"here"<<endl;
	    	Tactic::Param param;
	    	float dist_threshold = 1000.0f;
	    	static int flag = 0;
	    	float x_threshold = 0;
			  // for(int i=0;i<HomeTeam::SIZE;i++)
			  //   roleList[i].clear();
	    	param.PositionP.x=CENTER_X;
	    	roleList[0].push_back(std::make_pair("TGoalie", param));
			  
			  param.MarkBotP.awayBotID = 0;
			  roleList[1].push_back(std::make_pair("TMark", param));

			  param.InterceptP.awayBotID = 1;
			  roleList[2].push_back(std::make_pair("TPosition", param));

			  param.PositionP.x= CENTER_X - GAP;
			  param.PositionP.y= CENTER_Y + GAP/2;
			  param.PositionP.finalSlope= -PI/4;
			  roleList[3].push_back(std::make_pair("TPosition", param));
			  roleList[3].push_back(std::make_pair("TStop", param));

			  param.PositionP.x= CENTER_X - GAP;
			  param.PositionP.y= CENTER_Y - GAP/2;
			  param.PositionP.finalSlope= PI/4;
			  roleList[4].push_back(std::make_pair("TPosition", param));
			  roleList[4].push_back(std::make_pair("TStop", param));

			  param.PositionP.x= CENTER_X + GAP;
			  param.PositionP.y= CENTER_Y + GAP/2;
			  param.PositionP.finalSlope= PI/4;
			  roleList[5].push_back(std::make_pair("TPosition", param));
			  roleList[5].push_back(std::make_pair("TStop", param));

  	    	//THREE MAJOR CASES
			if(((pow(state.ballVel.x, 2) + pow(state.ballVel.y, 2)) < LOW_BALL_VELOCITY_THRES_SQ)
				 && nearest_opp_to_ball(dist_threshold)){

				//POSITION THE PRIMARY DEFENDERS
				if(state.ballPos.x <= x_threshold){
					//since the open angle is less one primary defender is sufficient
					if(state.ballPos.y > 0)
						param.DefendARCP.side = 0;
					else 
						param.DefendARCP.side = 1;
				cout <<"inner primary defender tactic assigned \n";
				roleList[4].push_back(std::make_pair("TDefendARC", param));
				}
				else{
					//both the primary defenders would be required
					flag = 1;

					cout << "else condition primary defender assigned\n";
					param.DefendARCP.side = 0;
					roleList[4].push_back(std::make_pair("TDefendARC", param));

					param.DefendARCP.side = 1;
					roleList[5].push_back(std::make_pair("TDefendARC", param));
				}
			}
	    }

	};//class defence
}//namespace  strategy

#endif
