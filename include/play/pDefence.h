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
			float dist_threshold;//check nearest_opp_to_ball() function
			static int flag = 0; //this corresponds only one primary defender being used

			/*
				TODO:
				#1 calculate the x_threshold value
				#2 calculate the dist_threshold value
				#3 intercept_idx @ line no 124
			*/

 
			//goalie is common for all the cases
			roleList[0].push_back(std::make_pair("TGoalie", param));
			roleList[0].push_back(std::make_pair("TStop", param));

			/*
				there are 3 possible cases
				#1 velocity of ball is less than a threshold value
				 	AND
				 	dist(nearest opp to ball, ball) < threshold value

				#2 velocity of ball is less than a threshold value
				 	AND
				 	dist(nearest opp to ball, ball) > threshold value

				#3 velocity of ball is greater than a threshold value

			*/

			//THREE MAJOR CASES
			if(((pow(state.balVel.x, 2) + pow(state.balVel.y, 2)) < LOW_BALL_VELOCITY_THRES_SQ)
				 && nearest_opp_to_ball()){

				//POSITION THE PRIMARY DEFENDERS
				if(state.ballPos.x <= x_threshold){
					//since the open angle is less one primary defender is sufficient
					if(state.ballPos.y > 0)
						param.DefendARCP.side = 0;
					else 
						para.DefendARCP.side = 1;

				roleList[1].push_back(std::make_pair("TDefendARC", param));
				}
				else{
					//both the primary defenders would be required
					flag = 1;

					param.DefendARCP.side = 0;
					roleList[1].push_back(std::make_pair("TDefendARC", param));

					param.DefendARCP.side = 1;
					roleList[2].push_back(std::make_pair("TDefendARC", param));
				}

				//POSITION THE SECONDARY DEFENDERS
				float nearest_dist = 1000000.0f;
				int nearest_bot = -1;
				Vector2D<float> ball_pos(state.ballPos.x, state.ballPos.y);
				for(int idx = 0; idx != AwayTeam::SIZE; ++idx){
					Vector2D<float> away_bot(state.awayPos[idx].x, state.awayPos[idx].y);

					if(Vector2D<float>::dist(ball_pos, away_bot) < nearest_dist){
						nearest_bot = idx;
						nearest_dist = Vector2D<float>::dist(ball_pos, away_bot);
					}
				}

				//count the number of opponent bots on our side
				int count = 0;
				std::vector<int> opp_in_our_half;

				for(idx = 0; idx != AwayTeam::SIZE; ++idx){
					//make sure that we donot include the bot which is nearest to the ball
					if(state.awayPos[idx].x < 0 && idx != nearest_bot){
						++count;

						//note down the opp. botIDs
						opp_in_our_half.push_back(idx);
					}
				}

				if(count == 1){
					//go for one marker and one intercept
					param.MarkBotP.awayBotID = opp_in_our_half[0];
					roleList[3].push_back(std::make_pair("TMark", param));

					//param.InterceptP.awayBotID = opp_in_our_half[0];
					//param.InterceptP.where = 1;
					//roleList[4].push_back(std::make_pair("TIntercept", param));
				}
				else if(count == 2){
					   param.MarkBotP.awayBotID = opp_in_our_half[0];
					   roleList[3].push_back(std::make_pair("TMark", param));

					   param.MarkBotP.awayBotID = opp_in_our_half[1];
					   roleList[4].push_back(std::make_pair("TMark", param));

					if(!flag){
					   //intercept the goal of the bot which is more likely to score the goal
					   //this tactic is assigned to the extra primary defender
					   int intercept_idx = 0;
					   //param.InterceptP.awayBotID = opp_in_our_half[intercept_idx];
					   //param.InterceptP.where = 1;
					   //roleList[2].push_back(std::make_pair("TIntercept", param));
					}
				}
				else {
						param.MarkBotP.awayBotID = opp_in_our_half[0];
					   roleList[3].push_back(std::make_pair("TMark", param));

					   param.MarkBotP.awayBotID = opp_in_our_half[1];
					   roleList[4].push_back(std::make_pair("TMark", param));

					if(!flag){
						//we'll have to even mark the other opponent instead of intercepting
						param.MarkBotP.awayBotID = opp_in_our_half[2];
					   roleList[2].push_back(std::make_pair("TMark", param));
					}
				}

			}
			else if((pow(state.balVel.x, 2) + pow(state.balVel.y, 2)) > LOW_BALL_VELOCITY_THRES_SQ){
				
			}
			else if(((pow(state.balVel.x, 2) + pow(state.balVel.y, 2)) < LOW_BALL_VELOCITY_THRES_SQ)
				&& !nearest_opp_to_ball()){
				
			}




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
		}

		//some local functions
		bool nearest_opp_to_ball(){
			//this returns true if the distance to the ball from the nearest opponent bot is
			// less than threshold value
			Vector2D<float> ball_pos(state.ballPos.x, state.ballPos.y);

			for(int idx = 0; idx != AwayTeam::SIZE; ++idx){
				Vector2D<float> away_bot(state.awayPos[idx].x, state.awayPos[idx].y);

				if(Vector2D<float>::dist(ball_pos, away_bot) < dist_threshold)
					return true;
		   }
		   return false;
		}

	};//class defence
}//namespace  strategy

#endif