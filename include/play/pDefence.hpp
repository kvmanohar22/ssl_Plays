#ifndef PDEFENCE_HPP
#define PDEFENCE_HPP

#include "ros/ros.h"
#include <utility>
#include "play.hpp"
#include "krssg_ssl_msgs/BeliefState.h"
#include "tactics/tactic.h"
#include <ssl_common/config.h>
#include <ssl_common/geometry.hpp>

/*
	TODO:
		#1 primary defenders going back and hitting the ball back
		#2 
		#3 
*/



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
			int roles_assigned = 0;
			float alpha = 1.0;
			float LOW_BALL_VELOCITY_THRES = alpha * LOW_BALL_VELOCITY_THRES; 
			float LOW_BALL_VELOCITY_THRES_SQ = LOW_BALL_VELOCITY_THRES * LOW_BALL_VELOCITY_THRES; 
			float maxx_threshold = HALF_FIELD_MAXX/2.0f; //this is to determine if only one primary defender is sufficient
			float minx_threshold = -HALF_FIELD_MAXX/2.0f;
			float dist_threshold = 1000.0f; //check nearest_opp_to_ball() function
			static int flag = 0; //this corresponds only one primary defender being used

	        roleList[0].push_back(std::make_pair("TGoalie", param));
	        roles_assigned += 1;

/*			roleList[1].push_back(std::make_pair("TDefendARC_left", param));
			roleList[2].push_back(std::make_pair("TDefendARC_right", param)); 

		    param.MarkBotP.awayBotID = 0;
		    roleList[3].push_back(std::make_pair("TMark", param));

		    param.PositionP.x= CENTER_X;
		    param.PositionP.y= CENTER_Y;
		    param.PositionP.finalSlope= PI/4;
		    roleList[4].push_back(std::make_pair("TPosition", param));
		    roleList[4].push_back(std::make_pair("TStop", param));			       	

		    param.PositionP.x= CENTER_X + GAP;
		    param.PositionP.y= CENTER_Y + GAP/2;
		    param.PositionP.finalSlope= PI/4;
		    roleList[5].push_back(std::make_pair("TPosition", param));
		    roleList[5].push_back(std::make_pair("TStop", param));			       	
*/
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

			// ***************** CASE 1 ******************** //
			if(((pow(state.ballVel.x, 2) + pow(state.ballVel.y, 2)) < LOW_BALL_VELOCITY_THRES_SQ)
				 && nearest_opp_to_ball(dist_threshold)){

				//POSITION THE PRIMARY DEFENDERS
				if(state.ballPos.x >= maxx_threshold){
					//since the open angle is less one primary defender is sufficient
					roleList[1].push_back(std::make_pair("TDefendARC_left", param));
					roles_assigned += 1;
				}
				else if(state.ballPos.x <= minx_threshold){
					roleList[1].push_back(std::make_pair("TDefendARC_right", param));
					roles_assigned += 1;	
				}
				else{
					//both the primary defenders would be required
					flag = 1;

					roleList[1].push_back(std::make_pair("TDefendARC_left", param));
					roles_assigned += 1;
					roleList[2].push_back(std::make_pair("TDefendARC_right", param));
					roles_assigned += 1;
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

				for(int idx = 0; idx != AwayTeam::SIZE; ++idx){
					//make sure that we donot include the bot which is nearest to the ball
					if(state.awayPos[idx].x < 0 && idx != nearest_bot){
						++count;

						//note down the opp. botIDs
						opp_in_our_half.push_back(idx);
					}
				}

				if(count == 1){
					/*
						GO FOR 
							one marker 
							one intercept
							one attacker in the opponent's goalie side
					*/
					param.MarkBotP.awayBotID = opp_in_our_half[0];
					roleList[3].push_back(std::make_pair("TMark", param));
					roles_assigned += 1;

					param.InterceptP.awayBotID = opp_in_our_half[0];
					param.InterceptP.where = 1;
					roleList[4].push_back(std::make_pair("TIntercept", param));
					roles_assigned += 1;

					roleList[5].push_back(std::make_pair("TKickToGoal", param));
					roles_assigned += 1;
				}
				else if(count == 2){
					/*
					 	we'll mark both the opponents on our half
					 	and if only one primary defender is used,
					 	we'll use him to intercept the goal from the opponent 
					 	who is most likely to score the goal
					*/
					param.MarkBotP.awayBotID = opp_in_our_half[0];
					roleList[3].push_back(std::make_pair("TMark", param));
					roles_assigned += 1;

					param.MarkBotP.awayBotID = opp_in_our_half[1];
					roleList[4].push_back(std::make_pair("TMark", param));
					roles_assigned += 1;

					roleList[5].push_back(std::make_pair("TKickToGoal", param));
					roles_assigned += 1;
					
					if(!flag) {
					   //intercept the goal of the bot which is more likely to score the goal
					   //this tactic is assigned to the extra primary defender
					   int intercept_idx = 0;
					   param.InterceptP.awayBotID = opp_in_our_half[intercept_idx];
					   param.InterceptP.where = 1;
					   roleList[2].push_back(std::make_pair("TIntercept", param));
					   roles_assigned += 1;
					}
				}
				else {
					/*
						we'll have: BASICALLY AN ATTACK PLAY
						two primary defenders
						one goalie

						TODO :: DECIDE THE COMBINATION HERE
						one attack supporter
						one kicker 
						one recieve
					*/
				    param.AttackSupportP.id=state.our_bot_closest_to_ball;
				    roleList[3].push_back(std::make_pair("TAttackSupport1_Center", param));
				    roles_assigned += 1;

				    roleList[4].push_back(std::make_pair("TKickToGoal", param));
				    roles_assigned += 1;

				    param.AttackSupportP.id=state.our_bot_closest_to_ball;
				    roleList[5].push_back(std::make_pair("TAttackSupport1_Wing", param));
				    roles_assigned += 1;

					if(!flag){
					    /*
						   use the other primary defender if not used
					    */

					    if(state.ballPos.y > 0){
					   		roleList[2].push_back(std::make_pair("TDefendARC_right", param));
					   		roles_assigned += 1;
					    }
					    else{
					   		roleList[2].push_back(std::make_pair("TDefendARC_left", param));
					    	roles_assigned += 1;
					    }
					}
				}
				roleList[1].push_back(std::make_pair("TDefendARC_left", param));
				roleList[2].push_back(std::make_pair("TDefendARC_right", param)); 

		    	param.MarkBotP.awayBotID = 0;
		    	roleList[3].push_back(std::make_pair("TMark", param));		
	
	
			} // ************************** CASE 2 ************************** //
			else if((pow(state.ballVel.x, 2) + pow(state.ballVel.y, 2)) > LOW_BALL_VELOCITY_THRES_SQ){
				
				/*					
					if our bot can reach the ball first
						1 go to ball () 
						2 attack supoport
						2 primary defenders
						1 goalie
					
	         	*/
				// param.DefendARCP.side = 0;
				roleList[1].push_back(std::make_pair("TDefendARC_left", param));
				roles_assigned += 1;

				// param.DefendARCP.side = 1;
				roleList[2].push_back(std::make_pair("TDefendARC_right", param));
				roles_assigned += 1;

				/*
					we need 
						one reciever
						one go to ball
					 	one(two) attack supporter(s)
				*/
				param.GoToBallP.intercept =  true;
				roleList[3].push_back(std::make_pair("TGoToBall", param));
				roles_assigned += 1;

			    param.AttackSupportP.id=state.our_bot_closest_to_ball;
			    roleList[4].push_back(std::make_pair("TAttackSupport1_Center", param));
			    roles_assigned += 1;

			    param.AttackSupportP.id=state.our_bot_closest_to_ball;
			    roleList[5].push_back(std::make_pair("TAttackSupport1_Wing", param));
			    roles_assigned += 1;

			    assert(roles_assigned == 6);
			}// ************************** CASE 3 ************************** //
			else if(((pow(state.ballVel.x, 2) + pow(state.ballVel.y, 2)) < LOW_BALL_VELOCITY_THRES_SQ)
				&& !nearest_opp_to_ball(dist_threshold)){
				
				/*
					one goalie + 2 primary defenders
					one reciever
					one kick to goal
					one attack support
				*/

				// param.DefendARCP.side = 0;
				roleList[1].push_back(std::make_pair("TDefendARC_left", param));
			    roles_assigned += 1;

				// param.DefendARCP.side = 1;
				roleList[2].push_back(std::make_pair("TDefendARC_right", param));
				roles_assigned += 1;

				roleList[3].push_back(std::make_pair("TKickToGoal", param));
				roles_assigned += 1;

				/*
					TODO: Add one reciever here
				*/
			    param.AttackSupportP.id=state.our_bot_closest_to_ball;
			    roleList[4].push_back(std::make_pair("TAttackSupport1_Center", param));
			    roles_assigned += 1;

			    param.AttackSupportP.id=state.our_bot_closest_to_ball;
			    roleList[5].push_back(std::make_pair("TAttackSupport1_Wing", param));
			    roles_assigned += 1;

			    assert(roles_assigned == 6);
			}

			computeMaxTacticTransits();
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

		bool nearest_opp_to_ball(float dist_threshold){
        /*
       		this returns true if the distance to the ball from the nearest opponent bot is
			less than threshold value
		*/
			Vector2D<float> ball_pos(state.ballPos.x, state.ballPos.y);

			for(int idx = 0; idx != AwayTeam::SIZE; ++idx){
				Vector2D<float> away_bot(state.awayPos[idx].x, state.awayPos[idx].y);

				if(Vector2D<float>::dist(ball_pos, away_bot) < dist_threshold)
					return true;
		    }
		    return false;
		}

	    void updateParam() {

		    Tactic::Param param;
			
			//some local variables
		    int roles_assigned = 0;
			float alpha = 1.0;
			float LOW_BALL_VELOCITY_THRES = alpha * LOW_BALL_VELOCITY_THRES; 
			float LOW_BALL_VELOCITY_THRES_SQ = LOW_BALL_VELOCITY_THRES * LOW_BALL_VELOCITY_THRES; 
			float maxx_threshold = HALF_FIELD_MAXX/2.0f; //this is to determine if only one primary defender is sufficient
			float minx_threshold = -HALF_FIELD_MAXX/2.0f;
			float dist_threshold = 1000.0f; //check nearest_opp_to_ball() function
			static int flag = 0; //this corresponds to only one primary defender being used

	        roleList[0].push_back(std::make_pair("TGoalie", param));
	        roles_assigned += 1;
/*			roleList[1].push_back(std::make_pair("TDefendARC_left", param));
			roleList[2].push_back(std::make_pair("TDefendARC_right", param)); 

		    param.MarkBotP.awayBotID = 0;
		    roleList[3].push_back(std::make_pair("TMark", param));

		    // param.InterceptP.awayBotID = 1;
		    // param.InterceptP.where = 0;
		    // roleList[4].push_back(std::make_pair("TIntercept", param));
		      

		    param.PositionP.x= CENTER_X;
		    param.PositionP.y= CENTER_Y;
		    param.PositionP.finalSlope= PI/4;
		    roleList[4].push_back(std::make_pair("TPosition", param));
		    roleList[4].push_back(std::make_pair("TStop", param));			       	

		    param.PositionP.x= CENTER_X + GAP;
		    param.PositionP.y= CENTER_Y + GAP/2;
		    param.PositionP.finalSlope= PI/4;
		    roleList[5].push_back(std::make_pair("TPosition", param));
		    roleList[5].push_back(std::make_pair("TStop", param));			       	
*/

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

			// ***************** CASE 1 ******************** //
			if(((pow(state.ballVel.x, 2) + pow(state.ballVel.y, 2)) < LOW_BALL_VELOCITY_THRES_SQ)
				 && nearest_opp_to_ball(dist_threshold)){

				//POSITION THE PRIMARY DEFENDERS
				if(state.ballPos.x >= maxx_threshold){
					//since the open angle is less one primary defender is sufficient
					roleList[1].push_back(std::make_pair("TDefendARC_left", param));
					roles_assigned += 1;
				}
				else if(state.ballPos.x <= minx_threshold){
					roleList[1].push_back(std::make_pair("TDefendARC_right", param));
					roles_assigned += 1;	
				}
				else{
					//both the primary defenders would be required
					flag = 1;

					roleList[1].push_back(std::make_pair("TDefendARC_left", param));
					roles_assigned += 1;

					// param.DefendARCP.side = 1;
					roleList[2].push_back(std::make_pair("TDefendARC_right", param));
					roles_assigned += 1;
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

				for(int idx = 0; idx != AwayTeam::SIZE; ++idx){
					//make sure that we donot include the bot which is nearest to the ball
					if(state.awayPos[idx].x < 0 && idx != nearest_bot){
						++count;

						//note down the opp. botIDs
						opp_in_our_half.push_back(idx);
					}
				}

				if(count == 1){
					/*
						GO FOR 
							one marker 
							one intercept
							one attacker in the opponent's goalie side
					*/
					param.MarkBotP.awayBotID = opp_in_our_half[0];
					roleList[3].push_back(std::make_pair("TMark", param));
					roles_assigned += 1;

					param.InterceptP.awayBotID = opp_in_our_half[0];
					param.InterceptP.where = 1;
					roleList[4].push_back(std::make_pair("TIntercept", param));
					roles_assigned += 1;

					roleList[5].push_back(std::make_pair("TKickToGoal", param));
					roles_assigned += 1;
				}
				else if(count == 2){
					/*
					 	we'll mark both the opponents on our half
					 	and if only one primary defender is used,
					 	we'll use him to intercept the goal from the opponent 
					 	who is most likely to score the goal
					*/
					param.MarkBotP.awayBotID = opp_in_our_half[0];
					roleList[3].push_back(std::make_pair("TMark", param));
					roles_assigned += 1;

					param.MarkBotP.awayBotID = opp_in_our_half[1];
					roleList[4].push_back(std::make_pair("TMark", param));
					roles_assigned += 1;

					roleList[5].push_back(std::make_pair("TKickToGoal", param));
					roles_assigned += 1;

					if(!flag) {
					   //intercept the goal of the bot which is more likely to score the goal
					   //this tactic is assigned to the extra primary defender
					   int intercept_idx = 0;
					   param.InterceptP.awayBotID = opp_in_our_half[intercept_idx];
					   param.InterceptP.where = 1;
					   roleList[2].push_back(std::make_pair("TIntercept", param));
					   roles_assigned += 1;
					}
				}
				else {
					/*
						we'll have: BASICALLY AN ATTACK PLAY
						two primary defenders
						one goalie

						TODO :: DECIDE THE COMBINATION HERE
						one attack supporter
						one kicker 
						one recieve
					*/
				    param.AttackSupportP.id=state.our_bot_closest_to_ball;
				    roleList[3].push_back(std::make_pair("TAttackSupport1_Center", param));
				    roles_assigned += 1;

				    roleList[4].push_back(std::make_pair("TKickToGoal", param));
				    roles_assigned += 1;

				    param.AttackSupportP.id=state.our_bot_closest_to_ball;
				    roleList[5].push_back(std::make_pair("TAttackSupport1_Wing", param));
				    roles_assigned += 1;

					if(!flag){
					  /*
						use the other primary defender if not used
					  */
					   if(state.ballPos.y > 0){
		        	   	    roleList[2].push_back(std::make_pair("TDefendARC_right", param));
					   		roles_assigned += 1;
					   }
					   	else{
						   	roleList[2].push_back(std::make_pair("TDefendARC_left", param));
					   		roles_assigned += 1;
					   	}
					}
				}

			} // ************************** CASE 2 ************************** //
			else if((pow(state.ballVel.x, 2) + pow(state.ballVel.y, 2)) > LOW_BALL_VELOCITY_THRES_SQ){
				
				/*					
					if our bot can reach the ball first
						1 go to ball () 
						2 attack supoport
						2 primary defenders
						1 goalie
					
	         	*/

				roleList[1].push_back(std::make_pair("TDefendARC_left", param));
				roleList[2].push_back(std::make_pair("TDefendARC_right", param));
					
				/*
					we need 
						one reciever
						one go to ball
					 	one(two) attack supporter(s)
				*/
				param.GoToBallP.intercept =  true;
				roleList[3].push_back(std::make_pair("TGoToBall", param));

			    param.AttackSupportP.id=state.our_bot_closest_to_ball;
			    roleList[4].push_back(std::make_pair("TAttackSupport1_Center", param));

			    param.AttackSupportP.id=state.our_bot_closest_to_ball;
			    roleList[5].push_back(std::make_pair("TAttackSupport1_Wing", param));

			}// ************************** CASE 3 ************************** //
			else if(((pow(state.ballVel.x, 2) + pow(state.ballVel.y, 2)) < LOW_BALL_VELOCITY_THRES_SQ)
				&& !nearest_opp_to_ball(dist_threshold)){
				
				/*
					one goalie + 2 primary defenders
					one reciever
					one kick to goal
					one attack support
				*/

				roleList[1].push_back(std::make_pair("TDefendARC_left", param));
				roleList[2].push_back(std::make_pair("TDefendARC_right", param));

				roleList[3].push_back(std::make_pair("TKickToGoal", param));

				/*
					TODO: Add one reciever here
				*/
			    param.AttackSupportP.id=state.our_bot_closest_to_ball;
			    roleList[4].push_back(std::make_pair("TAttackSupport1_Center", param));

			    param.AttackSupportP.id=state.our_bot_closest_to_ball;
			    roleList[5].push_back(std::make_pair("TAttackSupport1_Wing", param));
			}

	    } //update param

	};//class defence
}//namespace  strategy

#endif