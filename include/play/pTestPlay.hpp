#ifndef PTEST_PLAY_HPP
#define PTEST_PLAY_HPP

#include <fstream>
#include <utility>
#include "play.hpp"
#include "krssg_ssl_msgs/BeliefState.h"
#include "tactics/tactic.h"
#include <ssl_common/config.h>
#include <ssl_common/geometry.hpp>

#define GAP 1000  //SELECT(1000,100)

namespace Strategy
{
  class PTestPlay : public Play
  {
  public:
    inline PTestPlay(const krssg_ssl_msgs::BeliefState& state) 
      : Play(state)
    {
      name = "TestPlay";

      assert(HomeTeam::SIZE == 6); // TestPlay is applicable for a team of 3 robots only
      printf("in Test Play \n");
      Tactic::Param param;

     
      param.PassToPointP.x=OPP_GOAL_X;
      param.PassToPointP.y=CENTER_Y;
      roleList[0].push_back(std::make_pair("TPassToPoint", param));
      roleList[0].push_back(std::make_pair("TStop", param));

      param.ReceiveP.x=OPP_GOAL_X;
      param.ReceiveP.y=CENTER_Y;
      roleList[1].push_back(std::make_pair("TReceive", param));
      roleList[1].push_back(std::make_pair("TStop", param));

      param.PositionP.x= CENTER_X;
      param.PositionP.y= CENTER_Y - 2*GAP;
      param.PositionP.finalSlope= PI/2;
      roleList[2].push_back(std::make_pair("TPosition", param));
      roleList[2].push_back(std::make_pair("TStop", param));

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

      param.PositionP.x= CENTER_X - GAP;
      param.PositionP.y= CENTER_Y - GAP/2;
      param.PositionP.finalSlope= PI/4;
      roleList[5].push_back(std::make_pair("TPosition", param));
      roleList[5].push_back(std::make_pair("TStop", param));

      param.PositionP.x= CENTER_X - GAP;
      param.PositionP.y= CENTER_Y - GAP/2;
      param.PositionP.finalSlope= PI/4;
      roleList[5].push_back(std::make_pair("TPosition", param));
      roleList[5].push_back(std::make_pair("TStop", param));

      computeMaxTacticTransits();
    }

    inline ~PTestPlay()
    { }

    inline bool applicable(void) const
    {
      // printf("Set position is applicable\n");
      // TODO make it more sophisticated
      return false;
    }

    inline Result done(void) const
    {
      // TODO make it more sophisticated and also use the timeout info to determine if the play has terminated
      // printf("Done condition not finalised\n");
      return NOT_TERMINATED;
    }
  }; // class PTestPlay
} // namespace Strategy

#endif // PTEST_PLAY_HPP
