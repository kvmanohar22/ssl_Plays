#include "pPassTest.hpp"
#include <utility>
#include "play.hpp"
#include "krssg_ssl_msgs/BeliefState.h"
#include "tactics/tactic.h"
#include <ssl_common/config.h>
#include <ssl_common/geometry.hpp>
#include <math.h>

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
   //      int passrer_id,receiver_id=0,marker_id;
   //      float maxProb=0;
   //      while(receiver_id<6)
   //        {
   //        	marker_id=findMarker(receiver_id);

   //          Vector2D<int> passPoint,Point1,Point2;
   //          bool intersecting1=true,intersecting2=true;
   //          int x1,x2,y1,y2,x3,y3,x4,y4;
            
   //          //#############this is for intersection of ball/receiver conic and receiver marker line
   //          float c1,a1=c1=(u*u-w*w);
   //          float b1=-2*(state.homePos[passrer_id].x*u*u-state.homePos[receiver_id].x*w*w);
   //          float d1=-2*(state.homePos[passrer_id].y*u*u-state.homePos[receiver_id].y*w*w);
   //          float e1=(state.homePos[passrer_id].x*state.homePos[passrer_id].x+state.homePos[passrer_id].y*state.homePos[passrer_id].y)*u*u-(state.homePos[receiver_id].x*state.homePos[receiver_id].x+state.homePos[receiver_id].y*state.homePos[receiver_id].y)*w*w;
           
   //          float a2=-2*(state.awayPos[marker_id].x*u*u-state.homePos[receiver_id].x*v*v);
   //          float b2=-2*(state.awayPos[marker_id].y*u*u-state.homePos[receiver_id].y*v*v);
   //          float c2=(state.awayPos[marker_id].x*state.awayPos[marker_id].x+state.awayPos[marker_id].y*state.awayPos[marker_id].y)*u*u-(state.homePos[receiver_id].x*state.homePos[receiver_id].x+state.homePos[receiver_id].y*state.homePos[receiver_id].y)*v*v;
            
   //          if((b1*pow(a2,2) + a1*pow(b2,2))!=0 || a2!=0)
   //          {
   //             x1=-(c2 - (b2*(pow(a2,2)*d1 + a2*pow((pow(a2,2)*pow(d1,2) - 4*b1*e1*pow(a2,2) - 2*a2*b2*c1*d1 + 4*b1*a2*c1*c2 + pow(b2,2)*pow(c1,2) - 4*a1*e1*pow(b2,2) + 4*a1*b2*c2*d1 - 4*a1*b1*pow(c2,2)),(1/2)) + 2*a1*b2*c2 - a2*b2*c1))/(2*(b1*pow(a2,2) + a1*pow(b2,2))))/a2 ;
   //             y1=-(pow(a2,2)*d1 + a2*pow((pow(a2,2)*pow(d1,2) - 4*b1*e1*pow(a2,2) - 2*a2*b2*c1*d1 + 4*b1*a2*c1*c2 + pow(b2,2)*pow(c1,2) - 4*a1*e1*pow(b2,2) + 4*a1*b2*c2*d1 - 4*a1*b1*pow(c2,2)),(1/2)) + 2*a1*b2*c2 - a2*b2*c1)/(2*(b1*pow(a2,2) + a1*pow(b2,2))) ;
              
   //             x2=-(c2 - (b2*(pow(a2,2)*d1 - a2*pow((pow(a2,2)*pow(d1,2) - 4*b1*e1*pow(a2,2) - 2*a2*b2*c1*d1 + 4*b1*a2*c1*c2 + pow(b2,2)*pow(c1,2) - 4*a1*e1*pow(b2,2) + 4*a1*b2*c2*d1 - 4*a1*b1*pow(c2,2)),(1/2)) + 2*a1*b2*c2 - a2*b2*c1))/(2*(b1*pow(a2,2) + a1*pow(b2,2))))/a2 ;
   //             y2=-(pow(a2,2)*d1 - a2*pow((pow(a2,2)*pow(d1,2) - 4*b1*e1*pow(a2,2) - 2*a2*b2*c1*d1 + 4*b1*a2*c1*c2 + pow(b2,2)*pow(c1,2) - 4*a1*e1*pow(b2,2) + 4*a1*b2*c2*d1 - 4*a1*b1*pow(c2,2)),(1/2)) + 2*a1*b2*c2 - a2*b2*c1)/(2*(b1*pow(a2,2) + a1*pow(b2,2))) ;
   //          }
   //          else 
   //          {
   //            intersecting1=false;
   //          }
            
   //           //#############this is for intersection of ball/marker conic and receiver marker line
   //            a1=c1=v*v-w*w ;
   //            b1=-2*(state.homePos[passrer_id].x*v*v-state.awayPos[marker_id].x*w*w);
   //            d1=-2*(state.homePos[passrer_id].y*v*v-state.awayPos[marker_id].y*w*w);
   //            e1=(state.homePos[passrer_id].x*state.homePos[passrer_id].x+state.homePos[passrer_id].y*state.homePos[passrer_id].y)*v*v-(state.awayPos[marker_id].x*state.awayPos[marker_id].x+state.homePos[receiver_id].y*state.homePos[receiver_id].y)*w*w;
            
   //          if((b1*pow(a2,2) + a1*pow(b2,2))!=0 || a2!=0)
   //          {
   //             x3=-(c2 - (b2*(pow(a2,2)*d1 + a2*pow((pow(a2,2)*pow(d1,2) - 4*b1*e1*pow(a2,2) - 2*a2*b2*c1*d1 + 4*b1*a2*c1*c2 + pow(b2,2)*pow(c1,2) - 4*a1*e1*pow(b2,2) + 4*a1*b2*c2*d1 - 4*a1*b1*pow(c2,2)),(1/2)) + 2*a1*b2*c2 - a2*b2*c1))/(2*(b1*pow(a2,2) + a1*pow(b2,2))))/a2 ;
   //             y3=-(pow(a2,2)*d1 + a2*pow((pow(a2,2)*pow(d1,2) - 4*b1*e1*pow(a2,2) - 2*a2*b2*c1*d1 + 4*b1*a2*c1*c2 + pow(b2,2)*pow(c1,2) - 4*a1*e1*pow(b2,2) + 4*a1*b2*c2*d1 - 4*a1*b1*pow(c2,2)),(1/2)) + 2*a1*b2*c2 - a2*b2*c1)/(2*(b1*pow(a2,2) + a1*pow(b2,2))) ;
              
   //             x4=-(c2 - (b2*(pow(a2,2)*d1 - a2*pow((pow(a2,2)*pow(d1,2) - 4*b1*e1*pow(a2,2) - 2*a2*b2*c1*d1 + 4*b1*a2*c1*c2 + pow(b2,2)*pow(c1,2) - 4*a1*e1*pow(b2,2) + 4*a1*b2*c2*d1 - 4*a1*b1*pow(c2,2)),(1/2)) + 2*a1*b2*c2 - a2*b2*c1))/(2*(b1*pow(a2,2) + a1*pow(b2,2))))/a2 ;
   //             y4=-(pow(a2,2)*d1 - a2*pow((pow(a2,2)*pow(d1,2) - 4*b1*e1*pow(a2,2) - 2*a2*b2*c1*d1 + 4*b1*a2*c1*c2 + pow(b2,2)*pow(c1,2) - 4*a1*e1*pow(b2,2) + 4*a1*b2*c2*d1 - 4*a1*b1*pow(c2,2)),(1/2)) + 2*a1*b2*c2 - a2*b2*c1)/(2*(b1*pow(a2,2) + a1*pow(b2,2))) ;
   //          }
   //          else 
   //          {
   //            intersecting2=false;
   //          }

   //          //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!selecting point!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   //          float max_theta=0,maxX,maxY;
   //          Vector2D<int> GoalPoint(OPP_GOAL_X,OPP_GOAL_Y);
   //          Vector2D<int> GoalPole;
            
   //          if(Vector2D<int>::dist(Vector2D<int>(state.awayPos[state.opp_goalie].x,state.awayPos[state.opp_goalie].y),Vector2D<int>(OPP_GOAL_X,OPP_GOAL_MAXY)) > Vector2D<int>::dist(Vector2D<int>(state.awayPos[state.opp_goalie].x,state.awayPos[state.opp_goalie].y),Vector2D<int>(OPP_GOAL_X,OPP_GOAL_MINY)))
   //          {
   //          	GoalPole.x=OPP_GOAL_X;
   //          	GoalPole.y=OPP_GOAL_MAXY;
   //          }
   //          else
   //          {
   //          	GoalPole.x=OPP_GOAL_X;
   //          	GoalPole.y=OPP_GOAL_MAXY;
   //          }//finding the goal pole further from the goalkeeper

   //          if(intersecting2==true && intersecting1==true && ((x1<x3<x2 && y1<y3<y2)||(x2<x3<x1 && y2<y3<y1)||(x1<x4<x2 && y1<y4<y2)||(x2<x4<x1 && y2<y4<y1))) // case 1 : both ellipses interect and one intersectin pt of marker's ellipse lies between those of the receiver's
   //          {
   //            if((x1<x3<x2 && y1<y3<y2)||(x2<x3<x1 && y2<y3<y1)) //X3 lies between X1 and X2 
   //            {
   //              if((x3<x2<x4 && y3<y2<y4)||(x4<x2<x3 && y4<y2<y3)) //X2 lies between X3 and X4
   //              {
   //                Point1.x=x2;
   //                Point1.y=y2;
   //                Point2.x=x4;
   //                Point2.y=y4;
                  
   //                if(Point1.x<Point2.x)
   //                {
   //                    for (int x= Point1.x; x < Point2.x; ++x)
   //                    {
   //                      if(b2!=0)
   //                      {
   //                         int y=-(a2*x+c2)/b2;
   //                        if(checkPointInField( Vector2D<int> (x,y)))
   //                        {
   //                          float theta=fabs(normalizeAngle(Vector2D<int>::angle(GoalPoint,Vector2D<int> (x,y))-Vector2D<int>::angle(Vector2D<int>(state.awayPos[state.opp_goalie].x,state.awayPos[state.opp_goalie].y),GoalPole)));
   //                          if(theta>max_theta) 
   //                            {
   //                              max_theta=theta;
   //                              maxX=x;
   //                              maxY=y;
   //                            }  
   //                        }
                          
   //                      }
                       
   //                    }
   //                }
   //                else
   //                {
   //                    for (int x= Point2.x; x < Point1.x; ++x)
   //                    {
   //                      if(b2!=0)
   //                      {
   //                         int y=-(a2*x+c2)/b2;
   //                        if(checkPointInField( Vector2D<int> (x,y)))
   //                        {
   //                           float theta=fabs(Vector2D<int>::angle(GoalPoint,Vector2D<int> (x,y)));
   //                          if(theta>max_theta) 
   //                            {
   //                              max_theta=theta;
   //                              maxX=x;
   //                              maxY=y;
   //                            }

   //                        }
                         
   //                      }
                       
   //                    }
   //                }

   //                passPoint.x=maxX;
   //                passPoint.y=maxY;
   //              }
   //              else if((x3<x1<x4 && y3<y1<y4)||(x4<x1<x3 && y4<y1<y3)) //X1 lies between X3 and X4
   //              {
   //                Point1.x=x1;
   //                Point1.y=y1;
   //                Point2.x=x4;
   //                Point2.y=y4;

   //                if(Point1.x<Point2.x)
   //                {
   //                    for (int x= Point1.x; x < Point2.x; ++x)
   //                    {
   //                      if(b2!=0)
   //                      {
   //                        int y=-(a2*x+c2)/b2;
   //                        if(checkPointInField( Vector2D<int> (x,y)))
   //                        {
   //                           float theta=fabs(Vector2D<int>::angle(GoalPoint,Vector2D<int> (x,y)));
   //                          if(theta>max_theta) 
   //                            {
   //                              max_theta=theta;
   //                              maxX=x;
   //                              maxY=y;
   //                            }

   //                        }
                         
   //                      }
   //                    }
   //                }
   //                 else
   //                {
   //                    for (int x= Point2.x; x < Point1.x; ++x)
   //                    {
   //                      if(b2!=0)
   //                      {
   //                         int y=-(a2*x+c2)/b2;
   //                        if(checkPointInField( Vector2D<int> (x,y)))
   //                        {
   //                            float theta=fabs(Vector2D<int>::angle(GoalPoint,Vector2D<int> (x,y)));
   //                          if(theta>max_theta) 
   //                            {
   //                              max_theta=theta;
   //                              maxX=x;
   //                              maxY=y;
   //                            }
   //                        }
                          
   //                      }
                       
   //                    }
   //                }

   //                passPoint.x=maxX;
   //                passPoint.y=maxY;
   //              }
                
   //            }
   //            else if((x1<x4<x2 && y1<y4<y2)||(x2<x4<x1 && y2<y4<y1)) //X4 lies between X1 and X2 
   //            {
   //               if((x3<x2<x4 && y3<y2<y4)||(x4<x2<x3 && y4<y2<y3)) //X2 lies between X3 and X4
   //              {
   //                Point1.x=x2;
   //                Point1.y=y2;
   //                Point2.x=x3;
   //                Point2.y=y3;

   //                if(Point1.x<Point2.x)
   //                {
   //                    for (int x= Point1.x; x < Point2.x; ++x)
   //                    {
   //                      if(b2!=0)
   //                     {
   //                      int y=-(a2*x+c2)/b2;
   //                      if(checkPointInField( Vector2D<int> (x,y)))
   //                      {
   //                        float theta=fabs(Vector2D<int>::angle(GoalPoint,Vector2D<int> (x,y)));
   //                        if(theta>max_theta) 
   //                          {
   //                            max_theta=theta;
   //                            maxX=x;
   //                            maxY=y;
   //                          }
   //                      }
                        
   //                     }
   //                    }
   //                }
   //                else
   //                {
   //                    for (int x= Point2.x; x < Point1.x; ++x)
   //                    {
   //                      if(b2!=0)
   //                      {
   //                         int y=-(a2*x+c2)/b2;
   //                        if(checkPointInField( Vector2D<int> (x,y)))
   //                        {
   //                          float theta=fabs(Vector2D<int>::angle(GoalPoint,Vector2D<int> (x,y)));
   //                          if(theta>max_theta) 
   //                            {
   //                              max_theta=theta;
   //                              maxX=x;
   //                              maxY=y;
   //                            }
   //                        }
                          
   //                      }
                       
   //                    }
   //                }
   //                passPoint.x=maxX;
   //                passPoint.y=maxY;
   //              }
   //              else if((x3<x1<x4 && y3<y1<y4)||(x4<x1<x3 && y4<y1<y3)) //X1 lies between X3 and X4
   //              {
   //                Point1.x=x1;
   //                Point1.y=y1;
   //                Point2.x=x3;
   //                Point2.y=y3;

   //                if(Point1.x<Point2.x)
   //                {
   //                    for (int x= Point1.x; x < Point2.x; ++x)
   //                    {
   //                      if(b2!=0)
   //                      {
   //                        int y=-(a2*x+c2)/b2;
   //                        if(checkPointInField( Vector2D<int> (x,y)))
   //                        {
   //                          float theta=fabs(Vector2D<int>::angle(GoalPoint,Vector2D<int> (x,y)));
   //                          if(theta>max_theta) 
   //                            {
   //                              max_theta=theta;
   //                              maxX=x;
   //                              maxY=y;
   //                          }
   //                        }
                          
   //                      }
   //                    }
   //                }
   //                else
   //                {
   //                    for (int x= Point2.x; x < Point1.x; ++x)
   //                    {
   //                      if(b2!=0)
   //                      {
   //                         int y=-(a2*x+c2)/b2;
   //                        if(checkPointInField( Vector2D<int> (x,y)))
   //                        {
   //                          float theta=fabs(Vector2D<int>::angle(GoalPoint,Vector2D<int> (x,y)));
   //                          if(theta>max_theta) 
   //                          {
   //                            max_theta=theta;
   //                            maxX=x;
   //                            maxY=y;
   //                          }
   //                        }
                          
   //                      }
                       
   //                    }
   //                }
   //                passPoint.x=maxX;
   //                passPoint.y=maxY;
   //              }
   //            }

   //          }
   //          else if(intersecting1==true) //case 2 both ellipses intersect the line but have no pt of intersection between themselves
   //          {
   //            Point1.x=x1;
   //            Point1.y=y1;
   //            Point2.x=x2;
   //            Point2.y=y2;

   //            if(Point1.x<Point2.x)
   //                {
   //                    for (int x= Point1.x; x < Point2.x; ++x)
   //                    {
   //                      if(b2!=0)
   //                      {
   //                        int y=-(a2*x+c2)/b2;
   //                        if(checkPointInField( Vector2D<int> (x,y)))
   //                        {
   //                          float theta=fabs(Vector2D<int>::angle(GoalPoint,Vector2D<int> (x,y)));
   //                         if(theta>max_theta) 
   //                          {
   //                            max_theta=theta;
   //                            maxX=x;
   //                            maxY=y;
   //                          }
   //                        }
                          
   //                      }
   //                    }
   //                }
   //              else
   //                {
   //                    for (int x= Point2.x; x < Point1.x; ++x)
   //                    {
   //                      if(b2!=0)
   //                      {
   //                         int y=-(a2*x+c2)/b2;
   //                        if(checkPointInField( Vector2D<int> (x,y)))
   //                        {
   //                          float theta=fabs(Vector2D<int>::angle(GoalPoint,Vector2D<int> (x,y)));
   //                          if(theta>max_theta) 
   //                            {
   //                              max_theta=theta;
   //                              maxX=x;
   //                              maxY=y;
   //                            }
   //                        }
                          
   //                      }
                       
   //                    }
   //                }
   //                passPoint.x=maxX;
   //                passPoint.y=maxY;

   //          }
   //          else  // case 3 when the ellipse of ball/receiver interaction does not intersect the line of receiver/marker interaction 
   //          {
   //            c1,a1=c1=(u*u-w*w);
   //            b1=-2*(state.homePos[passrer_id].x*u*u-state.homePos[receiver_id].x*w*w);
   //            d1=-2*(state.homePos[passrer_id].y*u*u-state.homePos[receiver_id].y*w*w);
   //            e1=(state.homePos[passrer_id].x*state.homePos[passrer_id].x+state.homePos[passrer_id].y*state.homePos[passrer_id].y)*u*u-(state.homePos[receiver_id].x*state.homePos[receiver_id].x+state.homePos[receiver_id].y*state.homePos[receiver_id].y)*w*w;

   //            a2=(OPP_GOAL_Y-state.homePos[receiver_id].y);
   //            b2=(OPP_GOAL_X-state.homePos[receiver_id].x);
   //            c2=(OPP_GOAL_X-state.homePos[receiver_id].x)*OPP_GOAL_Y-(OPP_GOAL_Y-state.homePos[receiver_id].y)*OPP_GOAL_X;

   //             x1=-(c2 - (b2*(pow(a2,2)*d1 + a2*pow((pow(a2,2)*pow(d1,2) - 4*b1*e1*pow(a2,2) - 2*a2*b2*c1*d1 + 4*b1*a2*c1*c2 + pow(b2,2)*pow(c1,2) - 4*a1*e1*pow(b2,2) + 4*a1*b2*c2*d1 - 4*a1*b1*pow(c2,2)),(1/2)) + 2*a1*b2*c2 - a2*b2*c1))/(2*(b1*pow(a2,2) + a1*pow(b2,2))))/a2 ;
   //             y1=-(pow(a2,2)*d1 + a2*pow((pow(a2,2)*pow(d1,2) - 4*b1*e1*pow(a2,2) - 2*a2*b2*c1*d1 + 4*b1*a2*c1*c2 + pow(b2,2)*pow(c1,2) - 4*a1*e1*pow(b2,2) + 4*a1*b2*c2*d1 - 4*a1*b1*pow(c2,2)),(1/2)) + 2*a1*b2*c2 - a2*b2*c1)/(2*(b1*pow(a2,2) + a1*pow(b2,2))) ;
              
   //             x2=-(c2 - (b2*(pow(a2,2)*d1 - a2*pow((pow(a2,2)*pow(d1,2) - 4*b1*e1*pow(a2,2) - 2*a2*b2*c1*d1 + 4*b1*a2*c1*c2 + pow(b2,2)*pow(c1,2) - 4*a1*e1*pow(b2,2) + 4*a1*b2*c2*d1 - 4*a1*b1*pow(c2,2)),(1/2)) + 2*a1*b2*c2 - a2*b2*c1))/(2*(b1*pow(a2,2) + a1*pow(b2,2))))/a2 ;
   //             y2=-(pow(a2,2)*d1 - a2*pow((pow(a2,2)*pow(d1,2) - 4*b1*e1*pow(a2,2) - 2*a2*b2*c1*d1 + 4*b1*a2*c1*c2 + pow(b2,2)*pow(c1,2) - 4*a1*e1*pow(b2,2) + 4*a1*b2*c2*d1 - 4*a1*b1*pow(c2,2)),(1/2)) + 2*a1*b2*c2 - a2*b2*c1)/(2*(b1*pow(a2,2) + a1*pow(b2,2))) ;
   //             if(fabs(x1-OPP_GOAL_X)+fabs(y1-OPP_GOAL_Y)>fabs(x2-OPP_GOAL_X)+fabs(y2-OPP_GOAL_Y))
   //             {
   //              passPoint.x=x2;
   //              passPoint.y=y2;
   //             }
   //             else
   //             {
   //              passPoint.x=x1;
   //              passPoint.y=y1;
   //             }
   //          }  // passPoint selected for passing 
            
   //          if(passPoint.x==0 && passPoint.y==0)
   //          {
   //            passPoint.x=state.homePos[receiver_id].x;
   //            passPoint.y=state.homePos[receiver_id].y;
   //          } //if no valid pass point found then consider the 
           
   //         //!!!!!!!!!!!!!!!!!!!  finding out the probability of scoring and receiving at that point !!!!!!!!!!!!!!!!!!
   //          float Prob_total,Prob_scoring,Prob_receiving;
   //          float pa1,pa2,pa3,pa4; 
   //          /*
   //            pa1=probability of receiving pass based on number of opponents close to ball
   //            pa2=probability of receiving pass based on opponents who can intercept the pass
   //            pa3=probability of receiving pass based on length of pass 
   //            pa4=probability of receiving pass based on location of passPoint
   //          */ 
             
   //            //calculating pa1
   //            {
   //              float temp=1.0000f;
   //              for (int id = 0; id < HomeTeam::SIZE; ++id)
   //              {
   //                Vector2D<int> ballPos(state.ballPos.x,state.ballPos.y);
   //                Vector2D<int> passerPos(state.homePos[passrer_id].x,state.homePos[passrer_id].y);
   //                Vector2D<int> receivePos(passPoint.x,state.homePos[receiver_id].y);

   //                if(Vector2D<int>::dist(ballPos,passerPos)<4*BOT_RADIUS && fabs(Vector2D<int>::angle(passerPos,receivePos)-Vector2D<int>::angle(passerPos,Vector2D<int> (state.awayPos[id].x,state.awayPos[id].y)))< PI/10 ) 
   //                {
   //                  temp*=fabs(Vector2D<int>::angle(passerPos,receivePos)-Vector2D<int>::angle(passerPos,Vector2D<int> (state.awayPos[id].x,state.awayPos[id].y)))/PI;
   //                }
   //              } 
   //              pa1=temp+0.00001f;
   //            }
             
   //            //calculating pa2
   //            {
   //              float temp;
   //              for (int id = 0; id < HomeTeam::SIZE; ++id)
   //              {
   //                Vector2D<int> ballPos(state.ballPos.x,state.ballPos.y);
   //                Vector2D<int> passerPos(state.homePos[passrer_id].x,state.homePos[passrer_id].y);
   //                Vector2D<int> receivePos(passPoint.x, passPoint.y);
   //                Vector2D<int> interceptorPos(state.awayPos[id].x , state.awayPos[id].y);

   //                if(Vector2D<int>::dist(Vector2D<int> (state.awayPos[id].x,state.awayPos[id].y),receivePos)<6*BOT_RADIUS)
   //                {
                    
   //                  float distance_from_line = fabs(((passPoint.y - state.ballPos.y) * (state.awayPos[id].x)) - ((passPoint.x - state.ballPos.x) * (state.awayPos[id].y)) + (passPoint.x * state.ballPos.y) - (passPoint.y * state.ballPos.x)) / sqrt((receivePos - ballPos).absSq());
   //                  float projected_distance = sqrt((interceptorPos - ballPos).absSq()) * cos( (Vector2D<int>::angle(passerPos,receivePos)-Vector2D<int>::angle(passerPos,Vector2D<int> (state.awayPos[id].x,state.awayPos[id].y))) * PI / 180.0);

   //                  float prob_part_1 = (1 - (projected_distance / sqrt((ballPos - receivePos).absSq()))) + 0.01;
   //                  float prob_part_2 = (distance_from_line / (2.83 * HALF_FIELD_MAXX)) + 0.01;

   //                  if(prob_part_2 * prob_part_1 < temp)
   //                    temp = prob_part_1 * prob_part_2;

   //                }
   //              } 
   //               pa2 = temp+0.00001f;
   //            }

   //            //calculating pa3
   //           {
   //               Vector2D<int> passerPos(state.homePos[passrer_id].x,state.homePos[passrer_id].y);
   //               Vector2D<int> receiverPos(state.homePos[receiver_id].x,state.homePos[receiver_id].y);
   //               float dista=Vector2D<int>::dist(passerPos,receiverPos);
   //               if(dista<=maxX)
   //               pa3=exp(-3*pow(2*dista/maxX-1,2));
   //               else
   //               pa3=0.00001f;
   //          }

   //          //calculating pa4
   //          {
			// 	//CONSTANTS ARE SET FOR HALF_FIELD_MAXX=900 AND HALF_FIELD_MAXY=600
			// 	double prx,pry,pconst=0.7;
			// 	int Danger_Self_Goal=-7*HALF_FIELD_MAXX/8,p_thres=7*HALF_FIELD_MAXX/8;

			// 	//FINDING PROBABILITY IN X DIRECTION
			// 	if(passPoint.x<Danger_Self_Goal)
			// 		prx=0;
			// 	else if(passPoint.x<0)
			// 		prx=pconst*erf((passPoint.x-Danger_Self_Goal)/10.0);
			// 	else if(passPoint.x<p_thres)
			// 		prx=pconst+0.3*erf((passPoint.x)/10.0);
			// 	else if(passPoint.x<HALF_FIELD_MAXX)
			// 		prx=1-erf(passPoint.x-10);
			// 	else prx=0;

			// 	//FINDING PROBABILITY IN Y DIRECTION
			// 	if(passPoint.y<-HALF_FIELD_MAXY)
			// 		pry=0;
			// 	else if(passPoint.y<0)
			// 		pry=erf((passPoint.y+HALF_FIELD_MAXY)/8000.0);
			// 	else if(passPoint.y<HALF_FIELD_MAXY)
			// 		pry=1-erf((passPoint.y)/8000.0);
			// 	else pry=0;

			// 	//COMBINING RESULTS
			// 	double pr=sqrt(prx*pry);

			// 	//pa4 = pr
			// }

   //          Prob_receiving=pa1*pa2*pa3*pa4;

   //            float pb1,pb2,pb3;
   //            /*
   //              pb1=probability of shot reaching faster than goalkeeper 
   //              pb2=wide enough theta
   //              pb3=not enough time for defenders to block the shot
   //            */

   //              //calculating pb1
   //              {
   //                Vector2D<int> ballPos(state.ballPos.x,state.ballPos.y);
   //                Vector2D<int> passerPos(state.homePos[passrer_id].x,state.homePos[passrer_id].y);
   //                Vector2D<int> receivePos(passerPos.x,passerPos.y);
   //                Vector2D<int> oppGoaliePos(state.awayPos[state.opp_goalie].x,state.awayPos[state.opp_goalie].y);
   //                float goalieReceiverDist=Vector2D<int>::dist(receivePos,Vector2D<int>(state.awayPos[state.opp_goalie].x,state.awayPos[state.opp_goalie].y) );
   //                float goaliePoleDist;
   //                if(Vector2D<int>::dist(oppGoaliePos,Vector2D<int> (OPP_GOAL_X, OPP_GOAL_MAXY ))>Vector2D<int>::dist(oppGoaliePos,Vector2D<int> (OPP_GOAL_X, OPP_GOAL_MINY )))
   //                {
   //                    goaliePoleDist=Vector2D<int>::dist(oppGoaliePos,Vector2D<int> (OPP_GOAL_X, OPP_GOAL_MAXY ));
   //                }
   //                else goaliePoleDist=Vector2D<int>::dist(oppGoaliePos,Vector2D<int> (OPP_GOAL_X, OPP_GOAL_MINY ));
   //                pb1=goaliePoleDist/goalieReceiverDist;
   //              }

   //              //calculating pb2
   //              {
   //              	float theta=fabs(Vector2D<int>::angle(Vector2D<int>(maxX,maxY),GoalPole)-Vector2D<int>::angle(Vector2D<int>(maxX,maxY),Vector2D<int>(state.awayPos[state.opp_goalie].x,state.awayPos[state.opp_goalie].y)));
   //              	float max_possible_theta=fabs(normalizeAngle(Vector2D<int>::angle(Vector2D<int>(maxX,maxY),Vector2D<int>(OPP_GOAL_X,OPP_GOAL_MAXY))-Vector2D<int>::angle(Vector2D<int>(maxX,maxY),Vector2D<int>(OPP_GOAL_X,OPP_GOAL_MINY))));
   //              	pb2=theta/max_possible_theta;
   //              }

   //              //calculating pb3
   //              {
   //              	float angle_temp=fabs(normalizeAngle(Vector2D<int>::angle(Vector2D<int>(state.awayPos[0].x,state.awayPos[0].y),Vector2D<int>(maxX,maxY)) - Vector2D<int>::angle(Vector2D<int>(OPP_GOAL_X,0),Vector2D<int>(maxX,maxY))));
   //              	float dist_temp=Vector2D<int>::dist(Vector2D<int>(maxX,maxY),Vector2D<int>(OPP_GOAL_X,0));
   //              	float temp=dist_temp*angle_temp;
   //              	for (int id = 0; id < HomeTeam::SIZE; ++id)
   //              	{
   //              		if(fabs(normalizeAngle(Vector2D<int>::angle(Vector2D<int>(state.awayPos[id].x,state.awayPos[id].y),Vector2D<int>(maxX,maxY)) - Vector2D<int>::angle(Vector2D<int>(OPP_GOAL_X,0),Vector2D<int>(maxX,maxY)))) * Vector2D<int>::dist(Vector2D<int>(maxX,maxY),Vector2D<int>(OPP_GOAL_X,0)) > temp)
   //              		{
   //              			temp=fabs(normalizeAngle(Vector2D<int>::angle(Vector2D<int>(state.awayPos[id].x,state.awayPos[id].y),Vector2D<int>(maxX,maxY)) - Vector2D<int>::angle(Vector2D<int>(OPP_GOAL_X,0),Vector2D<int>(maxX,maxY))) * Vector2D<int>::dist(Vector2D<int>(maxX,maxY),Vector2D<int>(OPP_GOAL_X,0)));
   //              		}
   //              	}
   //              	pb3=temp/=Vector2D<int>::dist(Vector2D<int>(maxX,maxY),Vector2D<int>(OPP_GOAL_X,0));
   //              }

   //              Prob_scoring=pb1*pb2*pb3;
   //              //-----------------------------------------------------

   //              Prob_total=Prob_receiving*Prob_scoring;
   //              if(Prob_total>maxProb)
   //              {
   //                maxProb=Prob_total;
   //                recvrID=receiver_id;
   //                destPassPoint.x=maxX;
   //                destPassPoint.y=maxY;
   //              }

   //          receiver_id++;
   //        }

      //just for debigging 
      destPassPoint.x=OPP_GOAL_X;
      destPassPoint.y=CENTER_Y;
      //*******************roles for the bots************************* 
      param.PassToPointP.x=destPassPoint.x;
      param.PassToPointP.x=destPassPoint.y;
      roleList[0].push_back(std::make_pair("TPassToPoint", param));
      
      param.ReceiveP.x=destPassPoint.x;
      param.ReceiveP.x=destPassPoint.y;
      roleList[1].push_back(std::make_pair("TReceive", param));

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

    bool PPassTest::checkPointInField(Vector2D<int> point)
    {
      if(fabs(point.x)<fabs(HALF_FIELD_MAXX) && fabs(point.y)<fabs(HALF_FIELD_MAXY)) return true;
      return false;
    }

}// namespace strategy