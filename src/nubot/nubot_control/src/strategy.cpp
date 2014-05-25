#include "nubot/nubot_control/strategy.hpp"

using namespace nubot;
Strategy::Strategy()
{
    opp_goal_pos_ = DPoint(900.0,0.0);
    dribble_state_ = false;

    NeedEvaluat  = false;
    stucktime_ = 0;
    running = false;

    worldmodelinfo_ = &(m_plan_.m_behaviour_.worldmodelinfo_);
}
void Strategy::quickShoot()
{
        float  leftDelta,rightDelta;
        float  LeftDelta,RightDelta;
        DPoint Target;
        double targetdirection;

        DPoint  BallGlobalPos =  worldmodelinfo_->ball_pos_;
        DPoint leftOppgoalCorner1 = DPoint(900,40);
        DPoint rightOppgoalCorner1 = DPoint(900,-40);
        double Bodyfacing =  worldmodelinfo_->robot_ori_.radian_;

        Target =  opp_goal_pos_;
        if(!evaluateKick(Target,leftDelta,rightDelta))  //
        {
            Target =  DPoint(900,0);
            targetdirection =  thetaof2p(BallGlobalPos,Target);
            LeftDelta = thetaof2p(BallGlobalPos,leftOppgoalCorner1);
            RightDelta = thetaof2p(BallGlobalPos,rightOppgoalCorner1);
        }
        else
        {
            targetdirection =  thetaof2p(BallGlobalPos,Target);
            LeftDelta =  targetdirection+leftDelta;
            RightDelta =  targetdirection-rightDelta; //
        }

        //
        if(Bodyfacing<LeftDelta && Bodyfacing>RightDelta) //
        {
             currentstate_ = AtShootSituation ;
             currentactionselection(currentstate_);
        }
        else
        {
            m_plan_.m_behaviour_.rotate2AbsOrienation(1.5,1.7,targetdirection,10);
        }

}

void
Strategy::kickball4Coop(DPoint target)
{

}

void
Strategy::avoidObs()
{


}
void
Strategy::triggerShoot(DPoint target)
{
 /*   CMSLControlDlg* ppdlg= (CMSLControlDlg*)(AfxGetApp()->GetMainWnd());
    int  shootdis = 0;
    //if(err_theta < pi/12 && abs(m_WorldModel->MyInfo.RelVel.Y ) <= 10) //»¹µÃÅÐ¶ÏœÇ¶È±ä»¯ÂÊ £¿ // ÖžÏòÇòÃÅµÄËÙ¶È¿ÉÒÔÓÐ £¬ÆäËû·œÏòµÄËÙ¶ÈÎª 0  µÄÇé¿ö ŒŽ¿É  £¡£¡
    double KicktargetForce = 0;
    double dis2kickTar = m_MyMath.Dis_2P(m_WorldModel->MyInfo.GlobalPos,target);
    shootdis = 	int(dis2kickTar/100.0);

    if ((dis2kickTar>100)&&(dis2kickTar<200))
    {
            shootdis=2;
    }
    if(shootdis < 1) shootdis=1;
                //checkFrontLength = m_WorldModel->ShotArray[shootdis+20-1];
    KicktargetForce = (int)((m_WorldModel->ShotArray[shootdis-1]));//*(1.0-(m_WorldModel->MyInfo.RelVel.X/LIMIT_VEL)));

    if((shootdis < shootshortdis)&&( shootdis > 4))
    {
       if (m_WorldModel->MyInfo.RelVel.x > DRIBLE_MIN_VEL2/3.0)
        {
            KicktargetForce =KicktargetForce*0.81;//KickRatio;///ÉäÃÅÁŠÁ¿ÏµÊýÐÞžÄ£¬±äÎªÈ«ŸÖ±äÁ¿
        }
        else
            KicktargetForce =KicktargetForce*0.99;///ÉäÃÅÁŠÁ¿ÏµÊýÐÞžÄ£¬±äÎªÈ«ŸÖ±äÁ¿
        }

           m_Plan.m_Behaviour.IsKick = KicktargetForce;

        if(m_Plan.m_Behaviour.IsKick)
        {
                    //0x55 ²»ÉäÃÅ£¬±Õ»·£¬0x5aÉäÃÅ£¬±Õ»·£¬0xa5²»ÉäÃÅ£¬¿ª»·
            if (m_Plan.m_Behaviour.isopenloop==0) {

            ppdlg->m_ControlModule.m_Command.SetAt(1,0x5a);
            ppdlg->m_ControlModule.m_Command.SetAt(8,m_Plan.m_Behaviour.IsKick);//abs80608

            }
            else if(m_Plan.m_Behaviour.isopenloop==1){

            ppdlg->m_ControlModule.m_Command.SetAt(1,0xaa);//ÐÞžÄ
            ppdlg->m_ControlModule.m_Command.SetAt(8,m_Plan.m_Behaviour.IsKick);//abs80608
                    }
            ppdlg->m_ControlModule.SendCommand();//µœµ×³É¹ŠÃ»ÓÐ £¬ ¿ÉÒÔÔÙ¿Ž¿Ž
        }
            m_Plan.m_Behaviour.IsDribble = FALSE;*/
}

void  Strategy::turn4Shoot()
{
        DPoint kicktarget = opp_goal_pos_;
        double Bodyfacing = worldmodelinfo_->robot_ori_.radian_;
        double theta2target =  thetaof2p(worldmodelinfo_->robot_pos_,kicktarget);
        double leftdelta,rightdelta;

        leftdelta = leftdelta_for_shoot_;
        rightdelta = rightdelta_for_shoot_;

        if( Bodyfacing < theta2target + leftdelta + deg(5)  && Bodyfacing > theta2target - rightdelta - deg(5)) // ¶ÔÓÚœÇËÙ¶ÈµÄ²¹³¥Òª×¢Òâ£¡Âú×ãÉäÃÅÌõŒþ
        {
            currentstate_ = AtShootSituation ;
            currentactionselection(currentstate_);
        }
        else
        {
            m_plan_.m_behaviour_.rotate2AbsOrienation(1.5,1.7,theta2target,10);

        }


}

void Strategy::dynamicQuickShoot()
{
    DPoint kicktarget  = opp_goal_pos_;   //Ä¿±ê
    double Bodyfacing  = worldmodelinfo_->robot_ori_.radian_;
    DPoint robotglobalpos = worldmodelinfo_->robot_pos_;

    double theta2target =  thetaof2p(robotglobalpos,kicktarget);

    float leftdelta,rightdelta;

    evaluateKick(kicktarget,leftdelta,rightdelta);

    leftdelta  =  leftdelta_for_shoot_;
    rightdelta =  rightdelta_for_shoot_;

    double thetaerr2kicktarget;
    thetaerr2kicktarget = theta2target - Bodyfacing;
    thetaerr2kicktarget = angularnorm(thetaerr2kicktarget);

             if( Bodyfacing<theta2target + leftdelta && Bodyfacing> theta2target - rightdelta ) // ¶ÔÓÚœÇËÙ¶ÈµÄ²¹³¥Òª×¢Òâ£¡
             {
                 currentstate_ = AtShootSituation;
                 currentactionselection(currentstate_);
             }
             else
             {
                     currentstate_ = TurnForShoot;
                     currentactionselection(currentstate_);
             }
}

DPoint
Strategy::defencepointCalculate()
{
    DPoint tmpball = worldmodelinfo_->ball_pos_;
    DPoint tmp;
    tmp = PointOfLine(tmpball,DPoint(-900,0),worldmodelinfo_->indist_);

    if (worldmodelinfo_->target_orientation_ != 0)
        tmp = RotOfPoint(tmp,tmpball,worldmodelinfo_->target_orientation_,true);
    return tmp;
}

void
Strategy::stuckProcess(int stucktime)
{

}
void
Strategy::findBall()
{
    DPoint robotglobalpos = worldmodelinfo_->robot_pos_;
    DPoint  target;
    if(searchball_active_ == false)
             {
                target =  DPoint(-650,0);
                 if(robotglobalpos.distance(target)< 50)
                 {
                     searchball_active_ = 1;
                 }
             }
             else if(searchball_active_ == 1)
             {
                 target =  DPoint(650,0);
                 if(robotglobalpos.distance(target)< 50)
                 {
                     searchball_active_ = 0;
                 }
             }

    m_plan_.m_behaviour_.move2Position(1.75,1.2,target,MAXVEL);

}
//  1  the ball is out of the field
//  2  the ball moves along the border
//  3
void
Strategy::activeCatchBall()
{

    DPoint target;

    if(worldmodelinfo_->ball_info_state_ == CannotSeeBall)
    {
        return;
    }

    if(IsLocationInField(worldmodelinfo_->robot_pos_))
    {
       m_plan_.catchBall();
    }
    else
    {
       return;
    }

}
/*
void
Strategy::activeCatchBall()
{
        DPoint target;
        double theta;

        DPoint BallGlobalpos = worldmodelinfo_->ball_pos_;
        DPoint BallGlobalVel;
        DPoint MyGlobalPos   = worldmodelinfo_->robot_pos_;

        if(BallGlobalpos.distance(MyGlobalPos)<VISIONSCALE)
            BallGlobalVel=worldmodelinfo_->ball_vel_;
        else
            BallGlobalVel=DPoint(0.0,0.0);


        DPoint outtarget=BallGlobalpos;

        DPoint catchspeed=vrel2global(DPoint(150,0),worldmodelinfo_->robot_ori_);//

        if(ISLocationInOppGoalArea(BallGlobalpos) && IsLocationInOppPenalty(MyGlobalPos))//
        {
           double thetaofball_global=m_MyMath.ThetaOf2P(DPoint(0,0),m_WorldModel->ball.GlobalPos);
           m_MyMath.ClearUpAngle(thetaofball_global);

           DPoint targetofGoalKeeper;
           targetofGoalKeeper.x=780;//Ð¡œûÇøx×ø±ê
           targetofGoalKeeper.y=targetofGoalKeeper.x*tan(thetaofball_global);

           if(IsInOppPenaltyLeftOrRight(MyGlobalPos))
            {
                targetofGoalKeeper.x=FIELDLENGTH/2.0-OurPenltyAreaX;
                targetofGoalKeeper.y=MyGlobalPos.y;
                thetaofball_global=m_MyMath.ThetaOf2P(targetofGoalKeeper,BallGlobalpos);
                m_Plan.CatchTarget(targetofGoalKeeper,thetaofball_global,300,LIMIT_VEL*0.7);
            }
            else
                m_Plan.CatchTarget(targetofGoalKeeper,thetaofball_global,300,LIMIT_VEL*0.7);


        }
        else if(OutFieldProcess(MyGlobalPos,outtarget,DPoint(0.0,0.0)))  //
        {
            theta=m_MyMath.ThetaOf2P(MyGlobalPos,BallGlobalpos);
            m_Plan.CatchTarget2(outtarget,theta,300,LIMIT_VEL*0.7);
        }
        else
        {
                if(ActiveRobotNum()>2) //
                {
                    if(IsInOurPenalty2(m_WorldModel->ball.GlobalPos))
                    {

                        if(BallGlobalpos.x-MyGlobalPos.x>20)// ,
                        {
                            target=BallGlobalpos;
                        }
                        else
                        {
                            target=FindNearstPenaltyOutPoint(m_WorldModel->ball.GlobalPos,m_WorldModel->MyInfo.GlobalPos);   //  œ«Ä¿±êÉè¶šÎªÇòÓë»úÆ÷ÈËÖ®ŒäµÄÁ¬ÏßºÍœûÇø±ßµÄœ»µã
                        }
                         theta=m_MyMath.ThetaOf2P(m_WorldModel->MyInfo.GlobalPos,m_WorldModel->ball.GlobalPos);
                         m_Plan.CatchTarget(target,theta,300,LIMIT_VEL*0.7);
                    }

                    else if(m_WorldModel->ball.GlobalPos.x>FIELDLENGTH/2.0-50&&abs(m_WorldModel->ball.GlobalPos.y)>FIELDWIDTH/2.0-30)//ÇòÔÚ±ßÏß
                    {
                        m_Plan.CatchBallFromBehind(BallGlobalVel,catchspeed,LIMIT_VEL,1.5*DRIBLEDIS,TRUE);   // µÃ×ÐÏžÊÔÊÔ
                    }
                    else //if(m_WorldModel->ball.GlobalPos)//LJ ÎªÁËžüºÏÀíµÄ·ÀÓù£¬²»ÄÜÈÃÖ÷¹¥Ò»Ö±×·Çò£¬ÐèÒªžøÖ÷¹¥ŒÓÉÏ²¿·ÖÏÞÖÆº¯Êý£¬¿ŒÂÇ³¡µØºÍ¶ÓÓÑÒòËØ
                    {
                        /*if(WhoDoWhenActiveLoseChance()!=m_WorldModel->MyNum||OurRobotCanCatchBallBeforeMe(m_WorldModel->MyInfo.GlobalPos, width_avoidcrash_ourself, 1.0)>0)//ProtectBallNearby:
                        {
                            ballshareglobalpos=GetBallLocation();
                            target=FindBestProtectPointNearby();
                            if(IsInOppPenalty2(target))
                                target = FindNearstOppPenaltyOutPoint(target);
                            TargetGlobal=target;
                            theta=m_MyMath.ThetaOf2P(m_WorldModel->ourgoal.GlobalPos,ballshareglobalpos);
                            m_Plan.m_Behaviour.Move2Target_PD(10.0,1.0,target,DPoint(0,0),LIMIT_VEL,0.0);
                            m_Plan.m_Behaviour.Turn2Angle(4.25,66,theta,10.0,0);

                            m_WorldModel->CurrentAction=ACProtectBallNearby; //LJ coachÏÔÊŸÓÃ
                        }
                        else
                            m_Plan.m_Behaviour.TraceBallCondition2(LIMIT_VEL,10);  //Ö»ÊÇžºÔðÈ¥×¥Çò£¬ Ã»ÓÐ¿ŒÂÇµ±»úÆ÷ÈË±»¶ÔÊÖ»úÆ÷ÈËŽøÇòµÄÊ±ºòÈçºÎ×¥Çó
                    }
                }
                else if(ActiveRobotNum()==2)   //  ÕâžöÊ±ºòÇ°·æ¿ÉÒÔœøÈëµœœûÇøµÄ
                {
                    if(m_WorldModel->RobotIfActive[0]&&GoalieWillOut)
                    {
                        if(m_WorldModel->MyInfo.GlobalPos.y>m_WorldModel->ball.GlobalPos.y)
                            target=m_WorldModel->ball.GlobalPos+DPoint(100,100);
                        else
                            target=m_WorldModel->ball.GlobalPos+DPoint(100,-100);
                        theta=m_MyMath.ThetaOf2P(m_WorldModel->MyInfo.GlobalPos,m_WorldModel->ball.GlobalPos);
                        m_Plan.CatchTarget(target,theta,300,LIMIT_VEL*0.7);
                    }
                    else if(IsInOurPenalty2(m_WorldModel->ball.GlobalPos))
                    {
                        if(IsInRisktoGoal())//LJ ·ÀÖ¹ÎÚÁú
                            m_Plan.CatchBallFromBehind(BallGlobalVel,catchspeed,LIMIT_VEL,1.5*DRIBLEDIS,TRUE);
                        else
                        {
                            theta=m_MyMath.ThetaOf2P(m_WorldModel->MyInfo.GlobalPos,m_WorldModel->ball.GlobalPos);
                            m_Plan.CatchTarget(m_WorldModel->ball.GlobalPos,theta,300,LIMIT_VEL*0.7);
                        }
                    }

                    else
                    {

                        if(abs(m_WorldModel->ball.GlobalPos.y)>FIELDWIDTH/2.0-30)//ÇòÔÚ±ßÏß
                        {
                             m_Plan.CatchBallFromBehind(BallGlobalVel,catchspeed,LIMIT_VEL,1.5*DRIBLEDIS,TRUE);
                        }
                        else
                        {
                            m_Plan.m_Behaviour.TraceBallCondition2(LIMIT_VEL,10);//zzw
                        }
                    }
                }
                else//³¡ÉÏÒ»žö»úÆ÷ÈË
                {
                    if(abs(m_WorldModel->ball.GlobalPos.y)>FIELDWIDTH/2.0-30)//ÇòÔÚ±ßÏß
                    {
                        m_Plan.CatchBallFromBehind(BallGlobalVel,catchspeed,LIMIT_VEL,1.5*DRIBLEDIS,TRUE);
                    }
                    else if(IsInOurPenalty2(m_WorldModel->ball.GlobalPos))
                    {
                        m_Plan.CatchBallFromBehind(BallGlobalVel,catchspeed,LIMIT_VEL,1.5*DRIBLEDIS,FALSE);
                    }
                    else
                    {
                         m_Plan.m_Behaviour.TraceBallCondition2(LIMIT_VEL,10);//zzw
                    }
                }
        }

}*/
void
Strategy::activeDecisionMaking()
{
     currentstate_ = currentstateSelection();
     currentactionselection(currentstate_);
}

unsigned char
Strategy::currentstateSelection()
{
    DPoint robotglobalpos   =  worldmodelinfo_->robot_pos_;
    float  robotorientation =  worldmodelinfo_->robot_ori_.radian_;

    bool   is_robot_stuck   =  worldmodelinfo_->is_robot_stuck_;
    int    ball_info_state  =  worldmodelinfo_->ball_info_state_;
    bool   dribble_state     =  dribble_state_;
    int    currentactiverobotnum = worldmodelinfo_->active_robot_num_;

    bool    quickshoot  =  quick_shoot_state_;
    bool    dynamicshoot = dynamic_shoot_state_;

    DPoint LFieldCorner = DPoint(FIELDLENGTH/2,FIELDWIDTH/2);
    DPoint RFieldCorner = DPoint(FIELDLENGTH/2,-FIELDWIDTH/2);
    DPoint oppgoalpos   = opp_goal_pos_;


    DPoint leftoppgoalcorner   =   DPoint(900,100);//  zzw12
    DPoint rightoppgoalcorner  =   DPoint(900,-100);
    DPoint leftoppgoalcorner1  =   DPoint(900,40);
    DPoint rightoppgoalcorner1 =   DPoint(900,-40);

    float leftdelta,rightdelta;
    int   kickforce;
    float targettheta;

    DPoint kicktarget = oppgoalpos;
    DPoint realtargetpos = kicktarget- robotglobalpos;
    float  theta2target =  realtargetpos.angle().radian_;

    bool isInRightSmallCorner = IsInRectangle(DPoint(950,-650),DPoint(950,-375),DPoint(625,-375),DPoint(350,-650),robotglobalpos);
    bool isInRightLargeCorner = IsInRectangle(DPoint(950,-650),DPoint(950,-275),DPoint(525,-275),DPoint(250,-650),robotglobalpos);
    bool isInLeftSmallCorner  = IsInRectangle(DPoint(950,650),DPoint(350,650),DPoint(625,375),DPoint(950,375),robotglobalpos);
    bool isInLeftLargeCorner  = IsInRectangle(DPoint(950,650),DPoint(250,650),DPoint(525,275),DPoint(950,275),robotglobalpos);

    bool isInSmallRegion      = IsInRectangle(DPoint(950,-350),DPoint(950,350),DPoint(625,350),DPoint(625,-350),robotglobalpos);
    bool isInLargeRegion      = IsInRectangle(DPoint(950,-375),DPoint(950,375),DPoint(600,375),DPoint(600,-375),robotglobalpos);

    leftdelta_for_shoot_       =  thetaof2p(robotglobalpos,DPoint(900,100)) - theta2target;

    leftdelta_for_shoot_       =  angularnorm(leftdelta_for_shoot_);

    rightdelta_for_shoot_      = theta2target - thetaof2p(robotglobalpos,DPoint(900,-100));

    rightdelta_for_shoot_      = angularnorm(rightdelta_for_shoot_);


    if(is_robot_stuck = true)
      {
            currentstate_ = Stucked;
            stucktime_++;
      }
       /* else if(m_WorldModel->MatchType==OUR_PENALTY)  //
        {
            MyState = Penalty;
        }*/
        else if(ball_info_state != NOTSEEBALL)// active has ball information, local/sharing
        {
            if(dribble_state == false)   //   active does not have the ball,
            {
                    currentstate_ = SeeNotDribbleBall;  //  catch ball progress
            }
            else if(dribble_state == true)    //  active hold the ball
            {
                if(IsLocationInOurField(robotglobalpos) && currentactiverobotnum > 2 )  //
                {
                    currentstate_ = AvoidObs;      //   active avoid obstacle
                }
                else if(quickshoot == true)  //
                {
                        quick_shoot_count_++;

                        currentstate_ = CoopKickQuickShoot;  //quick shoot process

                        if(quick_shoot_count_ >  200)
                        {
                            quick_shoot_state_ = false;   //
                            quick_shoot_count_ = 0;
                        }
                    }
                    else if (dynamicshoot = true)  //
                    {
                        dynamic_shoot_count_++;
                        currentstate_ = DynamicShoot;

                        if(dynamic_shoot_count_ >  200)
                        {
                            dynamic_shoot_state_ = false;
                            dynamic_shoot_count_ = 0;
                        }

                    }
                    else if(evaluateKick(kicktarget,leftdelta,rightdelta))
                    {

                        currentstate_ = TurnForShoot;
                        leftdelta_for_shoot_ =  leftdelta;
                        rightdelta_for_shoot_ =  rightdelta;

                    }
                   /* else if(IsInGoalRegon ==  TRUE && isInSmallRegion)
                    {
                        if(m_Plan.EvaluateKickFor2012(kicktarget,leftdelta,rightdelta))  //  Œì²éµ±Ç°·œÏòÉÏÊÇ·ñÓÐ»úÆ÷ÈË
                        {
                            currentstate_ = TurnForShoot;
                            leftdelta_for_shoot_ =  leftdelta;
                            rightdelta_for_shoot_ =  rightdelta;
                        }
                        else
                        {
                            currentstate_ = AvoidObs;
                        }
                    }
                    else if(IsInGoalRegon == FALSE && isInLargeRegion)
                    {
                        if(m_Plan.EvaluateKickFor2012(kicktarget,leftdelta,rightdelta))  //  Œì²éµ±Ç°·œÏòÉÏÊÇ·ñÓÐ»úÆ÷ÈË
                        {
                            currentstate_ = TurnForShoot;
                            turn4shootleftdelta =  leftdelta;
                            turn4shootrightdelta =  rightdelta;
                        }
                        else
                        {
                            currentstate_ = AvoidObs;
                        }
                        IsInGoalRegon =  TRUE;
                    }
                    // 				else if(MyState==InCorner&&(isInRightLargeCorner||isInLeftLargeCorner))
                    // 						MyState = InCorner;			//Ö±œÓ¹æ»® £¬generatemap
                    // 				else if((MyState!=InCorner)&&(isInSmallRegion))
                    // 						MyState = InCorner;
                    else if(BezierPathPlan4Shoot(m_Plan.Active_ControlPointList))	  // œ«¶ÔControlPointListœøÐÐž³Öµ							//  controllist Îª¿ÕÊ±£¬ Í¬Ê±Ã»ÓÐÉäÃÅ×ŽÌ¬  ---1£¬¹æ»®£»2 ±ÜÕÏ£šÃ»·šÊµÏÖ¹æ»®Ö®Ê±£©¡£
                    {										  //  ËäÈ»Ö»ÊÇžö×ŽÌ¬ÅÐ¶Ï£¬ÀïÃæÍ¬Ñù°üº¬ÁËÂ·Ÿ¶µÄ¹æ»®
                        currentstate_ = PathPlan;   // ¿ŒÂÇµœŸàÀë¶ÔÊÖÇòÃÅŸàÀë,¹æ»®³öÏàÒÀµÄÂ·Ÿ¶²¢œøÐÐÅÐ¶ÏÊÇ·ñºÏÀí¡£ Èô²»ºÏÀíÄÇÃŽ¿ŒÂÇ ±ÜÕÏ£¬²¢œøÐÐÖØÐÂ¹æ»® ¡£
                    }*/
                    else
                    {
                        currentstate_ = AvoidObs;
                    }
            }
        }
        else
        {
            currentstate_ = CannotSeeBall;   //   search ball progress
        }
        //////////////////////////////////////////////////////////////////////////
        if(currentstate_ != TurnForShoot )
        {
            leftdelta_for_shoot_ = thetaof2p(robotglobalpos,DPoint(900,100)) - theta2target;
            leftdelta_for_shoot_ = angularnorm(leftdelta_for_shoot_);
            rightdelta_for_shoot_ = theta2target - thetaof2p(robotglobalpos,DPoint(900,-100));
            rightdelta_for_shoot_ = angularnorm(rightdelta_for_shoot_);
        }



       /* if(MyState != PathPlan)   //
        {
            active_pointlistID.RemoveAll();
            active_VoronoiEdgeList.RemoveAll();
            m_Plan.Active_ControlPointList.RemoveAll(); //
            m_Plan.ClearBezierPlanForActive();

        }*/
        if (currentstate_ != DynamicShoot)  //
        {
            dynamic_shoot_state_  = false;
            dynamic_shoot_count_  = 0;
        }
        if(currentstate_ != CoopKickQuickShoot)
        {
            quick_shoot_state_  = false;
            quick_shoot_count_  = 0;
        }
        if(currentstate_ != Stucked)
        {
            stucktime_ = 0;
        }
        return  currentstate_;
}
void
Strategy::currentactionselection(unsigned char  state)
{
        DPoint target =  opp_goal_pos_;
        unsigned char MyState;
        MyState = state;

        switch(MyState)
        {
        case Stucked:
            stuckProcess(stucktime_);
            break;
        case Penalty:
            //ActivePenaltyKick();
            break;
        case SeeNotDribbleBall:
            activeCatchBall();
            break;
        case AvoidObs:
            avoidObs();
            break;
        case PathPlan:
            //TrackingPath(target);
            break;
        case CoopKickQuickShoot:
            quickShoot();//turning & shoot!
            break;
        case AtShootSituation:  //
            quick_shoot_state_   = false;
            dynamic_shoot_state_ = false;
            m_plan_.m_behaviour_.app_vx_ = m_plan_.m_behaviour_.app_vx_ = 0;
            triggerShoot(target);//
            break;
        case TurnForShoot:   //
            turn4Shoot();
            break;
        case DynamicShoot:  //
            dynamicQuickShoot();
            break;
        case CannotSeeBall:
            findBall();
            break;
        default:
            break;
        }

}
bool
Strategy::checkShootDirection(float direction,float swidth,float lwidth,float len)
{

    DPoint trap[4];
    DPoint robotglobalpos = worldmodelinfo_->robot_pos_;

    trap[0]=prel2global(robotglobalpos,direction,DPoint(10.0f,-swidth));
    trap[1]=prel2global(robotglobalpos,direction,DPoint(len,-lwidth));
    trap[2]=prel2global(robotglobalpos,direction,DPoint(len,lwidth));
    trap[3]=prel2global(robotglobalpos,direction,DPoint(10.0f,swidth));

    for(int k  = 0 ;  k < OBSRECORDNUM ; k++ )
    {
        if(IsInRectangle(trap[0],trap[1],trap[2],trap[3],worldmodelinfo_->obs_pos_[k]))
        {
           return false;
        }
    }

       return true;


}

bool
Strategy::evaluateKick(DPoint &target,float & leftdelta,float &rightdelta)
{
    //81201
     //if(abs(m_WorldModel->ball.GlobalPos.Y)>100 //ÔÚœÇÇòÇøžœœü£¬ŽøÏò³¡µØÖÐÑë
    //						&& (FIELDLENGTH/2-abs(m_WorldModel->ball.GlobalPos.X))<(abs(m_WorldModel->ball.GlobalPos.Y)-100)/1.5)
     //return false;  //  zzw ? 16:12 2012-3-8


     float   dis2oppgoal =  worldmodelinfo_->robot_pos_.distance(opp_goal_pos_);
     float   direction   =  thetaof2p(worldmodelinfo_->robot_pos_,opp_goal_pos_);
     DPoint  ballglobalpos = worldmodelinfo_->ball_pos_;

     DPoint leftOppgoalCorner   =  DPoint(900,100);
     DPoint rightOppgoalCorner  =  DPoint(900,-100);
     DPoint leftOppgoalCorner1  =  DPoint(900,50);
     DPoint rightOppgoalCorner1 =  DPoint(900,-50);

      // if the  distance between te robot and the target is beyound the VISIONSCALE
      // then we do not need any evaluation
     if(NeedEvaluat == false && dis2oppgoal<VISIONSCALE-BOARDWIDTH/2)
     {
         NeedEvaluat = true;
     }
     else if(NeedEvaluat == true && dis2oppgoal<VISIONSCALE+BOARDWIDTH/2)
     {
         NeedEvaluat = true;
     }
     else
         NeedEvaluat = false;





         if(checkShootDirection(direction,50,75,100))
         {
            if(!NeedEvaluat)
            {
             target = opp_goal_pos_;
             leftdelta =   thetaof2p(ballglobalpos,leftOppgoalCorner1) - direction;
             rightdelta = -thetaof2p(ballglobalpos,rightOppgoalCorner1) + direction;
             return true;
            }
         }
         else
         {
            return false;
         }


    static int lastSelectedNum=5;
    int GoalPointNum = 6;
    static int noObsTime[6]={0,0,0,0,0,0};
    DPoint GoalPoint[GoalPointNum];
    int CannotPassRobot[GoalPointNum];
    int candidateForSelect[GoalPointNum];
    int tempIsInParallelogram;
    int candidataCase=0;

    DPoint ballGlobalpos = ballglobalpos ;

    for (int i=0;i<GoalPointNum;i++)
    {
        CannotPassRobot[i]=0;
        GoalPoint[i] = DPoint(FIELDLENGTH/2,-100.0+40*i);	//
        candidateForSelect[i] = 0;//
    }
    GoalPoint[0]=DPoint(FIELDLENGTH/2,-90.0);
    GoalPoint[5]=DPoint(FIELDLENGTH/2,90.0);
    //
    for (int j=1;j<GoalPointNum-1;j++)
    {
        for(int i=0;i<OBSRECORDNUM;i++)
        {
            tempIsInParallelogram  = 0;
            tempIsInParallelogram  = IsInRectangle(DPoint(ballGlobalpos.x_,ballGlobalpos.y_-20.0),GoalPoint[j-1],GoalPoint[j+1],
                                                   DPoint(ballGlobalpos.x_,ballGlobalpos.y_+20.0),worldmodelinfo_->obs_pos_[i]);
            if(tempIsInParallelogram)
            {
                CannotPassRobot[j]++;   //
            }
        }
    }
    for(int j=1;j<GoalPointNum-1;j++)
    {
        if (CannotPassRobot[j]==0)
            noObsTime[j]++;
        else
            noObsTime[j] = 0;
    }//
    for(int j=1;j<GoalPointNum-1;j++)//È·ÈÏ×Êžñ
    {
        if (noObsTime[j]>=3)
        {
            candidateForSelect[j]=1;
        }
        else
            candidateForSelect[j]=0;
    }
    if (candidateForSelect[lastSelectedNum]==1)//ÉÏŽÎµÄµã±ŸÖÜÆÚ»¹ÓÐ×Êžñ£¬ÄÇŸÍÑ¡ËûÀ²£¡
    {

        switch(lastSelectedNum)
        {
        case 1:
            target = GoalPoint[lastSelectedNum];
            rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[1])-thetaof2p2(ballGlobalpos,GoalPoint[0]));
            if (candidateForSelect[2]==1)// ³¢ÊÔÓÐ¶àÉÙÊÇÂú×ãÉäÃÅÌõŒþµÄ zzw ? 16:40 2012-3-8  žÐŸõÓÐµãÎÊÌâ  £¬  ²»ÄÜ¹»×îŽó»¯
            {
                if (candidateForSelect[3]==1)
                {
                    if (candidateForSelect[4]==1)
                    {
                        leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[5])-thetaof2p2(ballGlobalpos,GoalPoint[1]));
                    }
                    else
                        leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[4])-thetaof2p2(ballGlobalpos,GoalPoint[1]));
                }
                else
                    leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[3])-thetaof2p2(ballGlobalpos,GoalPoint[1]));
            }
            else
                leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[2])-thetaof2p2(ballGlobalpos,GoalPoint[1]));
            break;
        case 2:
            target = GoalPoint[lastSelectedNum];
            if (candidateForSelect[1]==1)
            {
                rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[2])-thetaof2p2(ballGlobalpos,GoalPoint[0]));
            }
            else
                rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[2])-thetaof2p2(ballGlobalpos,GoalPoint[1]));
            if (candidateForSelect[3]==1)
            {
                if (candidateForSelect[4]==1)
                {
                    leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[5])-thetaof2p2(ballGlobalpos,GoalPoint[2]));
                }
                else
                    leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[4])-thetaof2p2(ballGlobalpos,GoalPoint[2]));
            }
            else
                leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[3])-thetaof2p2(ballGlobalpos,GoalPoint[2]));
            break;
        case 3:
            target = GoalPoint[lastSelectedNum];
            if (candidateForSelect[2]==1)
            {
                if (candidateForSelect[1]==1)
                {
                    rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[3])-thetaof2p2(ballGlobalpos,GoalPoint[0]));
                }
                else
                    rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[3])-thetaof2p2(ballGlobalpos,GoalPoint[1]));
            }
            else
                rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[3])-thetaof2p2(ballGlobalpos,GoalPoint[2]));
            if (candidateForSelect[4]==1)
            {
                leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[5])-thetaof2p2(ballGlobalpos,GoalPoint[3]));
            }
            else
                leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[4])-thetaof2p2(ballGlobalpos,GoalPoint[3]));
            break;
        case 4:
            target = GoalPoint[lastSelectedNum];
            if (candidateForSelect[3]==1)
            {
                if (candidateForSelect[2]==1)
                {
                    if (candidateForSelect[1]==1)
                    {
                        rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[4])-thetaof2p2(ballGlobalpos,GoalPoint[0]));
                    }
                    else
                        rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[4])-thetaof2p2(ballGlobalpos,GoalPoint[1]));
                }
                else
                    rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[4])-thetaof2p2(ballGlobalpos,GoalPoint[2]));
            }
            else
                rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[4])-thetaof2p2(ballGlobalpos,GoalPoint[3]));

            leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[5])-thetaof2p2(ballGlobalpos,GoalPoint[4]));
            break;
        default:
            break;
        }
        return true; //
    }
    else
    {
        candidataCase = candidateForSelect[4]*1000+candidateForSelect[3]*100+candidateForSelect[2]*10+candidateForSelect[1];
        switch(candidataCase) //  zzw ? 16:44 2012-3-8
        {
        case 0000:
            return false;  // ?
            break;
        case 0001:
            target = GoalPoint[1];
            rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[1])-thetaof2p2(ballGlobalpos,GoalPoint[0]));
            leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[2])-thetaof2p2(ballGlobalpos,GoalPoint[1]));
            lastSelectedNum = 1;
            break;
        case 0010:
            target = GoalPoint[2];
            rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[2])-thetaof2p2(ballGlobalpos,GoalPoint[1]));
            leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[3])-thetaof2p2(ballGlobalpos,GoalPoint[2]));
            lastSelectedNum = 2;
            break;
        case 0011:
        case 1011:
            target = GoalPoint[2];
            rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[2])-thetaof2p2(ballGlobalpos,GoalPoint[0]));
            leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[3])-thetaof2p2(ballGlobalpos,GoalPoint[2]));
            lastSelectedNum = 2;
            break;
        case 0100:
            target = GoalPoint[3];
            rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[3])-thetaof2p2(ballGlobalpos,GoalPoint[2]));
            leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[4])-thetaof2p2(ballGlobalpos,GoalPoint[3]));
            lastSelectedNum = 3;
            break;
        case 0101:
            if (lastSelectedNum == 4)
            {
                target = GoalPoint[3];
                rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[3])-thetaof2p2(ballGlobalpos,GoalPoint[2]));
                leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[4])-thetaof2p2(ballGlobalpos,GoalPoint[3]));
                lastSelectedNum = 3;
            }
            else
            {
                target = GoalPoint[1];
                rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[1])-thetaof2p2(ballGlobalpos,GoalPoint[0]));
                leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[2])-thetaof2p2(ballGlobalpos,GoalPoint[1]));
                lastSelectedNum = 1;
            }
            break;
        case 0110:
            if (lastSelectedNum == 4)
            {
                target = GoalPoint[3];
                rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[3])-thetaof2p2(ballGlobalpos,GoalPoint[1]));
                leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[4])-thetaof2p2(ballGlobalpos,GoalPoint[3]));
                lastSelectedNum = 3;
            }
            else
            {
                target = GoalPoint[2];
                rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[2])-thetaof2p2(ballGlobalpos,GoalPoint[1]));
                leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[4])-thetaof2p2(ballGlobalpos,GoalPoint[2]));
                lastSelectedNum = 2;
            }
            break;
        case 0111:
            target = GoalPoint[2];
            rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[2])-thetaof2p2(ballGlobalpos,GoalPoint[0]));
            leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[4])-thetaof2p2(ballGlobalpos,GoalPoint[2]));
            lastSelectedNum = 2;
            break;
        case 1000:
            target = GoalPoint[4];
            rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[4])-thetaof2p2(ballGlobalpos,GoalPoint[3]));
            leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[5])-thetaof2p2(ballGlobalpos,GoalPoint[4]));
            lastSelectedNum = 4;
            break;
        case 1001:
            if (lastSelectedNum == 3)
            {
                target = GoalPoint[4];
                rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[4])-thetaof2p2(ballGlobalpos,GoalPoint[3]));
                leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[5])-thetaof2p2(ballGlobalpos,GoalPoint[4]));
                lastSelectedNum = 4;
            }
            else
            {
                target = GoalPoint[1];
                rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[1])-thetaof2p2(ballGlobalpos,GoalPoint[0]));
                leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[2])-thetaof2p2(ballGlobalpos,GoalPoint[1]));
                lastSelectedNum = 1;
            }
            break;
        case 1010:
            target = GoalPoint[2];
            rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[2])-thetaof2p2(ballGlobalpos,GoalPoint[1]));
            leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[3])-thetaof2p2(ballGlobalpos,GoalPoint[2]));
            lastSelectedNum = 2;
            break;
        case 1100:
        case 1101:
            target = GoalPoint[3];
            rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[3])-thetaof2p2(ballGlobalpos,GoalPoint[2]));
            leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[5])-thetaof2p2(ballGlobalpos,GoalPoint[3]));
            lastSelectedNum = 3;
            break;
        case 1110:
            target = GoalPoint[3];
            rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[3])-thetaof2p2(ballGlobalpos,GoalPoint[1]));
            leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[5])-thetaof2p2(ballGlobalpos,GoalPoint[3]));
            lastSelectedNum = 3;
            break;
        case 1111:
            target = GoalPoint[3];
            rightdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[3])-thetaof2p2(ballGlobalpos,GoalPoint[0]));
            leftdelta = abs(thetaof2p2(ballGlobalpos,GoalPoint[5])-thetaof2p2(ballGlobalpos,GoalPoint[3]));
            lastSelectedNum = 3;
            break;

        default:
            return false;
            break;
        }
        return true;
    }
}

bool
Strategy::IsLocationInOurField(DPoint location)
{
    bool rtvl = false;
    static bool inourfield = false;

    if(!inourfield && location.x_ < 0)
    {

       inourfield  =  true;

    }
    else if(inourfield && location.x_ < LOCATIONERROR)
    {
        inourfield =  true;
    }
    else
    {
        inourfield =  false;
    }
    rtvl =  inourfield;
    return rtvl;
}
bool
Strategy::IsLocationInOppField(DPoint location)
{
    bool rtvl = false;
    static bool inoppfield = false;

    if(!inoppfield && location.x_ > 0)
    {
       inoppfield  =  true;

    }
    else if(inoppfield && location.x_ >  -LOCATIONERROR)
    {
        inoppfield =  true;
    }
    else
    {
        inoppfield =  false;
    }
    rtvl = inoppfield;

    return rtvl;

}
bool
Strategy::IsLocationInOurPenalty(DPoint location)
{
    bool rtvl = false;
    static bool inourpenalty = false;
    float  ourpenltyareaX =  -900.0 + OurPenltyAreaX;
    float  ourpenltyareaY1 =  -OurPenltyAreaY;
    float  ourpenltyareaY2 =   OurPenltyAreaY;


    if(!inourpenalty)
    {
        if(location.x_ < ourpenltyareaX &&  location.y_ > ourpenltyareaY1 && location.y_ <ourpenltyareaY2 )
        {
            inourpenalty = true;
        }
        else
            inourpenalty = false;
    }
    else if(inourpenalty)
    {
        if(location.x_ < (ourpenltyareaX +LOCATIONERROR) &&  location.y_ > (ourpenltyareaY1-LOCATIONERROR) && location.y_ < (ourpenltyareaY2+LOCATIONERROR))
        {
            inourpenalty = true;
        }
        else
            inourpenalty = false;
    }

    rtvl =  inourpenalty;
    return rtvl;
}

bool
Strategy::IsLocationInOppPenalty(DPoint location)
{

    bool rtvl = false;
    static bool inopppenalty = false;
    float  opppenltyareaX =  900.0 - OppPenaltyAreaX;
    float  opppenltyareaY1 =  -OppPenaltyAreaY;
    float  opppenltyareaY2 =   OppPenaltyAreaY;


    if(!inopppenalty)
    {
        if(location.x_ > opppenltyareaX &&  location.y_ > opppenltyareaY1 && location.y_ <opppenltyareaY2 )
        {
            inopppenalty = true;
        }
        else
            inopppenalty = false;
    }
    else if(inopppenalty)
    {
        if(location.x_ > (opppenltyareaX - LOCATIONERROR) &&  location.y_ > (opppenltyareaY1-LOCATIONERROR) && location.y_ < (opppenltyareaY2+LOCATIONERROR))
        {
            inopppenalty = true;
        }
        else
            inopppenalty = false;
    }

    rtvl =  inopppenalty;
    return rtvl;
}
bool Strategy::IsLocationInField(DPoint location)
{

    bool rtvl = false;
    static bool isinfiled = false;


    if(!isinfiled && abs(location.x_) < FIELDLENGTH/2 && abs(location.y_) < FIELDWIDTH/2)
    {
        isinfiled = true;
    }
    else if(isinfiled && abs(location.x_) < FIELDLENGTH/2 + LOCATIONERROR && abs(location.y_) < FIELDWIDTH/2+LOCATIONERROR)
    {
        isinfiled = true;
    }
    else
    {
        isinfiled = false;
    }

    rtvl = isinfiled;
    return rtvl;
}

bool
Strategy::IsLocationInOppGoalArea(DPoint location)
{
    bool rtvl = false;
    static bool inoppgoalarea = false;
    float  oppgoalkeeperreaX =  900.0 - OppGoalKeeperAreaX;
    float  oppgoalkeeperreaY1 =  -OppGoalKeeperAreaY;
    float  oppgoalkeeperreaY2 =   OppGoalKeeperAreaY;


    if(!inoppgoalarea)
    {
        if(location.x_ > oppgoalkeeperreaX &&  location.y_ > oppgoalkeeperreaY1 && location.y_ <oppgoalkeeperreaY2 )
        {
            inoppgoalarea = true;
        }
        else
            inoppgoalarea = false;
    }
    else if(inoppgoalarea)
    {
        if(location.x_ > (oppgoalkeeperreaX - LOCATIONERROR) &&  location.y_ > (oppgoalkeeperreaY1-LOCATIONERROR) && location.y_ < (oppgoalkeeperreaY2+LOCATIONERROR))
        {
            inoppgoalarea = true;
        }
        else
            inoppgoalarea = false;
    }

    rtvl =  inoppgoalarea;
    return rtvl;
}


void
Strategy::clearActiveState()
{

   stucktime_ = 0;
   NeedEvaluat = false;
   dynamic_shoot_state_ = false;
   dynamic_shoot_count_ = 0;
   quick_shoot_state_ = false;
   quick_shoot_count_ = 0;
   m_plan_.m_behaviour_.app_vx_ = 0;
   m_plan_.m_behaviour_.app_vy_ = 0;
   m_plan_.m_behaviour_.app_w_  = 0;

}
void
Strategy::stopGame()
{
   m_plan_.m_behaviour_.app_vx_ = 0;
   m_plan_.m_behaviour_.app_vy_ = 0;
   m_plan_.m_behaviour_.app_w_  = 0;
   running = false;
   game_begin_ = false;
   clearActiveState();
}
void
Strategy::runGame()
{
    //dribble_state_ = checkRobotDrible();

     DPoint target;
     float  target_orientation;
     DPoint rel_target;
     float  rel_target_ori;
     float  deldis = 10;
     float  distance2recordball;

     switch(worldmodelinfo_->game_ctrl_)
     {
     case CTRL_ZONEDEF:    // £¡


          target= worldmodelinfo_->target_;
          target_orientation = worldmodelinfo_->target_orientation_;
          rel_target =  pglobal2rel(worldmodelinfo_->robot_pos_,worldmodelinfo_->robot_ori_.radian_,target);
          rel_target_ori  = target_orientation - worldmodelinfo_->robot_ori_.radian_;
          m_plan_.m_behaviour_.move2Position(1.75,1.5,target,MAXVEL);
          m_plan_.m_behaviour_.rotate2AbsOrienation(1.75,1.5,target_orientation,10);
         // m_plan_.positionAvoidObs(rel_target,rel_target_ori,deldis,5*SINGLEPI_CONSTANT/180);

          break;
     case CTRL_MOVET:    //
         target= worldmodelinfo_->target_;
         target_orientation = worldmodelinfo_->target_orientation_;

         rel_target =  pglobal2rel(worldmodelinfo_->robot_pos_,worldmodelinfo_->robot_ori_.radian_,target);
         rel_target_ori  = target_orientation - worldmodelinfo_->robot_ori_.radian_;

         //m_plan_.positionAvoidObs(rel_target,rel_target_ori,deldis,5*SINGLEPI_CONSTANT/180);
         m_plan_.m_behaviour_.move2Position(1.75,1.5,target,MAXVEL);
         m_plan_.m_behaviour_.rotate2AbsOrienation(1.75,1.5,target_orientation,10);
         running=true;
         break;

     case CTRL_GOALKEEP://ÊØÃÅÔ±Ã»ÓÐÄ¿±êµã£¬×Ô¶¯ÅÜ;

         target=DPoint(-900,0); //moveto;
        // Goalie_GlobalforSV();  // ~~~~
         break;

     case CTRL_ATTACK:
         activeDecisionMaking();
         break;


     case CTRL_DEFENCE:

         target=defencepointCalculate();

        // if(worldmodelinfo_->ball_info_state_ != NOTSEEBALL )
        //     target_orientation =  thetaof2p(worldmodelinfo_->robot_pos_,worldmodelinfo_->ball_pos_);
        // else
         target_orientation =  thetaof2p(worldmodelinfo_->robot_pos_,worldmodelinfo_->ball_pos_);

         m_plan_.m_behaviour_.move2Position(2.75,1.5,target,MAXVEL);
         m_plan_.m_behaviour_.rotate2AbsOrienation(1.785,1.2,target_orientation,10);		//Target = m_MyMath.Global2Rel(m_WorldModel->MyInfo.GlobalPos,m_WorldModel->MyInfo.BodyFacing,target);
         break;


     case CTRL_PASS:
         target= worldmodelinfo_->target_;
         kickball4Coop(target);
         break;

     case CTRL_CATCH:    //  µ÷ÓÃ×¥Çòº¯Êý

         target= worldmodelinfo_->target_;
         if(game_begin_ == false)
         {
            ballpos_record_ = worldmodelinfo_->ball_pos_;
            distance2ball_record_ = worldmodelinfo_->robot_pos_.distance(ballpos_record_);
            game_begin_ = true;
         }
         if(distance2ball_record_< 340.0 && distance2ball_record_>= 0.0 && worldmodelinfo_->ball_info_state_ != CannotSeeBall)
         {
             distance2recordball = ballpos_record_.distance(worldmodelinfo_->ball_pos_);

             if(distance2recordball >  40)

                     activeCatchBall();

               else
                     stopGame();
         }
         else
         stopGame();
         break;

     case CTRL_PASS_MOVE:
        // PassMoveBehaviour();
         break;

     case CTRL_CATCH_MOVE:
         target=worldmodelinfo_->target_;
        // if(worldmodelinfo_->ball_info_state_ != NOTSEEBALL )
         target_orientation =  thetaof2p(worldmodelinfo_->robot_pos_,worldmodelinfo_->ball_pos_);
         //    else
         //    target_orientation =  thetaof2p(worldmodelinfo_->robot_pos_,m_WorldModel->MsgFromCoach.FuseBall);
         m_plan_.m_behaviour_.move2Position(2.75,1.5,target,MAXVEL);
         m_plan_.m_behaviour_.rotate2AbsOrienation(1.785,1.2,target_orientation,10);
         break;

     case CTRL_CATCH_FOCUS:        //ÊÇ²»ÊÇÓŠžÃŽ«»ØÀŽ£¬Ž«Çò»úÆ÷ÈËËùÔÚÎ»ÖÃ¶ø²»ÊÇœÓÇòµã;
        target= worldmodelinfo_->target_;
        activeCatchBall();
         break;

     case CTRL_ATTENTIONBALL:      //²»¶¯£¬µÈºò¶Ô·œ·¢Çò;  ????  ¶ÔÊÖ·£ÇòÕŸÎ»
         stopGame();
         break;
     case CTRL_SEARCHBALL:     //ÕÒÇò  zzw
         findBall();   // ¿ÉÒÔ°ÑÃ€ÇøÕÒ³öÀŽ
         break;
     case CTRL_STOP:
         running=false;
         stopGame();
         break;
     }
}

