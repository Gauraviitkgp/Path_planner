#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include "mavros_msgs/State.h"
#include "sensor_msgs/Imu.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <string>
#include <cmath>

#include "geometry_eigen_conversions.h"


using std::list;
using namespace std;

namespace hedwig_mpc 
{
	geometry_msgs::TwistStamped::ConstPtr hvel;
	geometry_msgs::PoseStamped::ConstPtr hpos;
	mavros_msgs::State::ConstPtr hstatus;
	sensor_msgs::Imu::ConstPtr himu;

	geometry_msgs::Quaternion ORIENT;
	geometry_msgs::Vector3 BODY_RATES;
	float THRUST;

	mavros_msgs::AttitudeTarget COMMAND;


	bool FIRST_RUN1,FIRST_RUN2,FIRST_RUN3,FIRST_RUN4;


	// ros::Publisher pub_thrust;
	// ros::Publisher pub_angles;
	ros::Publisher pub_controls;

	enum STATE 
	{
		kPosX = 0,
		kPosY = 1,
		kPosZ = 2,
		kOriW = 3,
		kOriX = 4,
		kOriY = 5,
		kOriZ = 6,
		kVelX = 7,
		kVelY = 8,
		kVelZ = 9
	};

	enum INPUT
	{
		kThrust = 0,
		kRateX = 1,
		kRateY = 2,
		kRateZ = 3
	};

	class hedwig_command
	{
	public:
		// geometry_msgs::TwistStamped hgeometry;
		geometry_msgs::Quaternion horientation;
		geometry_msgs::Vector3 hbody_rates;
		// geometry_msgs::PoseStamped hposition;
		float hthrust;
		float throttle;
		float attitude;
		bool armed;
		float m,c; // Used for extrapolating the c and accelaration value curves

		hedwig_command(){}

		// Setting directtly as messages
		hedwig_command(geometry_msgs::Quaternion horientation1, geometry_msgs::Vector3 hbody_rates1, float hthrust1)
		{
			horientation=horientation1;
			hbody_rates=hbody_rates1;
			hthrust=hthrust1;
			attitude=0;
			armed=false;
		}
		hedwig_command(geometry_msgs::Vector3 hbody_rates1, float hthrust1)
		{
			horientation.x=0;
			horientation.y=0;
			horientation.z=0;
			horientation.w=0;
			hbody_rates=hbody_rates1;
			hthrust=hthrust1;
			armed=false;
		}

		void set_command(geometry_msgs::Quaternion horientation1, geometry_msgs::Vector3 hbody_rates1, float hthrust1)
		{
			horientation=horientation1;
			hbody_rates=hbody_rates1;
			hthrust=hthrust1;
			armed=false;
		}

		void set_command(geometry_msgs::Vector3 hbody_rates1, float hthrust1)
		{
			horientation.x=0;
			horientation.y=0;
			horientation.z=0;
			horientation.w=0;
			hbody_rates=hbody_rates1;
			hthrust=hthrust1;
			armed=false;
		}
		void zero()
		{
			hthrust=0;
			horientation.x=0;
			horientation.y=0;
			horientation.z=0;
			horientation.w=1;
			hbody_rates.x=0;
			hbody_rates.y=0;
			hbody_rates.z=0;
		}
		void set_thrust(float th)
		{
			hthrust=th;
		}
		void set_angle(float th[3])
		{
			hbody_rates.x=th[0];
			hbody_rates.y=th[1];
			hbody_rates.z=th[2];
		}
		void set_mc(float z0, float z1, float acc0, float acc1)
		{
			// (z-z0)=(acc-acc0)*(z1-z0)/(acc1-acc0)
			m=(z1-z0)/(acc1-acc0);
			c=z0-acc0*(m);
		}

		void convert_to_throttle(float th)
		{
			throttle=th*m+c;
		}

		float get_thrust()
		{
			return hthrust;
		}

		geometry_msgs::Vector3 get_bodyrates()
		{
			return hbody_rates;
		}

		geometry_msgs::Quaternion get_orientation()
		{
			return horientation;
		}


	};

	class hedwig_state
	{
	public:
		geometry_msgs::PoseStamped hstate;
		geometry_msgs::TwistStamped hvel;
		sensor_msgs::Imu himu;
		ros::Time timestamp;
		float hPosX,hPosY,hPosZ,hOriW,hOriX,hOriY,hOriZ,hVelX,hVelY,hVelZ;
		float hAccX,hACCY,hACCz;

		hedwig_state(geometry_msgs::PoseStamped hstat, geometry_msgs::TwistStamped velocit)
		{
			hstate=hstat;
			hvel=velocit;
			
			timestamp=hstate.header.stamp;

			hPosX=hstate.pose.position.x;
			hPosY=hstate.pose.position.y;
			hPosZ=hstate.pose.position.z;
			
			hOriW=hstate.pose.orientation.w;
			hOriX=hstate.pose.orientation.x;
			hOriY=hstate.pose.orientation.y;
			hOriZ=hstate.pose.orientation.z;

			hVelX=hvel.twist.linear.x;
			hVelY=hvel.twist.linear.y;
			hVelZ=hvel.twist.linear.z;
		}

		hedwig_state(){};

		void set_state(geometry_msgs::PoseStamped hstat, geometry_msgs::TwistStamped velocit)
		{
			hstate=hstat;
			hvel=velocit;
			
			timestamp=hstate.header.stamp;

			hPosX=hstate.pose.position.x;
			hPosY=hstate.pose.position.y;
			hPosZ=hstate.pose.position.z;
			
			hOriW=hstate.pose.orientation.w;
			hOriX=hstate.pose.orientation.x;
			hOriY=hstate.pose.orientation.y;
			hOriZ=hstate.pose.orientation.z;

			hVelX=hvel.twist.linear.x;
			hVelY=hvel.twist.linear.y;
			hVelZ=hvel.twist.linear.z;
		}

	};

	class hthnatt_estimator
	{
	public:
		bool arm,success;
		mavros_msgs::State status;

		bool first_run,LIFTOFF=false,got_first_val, got_second_val, got_5_val;

		//Parameter for Total number of values
		#define TOTAL 5
		
		mavros_msgs::SetMode offb_set_mode;
		mavros_msgs::SetMode rtl_set_mode;
		mavros_msgs::CommandBool arm_cmd;
		mavros_msgs::AttitudeTarget COMMAND;
		hedwig_command HCOMM;
		ros::ServiceClient set_mode;
		ros::ServiceClient set_arm;
		
		hedwig_state hestate;
		sensor_msgs::Imu himu;

		ros::Time last_request;

		
		bool HOVER,PREFLIGHTCHECKED;

		float velz_values[TOTAL], th_values[TOTAL]; 

		float M,C; //thr=vel*M+C;
		bool achevied=false;

		//Parameters For PID
		float Kp;
		float Ki;
		float Kd;
		float delT;//Fine Parameter

		hthnatt_estimator()
		{
			//Parameters
			PREFLIGHTCHECKED=false; //true for avoiding Pre-flight checks, false for not
			HOVER=true;
			M=0.01276;
			C=0.561566;
			delT=0.25;
			Kp=0.007500;
			Ki=0.000000;
			Kd=0.014036;
			// TOTAL=5;

			success=false;first_run=true;got_first_val=false;got_second_val=false;got_5_val=false;
			
			offb_set_mode.request.custom_mode = "OFFBOARD";
			rtl_set_mode.request.custom_mode = "AUTO.RTL";
			last_request = ros::Time::now();
			COMMAND.header.frame_id="";
			HCOMM.zero();
		}

	
		void init()
		{
			
			if(first_run)
			{
				cout<<"\n===========================================\nINITIALIZING THROTTLE AND ATTITUDE ESTIMATOR\n===========================================\n";
				ros::Duration(0.5).sleep();
				cout<<"\033[1;34mRunning Pre-Flight Checks\033[0m\n";
				
				cout<<"\nChecking Flight Controller Unit Connection status\n";
				ros::Duration(1).sleep();
				if(status.connected)
					cout<<"\t\033[0;32mConnected: True\033[0m\n";
				else
					cout<<"\t\033[0;31mConnected: False Please Connect the Flight Controller Unit\033[0m\n";


				cout<<"\nChecking Guided Status\n";
				ros::Duration(1).sleep();
				if(status.guided)
					cout<<"\t\033[0;32mGuided: True\033[0m\n";
				else
					cout<<"\t\033[0;31mGuided: False\033[0m\n";


				cout<<"\nChecking Arm Status\n";
				ros::Duration(1).sleep();
				if(status.armed)
				{
					arm=true;
					cout<<"\t\033[0;31mArmed: True\033[0m\n";
				}
				else
				{
					arm=false;
					cout<<"\t\033[0;32mArmed: False\033[0m\n";
				}


				cout<<"\nChecking Mode of operation\n";
				ros::Duration(1).sleep();
				cout<<"\tMode: "<<status.mode<<"\n";
			

				
				if((status.mode).compare("AUTO.RTL")!=0||((status.mode).compare("AUTO.RTL")!=0&& status.armed==true))
				{
					cout<<"\033[1;31mNot landed, need to Land first. Proceeding to land\033[1m\n";
					if(set_mode.call(rtl_set_mode))
				 		cout<<"\033[0;32mRTL Enabled\033[0m\n";
				 	else
				 		cout<<"\033[0;31mNot able to send commands\033[0m\n";
				}
				else
				{
					cout<<"\033[1;32mLand check passed\033[1m\n";
				}
			}
			first_run=false;

			if(status.guided==false)
			{
				cout<<"\033[1;31m Waiting For guided connection\033[0m\n";
			}
			else if(status.connected==false)
			{
				cout<<"\033[1;31m Waiting For FCU connection\033[0m\n";
			}
			else if((status.mode).compare("AUTO.RTL")!=0 || status.armed==true)
			{
				cout<<"\033[0;33m Waiting to land\033[0m\n";
			}
			else
			{
				cout<<"\n\033[1;32mAll Pre-flight test Passed Proceeding for next step\033[0m\n";
				PREFLIGHTCHECKED=true;
				ros::Duration(0.5).sleep();

				cout<<"\n===========================================\nINITIALIZING ESTIMATOR\n===========================================\n";
			}
		}

		void initilize_step()
		{
			cout<<"\033[1;34mSetting Mode to offboard\033[0m\n";
			if(set_mode.call(offb_set_mode))
		 		cout<<"\t\033[0;32mOffboard Enabled\033[0m"<<endl;
		 	else
		 		cout<<"\t\033[0;31mNot able to send commands\033[0m"<<endl;

		 	cout<<"\n\033[1;34mPowering up Quad\033[0m"<<endl;
		}

		bool timeelapse(float k)
		{
			if(ros::Time::now() - last_request > ros::Duration(k))
				return true;
			else
				return false;
		}

		void calcmc()
		{
			// (thr-th_values[0])=(vel-velz_values[0])(th_values[1]-th_values[0])/(velz_values[1]-velz_values[0])
			float temp= (th_values[1]-th_values[0])/(velz_values[1]-velz_values[0]);
			M=temp;
			C=-velz_values[0]*temp+th_values[0];
			cout<<"\nFor hover velocity we need to set thrust to: "<<C;
			HCOMM.zero();
			HCOMM.hthrust=C;

			this->updatecommand();
			last_request = ros::Time::now();
		}

		void updatecommand()
		{
			COMMAND.thrust=HCOMM.get_thrust();
			COMMAND.body_rate=HCOMM.get_bodyrates();
			COMMAND.orientation=HCOMM.get_orientation();
		}

		void linearfit()
		{
			int i,j,k,n;
		    n=TOTAL;

		    float a,b;
		    
		    float xsum=0,x2sum=0,ysum=0,xysum=0;                //variables for sums/sigma of xi,yi,xi^2,xiyi etc
		    for (i=0;i<TOTAL;i++)
		    {
		        xsum=xsum+th_values[i];                        //calculate sigma(xi)
		        ysum=ysum+velz_values[i];                        //calculate sigma(yi)
		        x2sum=x2sum+pow(th_values[i],2);                //calculate sigma(x^2i)
		        xysum=xysum+th_values[i]*velz_values[i];                    //calculate sigma(xi*yi)
		    }
		    
		    a=(n*xysum-xsum*ysum)/(n*x2sum-xsum*xsum);            //calculate slope
		    b=(x2sum*ysum-xsum*xysum)/(x2sum*n-xsum*xsum);            //calculate intercept
		    
		    float y_fit[TOTAL];                        //an array to store the new fitted values of y    
		    for (i=0;i<n;i++)
		        y_fit[i]=a*th_values[i]+b;                    //to calculate y(fitted) at given x points
		    
		    cout<<"S.no"<<setw(5)<<"thr"<<setw(19)<<"velz(observed)"<<setw(19)<<"velz(fitted)"<<endl;
		    cout<<"-----------------------------------------------------------------\n";
		    for (i=0;i<n;i++)
		        cout<<i+1<<"."<<setw(8)<<th_values[i]<<setw(15)<<velz_values[i]<<setw(18)<<y_fit[i]<<endl;//print a table of x,y(obs.) and y(fit.)    
		    cout<<"\n\033[1;34mThe linear fit line is of the form:\033[0m\n\n"<<1/a<<" \033[0;33mvelz_values\033[0m + "<<-b/a<<endl;        //print the best fit line
		    M=1/a;
		    C=-b/a;
		}

		void hover_command()
		{
			
			static float error_sum=0;
			static float prev_error=0;
			static float error;
			static float SET=0;

			static int k=0;
			if(k==0)
			{
				HCOMM.hthrust=C;
				this->updatecommand();
	
			}

			if (timeelapse(delT))
			{
				// if(k%12==0)
				// 	SET=0;
				cout<<"\033[1;32mZ velocity is:\033[0m"<<hestate.hVelZ<<endl;
				cout<<"\t\033[1;33mThrust values is:\033[0m"<<HCOMM.get_thrust()<<endl;
				
				cout<<"\t\033[1;35mThrust Desired is:\033[0m"<<SET<<endl;
				error=(SET-hestate.hVelZ);
				error_sum+=error;
				cout<<"\t\t\033[1;31merror:\033[0m"<<error<<"\t\033[1;32mP:\033[0m"<<error*Kp<<"\t\033[1;33mI:\033[0m"<<error_sum*delT*Ki<<"\t\033[1;34mD:\033[0m"<<(error-prev_error)/delT*Kd<<endl;

				HCOMM.hthrust+=error*Kp+error_sum*delT*Ki+(error-prev_error)/delT*Kd;

				prev_error=error;
				
				cout<<endl;

				this->updatecommand();
				k++;
				last_request = ros::Time::now();
			}
		}

		void calcfirst()
		{
			static int k=0;
			int flag=0;

			if(k==0)
			{
				this->initilize_step();
			 	cout<<"\033[0;34mIncreasing thrust by 0.05 in every 1 second\033[0m"<<endl;
			 	k=1;
			}
			
			if(timeelapse(1))
			{
				cout<<"\033[0;34mChecking Liftoff\033[0m"<<endl;
				cout<<"\t\033[0;33mLinear Z velocity value is \033[0m"<<hestate.hVelZ<<endl;
				if(hestate.hVelZ>=0.1 || LIFTOFF==true )
				{
					if(hestate.hPosZ<50)
					{
						cout<<"\t\t\033[1;32mLiftoff Detected\033[0m"<<endl;
						cout<<"\t\tThrust Value is "<<HCOMM.get_thrust()<<endl;

						this->updatecommand();
						last_request = ros::Time::now();
						LIFTOFF=true;
					}
					else
					{
						velz_values[0]=hestate.hVelZ;
						th_values[0]=HCOMM.get_thrust();

						cout<<"\033[0;31mHeight has gone above 50 m, Proceeding to land\033[0m"<<endl;
						if((status.mode).compare("AUTO.RTL")!=0 && set_mode.call(rtl_set_mode))
			 				cout<<"\033[0;32mRTL Enabled\033[0m"<<endl;
			 			
			 			cout<<"\n\033[1;32mStored the first values velz_values[0]:"<<velz_values[0]<<" th_values[0]:"<<th_values[0]<<"\033[0m"<<endl;
						cout<<"Proceeding for next step\n";
						cout<<"\nStoring into LOG FILE"<<endl;
						cout<<"Restarting pre-flight checks"<<endl;

						PREFLIGHTCHECKED=false;
						first_run=false;
						got_first_val=true;
						LIFTOFF==false;
						HCOMM.zero();

						last_request = ros::Time::now();
	
				 	}

				}
				else
				{
					cout<<"\t\033[0;32mNo Liftoff Deteced Increasing thrust\033[0m"<<endl;
					HCOMM.hthrust+=0.05;
					cout<<"\t\t\033[1;33mSetting thrust to \033[0m"<<HCOMM.get_thrust()<<endl;

					this->updatecommand();
					last_request = ros::Time::now();
				}

			}
		}

		void calcsecond()
		{
			static int k=0;
			int flag=0;
			if(k==0)
			{
				cout<<"\033[1;34mSetting Mode to offboard\033[0m\n";
				if(set_mode.call(offb_set_mode))
			 		cout<<"\t\033[0;32mOffboard Enabled\033[0m"<<endl;
			 	else
			 		cout<<"\t\033[0;31mNot able to send commands\033[0m"<<endl;

			 	cout<<"\n\033[1;34mPowering up Quad\033[0m"<<endl;
			 	cout<<"\033[0;34mSetting the thrust Value as "<<th_values[0]-0.05/2<<"\033[0m"<<endl;
			 	HCOMM.hthrust+=th_values[0]-0.05/2;
			 	k=1;
			}
			
			if((timeelapse(1)))
			{
				cout<<"\033[0;34mChecking Liftoff\033[0m"<<endl;
				cout<<"\t\033[0;33mLinear Z velocity value is \033[0m"<<hestate.hVelZ<<endl;
				if(hestate.hVelZ>=0.1 || LIFTOFF==true )
				{
					if(hestate.hPosZ<20)
					{
						cout<<"\t\t\033[1;32mLiftoff Detected\033[0m"<<endl;
						cout<<"\t\tThrust Value is "<<HCOMM.get_thrust()<<endl;

						this->updatecommand();
						last_request = ros::Time::now();
						LIFTOFF=true;
					}
					else
					{
						velz_values[1]=hestate.hVelZ;
						th_values[1]=HCOMM.get_thrust();
						cout<<"\033[0;31mHeight has gone above 20 m, Proceeding to Hover"<<endl;
			 				cout<<"\033[0;32mRTL Enabled\033[0m"<<endl;
			 			

			 			cout<<"\n\033[1;32mStored the second values Velz_values[1]:"<<velz_values[1]<<" th_values[1]:"<<th_values[1]<<"\033[0m"<<endl;
						cout<<"Calculating hover values\n";
						cout<<"\nStoring into LOG FILE"<<endl;
						this->calcmc();
						got_second_val=true;
						last_request = ros::Time::now();

				 	}

				}
				else
				{
					cout<<"\t\033[0;32mNo Liftoff Deteced Increasing thrust\033[0m"<<endl;
					HCOMM.hthrust+=0.01;
					cout<<"\t\t\033[1;33mSetting thrust to \033[0m"<<HCOMM.get_thrust()<<endl;

					this->updatecommand();
					last_request = ros::Time::now();
				}

			}
		}

		void continous()
		{
			static int k=0, counter=0;
			static float prev_z;
			static bool LIFT=false, LIFT2=false;

			int flag=0;
			//Setting initial Mode
			if(k==0)
			{
				this->initilize_step();
			 	cout<<"\033[0;34mIncreasing thrust by 0.05 in every 1 second\033[0m"<<endl;
			 	k=1;
			}

			//Setup of Increasing velocity
			if(!LIFT&&timeelapse(1))
			{
				cout<<"\033[0;34mChecking Liftoff\033[0m"<<endl;
				cout<<"\t\033[0;33mLinear Z velocity value is \033[0m"<<hestate.hVelZ<<endl;
				
				if(hestate.hVelZ<=0.1) //Check if liftoff is not done
				{

					cout<<"\t\033[0;31mNo Liftoff Detected Increasing thrust\033[0m"<<endl;
					HCOMM.hthrust+=0.05; //Increasing thrust.
					cout<<"\t\t\033[1;33mSetting thrust to \033[0m"<<HCOMM.get_thrust()<<endl;

					this->updatecommand();
					last_request = ros::Time::now();
				}
				else //If liftoff is detected
				{
					cout<<"\t\033[1;32mLiftoff Detected\033[0m"<<endl;
					cout<<"\t\t\033[1;33mThrust Value is \033[0m"<<HCOMM.get_thrust()<<endl;
					cout<<"\t\t\033[0;35mDecreasing thrust and fine tuning\033[0m"<<endl;
					
					HCOMM.hthrust-=0.05;
					this->updatecommand();
					last_request = ros::Time::now()+ros::Duration(10);

					LIFT=true;
				}
			}



			if(LIFT&&timeelapse(0.25))
			{
				if(!LIFT2&& timeelapse(3))//Check if liftoff is not done
				{
					if(hestate.hVelZ<=0.1)
					{
						cout<<"\t\033[0;31mNo Liftoff Detected Increasing thrust by 0.0025\033[0m"<<endl;
						HCOMM.hthrust+=0.0025; //Increasing thrust.
						cout<<"\t\t\033[1;33mSetting thrust to \033[0m"<<HCOMM.get_thrust()<<endl;

						this->updatecommand();
						last_request = ros::Time::now();
					}
					else
					{
						cout<<"\t\033[1;32mFirst Liftoff Detected\033[0m"<<endl;
						cout<<"\t\t\033[1;33mThrust Value is \033[0m"<<HCOMM.get_thrust()<<endl;

						// HCOMM.hthrust+=0.0025; //Increasing thrust.
						// cout<<"\t\t\033[1;33mSetting thrust to \033[0m"<<HCOMM.get_thrust()<<endl;

						// this->updatecommand();
						last_request = ros::Time::now();
						LIFT2=true;
					}
				}
				if(LIFT2)
				{
					if(hestate.hVelZ>0.1 && hestate.hVelZ-prev_z>0.0)//If liftoff detected still climbing
					{
						prev_z=hestate.hVelZ;
						last_request = ros::Time::now();
					} 
					else if(hestate.hVelZ>0.1 && hestate.hVelZ-prev_z<=0.0)// If stablized
					{
						cout<<"\t\033[1;32mStoring Values\033[0m"<<endl;
						cout<<"\t\t\033[1;33mThrust Value is \033[0m"<<HCOMM.get_thrust()<<endl;
						cout<<"\t\t\033[1;35mVelocity Value is \033[0m"<<hestate.hVelZ<<endl;
						cout<<"\t\t\t\033[0;33mIncreasing thrust by 0.025 for next value\033[0m"<<endl;

						velz_values[counter]=hestate.hVelZ;
						th_values[counter++]=HCOMM.get_thrust();

						if(counter==TOTAL)
						{
							got_5_val=true;

							for (int i = 0; i < counter; i++)
				 				cout<<"\033[1;32mStored the "<< i<<"th value Vel"<<i<<":"<<velz_values[i]<<" thr"<<i<<":"<<th_values[i]<<"\033[0m"<<endl;
							
				 			this->linearfit();

							if(!HOVER)
							{
								cout<<"\033[0;31mGot 5 Values, Proceeding to land\033[0m"<<endl;
								if((status.mode).compare("AUTO.RTL")!=0 && set_mode.call(rtl_set_mode))
					 				cout<<"\033[0;32mRTL Enabled\033[0m"<<endl;
					 			
					 			cout<<endl;
								HCOMM.zero();
								this->updatecommand();
							}
				 			else
				 			{
								cout<<"\033[0;31mGot 5 Values, Proceeding to hover\033[0m"<<endl;
								HCOMM.hthrust=C;
								this->updatecommand();
							}


							last_request = ros::Time::now()+ros::Duration(7);
						}
						else
						{
							HCOMM.hthrust+=0.0025;
							COMMAND.thrust=HCOMM.get_thrust();
							last_request = ros::Time::now();
							LIFT=true;
						}							
					}
				}
			}
		}

		void estimatecm()
		{
			if(status.armed==false&&(timeelapse(3)))
			{
				cout<<"\033[1;34mTrying to arm the drone\033[0m\n";
				arm_cmd.request.value = true;
				if( set_arm.call(arm_cmd) && arm_cmd.response.success)
				{
                    cout<<"\t\033[0;32mQuad Armed success\033[0m\n";
                }
                cout<<"\033[1;34mSetting Mode to offboard\033[0m\n";
				if(set_mode.call(offb_set_mode))
			 		cout<<"\t\033[0;32mOffboard Enabled\033[0m"<<endl;
			 	else
			 		cout<<"\t\033[0;31mNot able to send commands\033[0m"<<endl;

                last_request = ros::Time::now();
			}

			#if 0
				if(status.armed==true && got_first_val==false)
				this->calcfirst();
			
				if(status.armed==true && got_first_val==true && got_second_val==false)
				this->calcsecond();
			#endif

			#if 1
				if(status.armed==true && got_5_val==false)
					this->continous();
			#endif


			if(HOVER==true&&got_5_val==true)
				this->hover_command();



		}
	};
}