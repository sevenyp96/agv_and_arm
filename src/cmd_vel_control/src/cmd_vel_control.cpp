#include "cmd_vel_control.h"


cmd_vel_control::cmd_vel_control()
{
  priority_level=JOY;
  level=2;

  serial::Timeout to = serial::Timeout::simpleTimeout(100);

  sp1.setPort("/dev/ttyUSB0");
  sp2.setPort("/dev/ttyUSB1");

  sp1.setBaudrate(115200);
  sp2.setBaudrate(115200);

  sp1.setTimeout(to);
  sp2.setTimeout(to);
  try
  {
      sp1.open();
      sp2.open();
  }
  catch(serial::IOException& e)
  {
      ROS_ERROR_STREAM("Unable to open port.");
  }

  if(sp1.isOpen() & sp2.isOpen())
  {
      ROS_INFO_STREAM("/dev/ttyUSB0&1 is opened.");
  }
  else
  {
      ROS_INFO_STREAM("Unable to open /dev/ttyUSB0|1 .");
  }
//sub set level,set speed
  sub_joy = nh.subscribe("joy_cmd",10,&cmd_vel_control::Joy_cmd_Callback,this);
  sub_navigation = nh.subscribe("cmd_vel",10,&cmd_vel_control::Cmd_vel_Callback,this);

//pub 
  encoder_pub = nh.advertise<msglist::encoder>("encoder", 1);
}

void cmd_vel_control::Joy_cmd_Callback(const geometry_msgs::Twist::ConstPtr& cmd)
{
  if(cmd->linear.y>0)
  {
  priority_level=JOY;
  level=2;
  }
  if(cmd->linear.y<0)
  {
  priority_level=NONE;
  level=0;
 // ROS_INFO("reset!");
  }

  uint8_t data_to_send1[20];
  uint8_t data_to_send2[20];
  rpm rpm1=convert_speed_to_rpm(cmd->linear.x,cmd->angular.z);
//  std::cout <<"rpm=   "<<rpm1.left <<std::endl;

  set_rpm(data_to_send1,rpm1.left);
  set_rpm(data_to_send2,-1*rpm1.right);  

  sp1.write(data_to_send1, 12);
  sp2.write(data_to_send2, 12);

}

void cmd_vel_control::Cmd_vel_Callback(const geometry_msgs::Twist::ConstPtr& cmd)
{
  uint8_t data_to_send1[20];
  uint8_t data_to_send2[20];


  rpm rpm1=convert_speed_to_rpm(cmd->linear.x,cmd->angular.z);
//  std::cout <<"rpm=   "<<rpm1.left <<std::endl;

  set_rpm(data_to_send1,rpm1.left);
  set_rpm(data_to_send2,-1*rpm1.right);  

  sp1.write(data_to_send1, 12);
  sp2.write(data_to_send2, 12);
  

}

cmd_vel_control::rpm cmd_vel_control::convert_speed_to_rpm(double linear,double angular)
{
  double wheel_length=0.25;//l=0.25m
  double n_deduce=11.7;    //maybe12
  double pi=3.14159;
  double wheel_radius=0.15;//r=0.15m
	
  double v_left =linear+angular*wheel_length;
  double v_right=linear-angular*wheel_length;
  int  rpm_left =0;
  int  rpm_right=0;
  rpm_left =v_left *60*n_deduce/(2*pi*wheel_radius);
  rpm_right=v_right*60*n_deduce/(2*pi*wheel_radius);
  rpm rpm1={0,0};
  rpm1.left=rpm_left;
  rpm1.right=rpm_right;
  return rpm1;
}

void cmd_vel_control::set_rpm(uint8_t s[20],int rpm)
{	
  if(rpm>999)
  {
    ROS_ERROR_STREAM("rpm set too high! shutdown!");
    nh.shutdown();
  }
  int sign_v=1;//positive
  if(rpm<0)
  {
	rpm=-1*rpm;
    sign_v=-1;//negative
  }
  
  int a,b,c;//100,10,1
  a=rpm/100;
  b=rpm/10-a*10;
  c=rpm-b*10-a*100;
  a+=48;
  b+=48;
  c+=48;//convert to ASCII
/*  std::cout <<"a=   "<<a <<std::endl;
  std::cout <<"b=   "<<b <<std::endl;
  std::cout <<"c=   "<<c <<std::endl;
*/ //test
   s[0] = 0x21;
   s[1] = 0x4D;
   s[2] = 0x20;
   s[3] = 0x00;
   s[4] = 0x00;
   s[5] = 0x00;
   s[6] = 0x20;
   s[7] = 0x00;
   s[8] = 0x00;
   s[9] = 0x00;
   s[10] = 0x00;
   s[11] = 0x0d;  
  
  if(sign_v>0)
  {
	s[3] = a;
    s[4] = b;
    s[5] = c;
    s[6] = 0x20;
    s[7] = 0x2D;
    s[8] = a;
    s[9] = b;
    s[10]= c;
  }

  if(sign_v<0)
  {
	s[3] = 0x2D;
    s[4] = a;
    s[5] = b;
    s[6] = c;
    s[7] = 0x20;
    s[8] = a;
    s[9] = b;
    s[10]= c;
  }


/*if(rpm==-70)
{
  s[3] = 0x2D;
  s[4] = 0x30;
  s[5] = 0x37;
  s[6] = 0x30;
  s[7] = 0x20;
  s[8] = 0x30;
  s[9] = 0x37;
  s[10] = 0x30;
}
  if(rpm==70)
{
  s[3] = 0x30;
  s[4] = 0x37;
  s[5] = 0x30;
  s[6] = 0x20;
  s[7] = 0x2D;
  s[8] = 0x30;
  s[9] = 0x37;
  s[10] = 0x30;
}*/ //important reference,don't remove!
}

void cmd_vel_control::send_encoder()
{
  uint8_t s_buffer1[20];
  s_buffer1[0]=0x3f;
  s_buffer1[1]=0x43;
  s_buffer1[2]=0x0d;

  uint8_t s_buffer2[20];
  s_buffer2[0]=0x3f;
  s_buffer2[1]=0x43;
  s_buffer2[2]=0x0d;

  sp1.write(s_buffer1,3);
  sp2.write(s_buffer2,3);

  size_t get_c1 = sp1.available();
  size_t get_c2 = sp2.available();

  if(get_c1!=0&&get_c2!=0)
  {
     uint8_t buffer1[1024];
     uint8_t buffer2[1024];

     get_c1 = sp1.read(buffer1, get_c1);
     std_msgs::String serial_data1;
     serial_data1.data=sp1.read(sp1.available());

     get_c2 = sp2.read(buffer2, get_c2);            
     std_msgs::String serial_data2;            
     serial_data2.data=sp2.read(sp2.available());

//     ROS_INFO_STREAM("Read: "<<serial_data1.data);
            
     for(int i=0; i<get_c1; i++){
     std::cout << std::hex << (buffer1[i] & 0xff) << " ";
     }
     std::cout << std::endl;

     for(int i=0; i<get_c2; i++){
     std::cout << std::hex << (buffer2[i] & 0xff) << " ";
     }
     std::cout << std::endl;

/*     for(int i=0; i<get_c1; i++){
     std::cout << buffer1[i]<< " ";
     }
     std::cout << std::endl;
*/    
	  if(buffer1[3]==0x43&&buffer2[3]==0x43)
	{ 
     encoder encoder1=get_encoder(buffer1);
     encoder encoder2=get_encoder(buffer2); 
	 encoder2.round1=(-1)*encoder2.round1;		
	 encoder2.round2=(-1)*encoder2.round2;

  	 std::cout << std::dec  <<"encoder1 round1:  " <<encoder1.round1<< "  ";
  	 std::cout << std::dec  <<"encoder1 round2:  " <<encoder1.round2<< "  ";
     std::cout<<std::endl;

  	 std::cout << std::dec  <<"encoder2 round1:  " <<encoder2.round1<< "  ";
  	 std::cout << std::dec  <<"encoder2 round2:  " <<encoder2.round2<< "  ";
     std::cout<<std::endl;

    		msglist::encoder encoder_msgs;
    		encoder_msgs.current_time=ros::Time::now();
    		encoder_msgs.encoder_left=encoder1.round2;
    		encoder_msgs.encoder_right=encoder2.round1;
    		encoder_msgs.count=encoder_count;     //zheng to fu
     		encoder_pub.publish(encoder_msgs);
     		encoder_count++;	
	}
   }
}

cmd_vel_control::encoder cmd_vel_control::get_encoder(uint8_t buffer1[20])
{
	 int n1=1,n2=1,n11=1,n21=1;
     int w1=0,w2=0;
     int total1=0,total2=0;
     int number[10];
     if(buffer1[5]==45)
     {
     		n1=-1;
     		n11=0;
     }
     if(buffer1[4]==61)
     {
     		for(int i=6-n11;buffer1[i]!=58;i++)
     		{
     			number[i-6+n11]=buffer1[i]-48;
     			//std::cout << std::dec  <<  number[i-6+n11]<< " ";
     			w1=i-(6-n11)+1;
     		}
     		
     		for(int i=0;i<w1;i++)
     		{
     			total1+=number[w1-i-1]*pow(10,i);
     		}
     		
     		total1*=n1;
//     		std::cout << std::dec  <<"round1:  " <<total1 << "  ";
     		
     		int b1=6-n11+w1+1;
     		
     	  if(buffer1[b1]==45)
    	  {
     		 	n2=-1;
     		 	n21=0;
    	  }
    	  
    	  int b2=b1+1-n21;
    	 
    	  for(int i=b2;buffer1[i]!=13;i++)
     		{
     			number[i-b2]=buffer1[i]-48;
     			//std::cout << std::dec  <<  number[i-b2]<< " ";
     			w2=i-b2+1;
     		}
    	 	
    	 	for(int i=0;i<w2;i++)
     		{
     			total2+=number[w2-i-1]*pow(10,i);
     		}
     		
     		total2*=n2;
//     		std::cout << std::dec<<"round2:  "  << total2 << "  "<<std::endl; 
      }
	encoder encoder_f;
	encoder_f.round1=total1;
	encoder_f.round2=total2;
    return encoder_f;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmd_vel_control_node");
  cmd_vel_control control1;
  ros::Rate loop_rate(10); 		
  		
    while(ros::ok())
    {
	control1.send_encoder();
	ros::spinOnce();
    loop_rate.sleep();
    }
//   sp3.close();
 
  ros::spin();
  return 0;
}
