#ifndef ROS_HEADER_FBK_31052022
#define ROS_HEADER_FBK_31052022
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include "ros/subscriber.h"
#include <Common/TimeSeriesTable.h>
#include <message_filters/subscriber.h>
#include <Ros/include/saver_node.h>
#include <OpenSim/Simulation/Model/Model.h>

namespace Ros
{
	class Reshuffler
	{
		public:
			std::vector<int> deshuffle_xput;
			std::vector<std::string> desired_label_order;
			std::vector<std::string> labels;
			size_t xput_size;
//Reshuffler();
//~Reshuffler();
			bool autoset_labelsOK();
			OpenSim::Model * model = nullptr;
			void set(std::vector<std::string> labels);
			void get_labels(ros::NodeHandle nh);
	
		//Reshuffler(OpenSim::Model * model_): model(model_) {}
	};

	class CommonNode:public SaverNode
	{
		public:
			CommonNode(bool Debug=true);
			virtual ~CommonNode() override;
			bool published_labels_at_least_once = false;

			void setModel(OpenSim::Model * model);
			OpenSim::Model * model;
			ros::NodeHandle n;
			ros::NodeHandle nh{"~"};
			
			//ros::Subscriber sub; //input
			message_filters::Subscriber<opensimrt_msgs::CommonTimed> sub;
			message_filters::Subscriber<opensimrt_msgs::PosVelAccTimed> sub_filtered;

			ros::Publisher pub; //output
			ros::Publisher pub_filtered; //output, but filtered
			bool publish_filtered = false;
			void onInit(int num_sinks = 1);
			virtual void callback(const opensimrt_msgs::CommonTimedConstPtr& message) 
			{
				ROS_ERROR_STREAM("callback1 not implemented!");
			}
			virtual void callback_filtered(const opensimrt_msgs::PosVelAccTimedConstPtr& message) 
			{
				ROS_ERROR_STREAM("callback_filtered not implemented!");
			}
			Reshuffler input, output;
		protected:
			ros::ServiceServer outLabelsSrv;
			bool outLabels(opensimrt_msgs::LabelsSrv::Request & req, opensimrt_msgs::LabelsSrv::Response& res );
	};
	template <typename T> std::vector<int> find_matches(std::vector<T> desired, std::vector<T> shuffled)
	{
		std::vector<int> el;
		assert(desired.size() == shuffled.size());
		for (auto d_item: desired)
		{
			for(int i= 0;i<shuffled.size();i++)
			{
				std::string s_item = shuffled[i];
				if (d_item == s_item)
				{
					el.push_back(i);
					break;
				}
			}

		}
		//first lets assert that el has the same length as desired, that is, all shuffled and desired elements are mapped to each other
		assert(el.size() == desired.size());
		// should have in el a vector which maches shuffled to desired.
		// shuffled[el[i]] == desired[i];
		// Let's test it;
		for (int i=0;i<desired.size();i++)
		{
			assert(shuffled[el[i]]==desired[i]);
		}
		return el;
	}	

}

#endif
