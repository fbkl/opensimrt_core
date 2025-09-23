#include "Ros/include/common_node.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include <memory>


#include "OpenSimUtils.h"

//constructor
Ros::CommonNode::CommonNode(bool Debug)
{
	if( Debug && ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		ros::console::notifyLoggerLevelsChanged();
	}
} 

bool Ros::Reshuffler::autoset_labelsOK()
{
	if (!model)	
		return false;
				auto columnNames = OpenSimRT::OpenSimUtils::getCoordinateNamesInMultibodyTreeOrder(*model);
				std::string columnNamesStr = "";
				for(auto cn:columnNames)
				{
					columnNamesStr+=cn+",";
				}

		ROS_WARN_STREAM(columnNamesStr);
		xput_size = columnNames.size();
		if (xput_size == 0)
			return false;
		labels = columnNames;

return true;

}


void Ros::Reshuffler::get_labels(ros::NodeHandle n)
{
	if (autoset_labelsOK())
		return;
	
	ROS_WARN_STREAM("Could not set labels from model info alone or autoset disabled, will try to setup by params and next to listen for external labels service provider");

	ros::Rate r(1);
	//output_labels = qTable.getColumnLabels();
	opensimrt_msgs::LabelsSrv l;
	//so much complexity to save a little bit of bandwidth... anyway, this needs to me moved to reshuffler. Also publishing and subscribing nodes need to be setup from resshuffler inputs. Dual_sinks need to be then simplified to account for this. 
	n.getParam("input_labels",labels);
	if (labels.size()>0)
		xput_size = labels.size();
	//while(!ros::service::call("in_labels", l))
	auto srv_name = ros::names::resolve("in_labels");
	while(ros::ok() && labels.size()==0)
	{
		if(ros::service::call("in_labels", l))
		{
			labels = l.response.data;
			if (labels.size()==0)
				ROS_ERROR_STREAM("Got labels with size 0! If try to unshuffle data, this will likely fail!");
			ROS_DEBUG_STREAM("got response for labels: "<< labels.size() << " elements.");
			set(labels);
			xput_size = labels.size();
			break;
		}
		ros::spinOnce();
		ROS_INFO_STREAM("Waiting to read input labels from: " << srv_name); 
		r.sleep();
	}

}

void Ros::CommonNode::setModel(OpenSim::Model * model)
{
	input.model = model;
	output.model = model;

}

void Ros::CommonNode::onInit(int num_sinks)
{
	pub = n.advertise<opensimrt_msgs::CommonTimed>("output", 1000);
	if (publish_filtered)
		pub_filtered = n.advertise<opensimrt_msgs::PosVelAccTimed>("output_filtered", 1000);
	outLabelsSrv = nh.advertiseService("out_labels", &CommonNode::outLabels, this);

	if (num_sinks == 1)
	{
		ROS_INFO_STREAM("registering ik callback");
		//register the callback
		sub.subscribe(n, "input",10);
		sub.registerCallback(&CommonNode::callback,this);
		sub_filtered.subscribe(n, "input_filtered",10);
		sub_filtered.registerCallback(&CommonNode::callback_filtered,this);
		//sub = nh.subscribe("input",5, &CommonNode::callback, this);
	}
	else
	{
		ROS_INFO_STREAM("not single_sink, callback isnt registered yet!");
	}
	if (num_sinks > 0)
	{
		input.get_labels(n);
	}
	else if (num_sinks==0)
	{
		ROS_INFO_STREAM("Source node registered (num_sinks = 0)");	    
		if (output.labels.size()==0)
			ROS_WARN_STREAM("I already registered the service for outputting labels, but the labels are not yet set. The downstream nodes will get empty label data!");
	}
	else
	{
		ROS_ERROR_STREAM("Unexpected number of sinks:" << num_sinks);

	}
	Ros::SaverNode::onInit();

}

void Ros::Reshuffler::set(std::vector<std::string> labels)
{
	if (desired_label_order.size() == 0)
	{
		ROS_WARN_STREAM("Label order not set. Data may not be aligned properly and results will be incorrect.");	
	}
	else
	{
		deshuffle_xput = find_matches(desired_label_order,labels);
		ROS_INFO_STREAM("Label order set. Make sure you are not directly accessing elements, but using something like 'raw_input[deshuffle_xput[i]]'");
	}
}

//destructor
Ros::CommonNode::~CommonNode()
{
	// we want to make sure that we clean the parameters set by this node so we can restart it with different parameters if needed and not worry if we left something behind. 
	// This isn't default behaviour though, so we maybe should say something. 
	ROS_INFO_STREAM("Deleting previsouly set node parameters.");
	ros::param::del("~");

}
bool Ros::CommonNode::outLabels(opensimrt_msgs::LabelsSrv::Request & req, opensimrt_msgs::LabelsSrv::Response& res )
{
	if (output.labels.size() == 0)
		ROS_ERROR_STREAM("output_labels variable not set!!!! Your client will not be able to know what is what so this will possibly crash.");
	res.data = output.labels;
	published_labels_at_least_once = true;
	ROS_INFO_STREAM("CALLED LABELS SRV");
	return true;
}


