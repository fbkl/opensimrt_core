#include <Ros/include/saver_node.h>
#include "Ros/include/resolv_dir.h"
#include "opensimrt_msgs/SetFileNameSrv.h"
#include "ros/service_client.h"
#include "ros/service_server.h"
#include <Common/TimeSeriesTable.h>
#include <Common/FileAdapter.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/CSVFileAdapter.h>
#include <memory>

//constructor
Ros::SaverNode::SaverNode(bool Debug): recording_enabled(false), epoch_start(0)
{
	if( Debug && ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		ros::console::notifyLoggerLevelsChanged();
	}
} 
void Ros::SaverNode::onInit()
{
	write_csv = nh.advertiseService("write_csv", &SaverNode::writeCsv, this);
	write_sto = nh.advertiseService("write_sto", &SaverNode::writeSto, this);
	write_all = nh.advertiseService("write", &SaverNode::write, this);
	
	startRecordingSrv 	= nh.advertiseService("start_recording", 	&SaverNode::startRecording, this);
	stopRecordingSrv 	= nh.advertiseService("stop_recording", 	&SaverNode::stopRecording, this);
	clearLoggersSrv 	= nh.advertiseService("clear_loggers", 		&SaverNode::clearLoggers, this);

	setNamePathSrv 		= nh.advertiseService("set_name_and_path", 	&SaverNode::setNamePath, this);
	
	nh.getParam("epoch_start", epoch_start);
	
	// Starts the thread to monitor the epoch time
	check_thread = boost::thread(&SaverNode::timeChecker, this);
}



//destructor
Ros::SaverNode::~SaverNode()
{
	if(!at_least_one_logger_initialized)
		ROS_ERROR_STREAM("You forgot to initialize loggers!");
        
	// Ensure the thread is properly joined
	check_thread.interrupt();
	check_thread.join();
}


bool Ros::SaverNode::startRecording(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO_STREAM("startRecording service called.");
	recording = true;
	return true;
}
bool Ros::SaverNode::stopRecording(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO_STREAM("stopRecording service called.");
	recording_count++;
	recording = false;
	return true;
}

bool Ros::SaverNode::clearLoggers(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO_STREAM("clearLoggers service called.");
	clearLoggers();
	return true;
}

bool Ros::SaverNode::writeCsv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO_STREAM("Write Csv service called.");
	saveCsvs();
	return true;
}
bool Ros::SaverNode::writeSto(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO_STREAM("Write Sto service called.");
	saveStos();
	return true;
}
bool Ros::SaverNode::write(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO_STREAM("Write service called.");
	save();
	return true;
}
void Ros::SaverNode::clearLoggers()
{
	for(auto& named_table:loggers)
	{
		ROS_WARN_STREAM("num_rows of table " << named_table->name << ": "
				<< named_table->absTable()->getNumRows());
		named_table->clear();
		ROS_WARN_STREAM("num_rows of table after deletion:"
				<< named_table->absTable()->getNumRows());
	}
}
void Ros::SaverNode::save() // newer method that chooses how to save
{
	for(auto& named_table:loggers)
	{
		auto loggerfilename = data_save_filename()+std::to_string(recording_count)+named_table->name;
		loggerfilename = resolved_file_prefix+ loggerfilename + named_table->extension();
		ROS_INFO_STREAM("Trying to save: " << loggerfilename);
		named_table->write(loggerfilename);
	}
}
void Ros::SaverNode::saveStos()
{
	for(auto& named_table:loggers)
	{
		auto loggerfilename = data_save_filename()+std::to_string(recording_count)+named_table->name+".sto";
		loggerfilename = resolved_file_prefix+ loggerfilename;
		ROS_INFO_STREAM("trying to save: " << loggerfilename);
		// NOT FileAdapter::writeFile: that looks the adapter up by extension and
		// then needs the table to be the type that adapter was built for. The
		// virtual knows its own type, so it calls STOFileAdapter_<T> directly.
		//this throws if you do it twice, so second save will crash
		//named_table->absTable()->addTableMetaData("notes",notes);
		//named_table->absTable()->addTableMetaData("description",description);
		named_table->writeSto(loggerfilename);
	}
}
void Ros::SaverNode::saveCsvs()
{
	for(auto& named_table:loggers)
	{
		auto loggerfilename = data_save_filename()+std::to_string(recording_count)+named_table->name+".csv";
		loggerfilename = resolved_file_prefix+ loggerfilename;
		ROS_INFO_STREAM("trying to save: " << loggerfilename);
		
		//this throws if you do it twice, so second save will crash
		//named_table->absTable()->addTableMetaData("notes",notes);
		//named_table->absTable()->addTableMetaData("description",description);
		named_table->writeCsv(loggerfilename);
	}
}

bool Ros::SaverNode::setNamePath(opensimrt_msgs::SetFileNameSrv::Request &req, opensimrt_msgs::SetFileNameSrv::Response &res)
{
	ROS_INFO_STREAM("name " << req.name << " path:"<< req.path);

	std::string dirname = resolve_dir(req.path);
	std::string filename = req.name;
	ROS_INFO_STREAM("dirname: " << dirname);
	nh.setParam("data_save_dir", dirname);
	nh.setParam("data_save_file", filename);
	notes = req.notes;
	description = req.description;
	// claude's suggestion was to set the table name here, instead of when we save it, which makes more sense actually
	for(auto& named_table:loggers)
	{
		if(named_table->absTable()->hasTableMetaDataKey("notes"))
			named_table->absTable()->removeTableMetaDataKey("notes");
		if(named_table->absTable()->hasTableMetaDataKey("description"))
			named_table->absTable()->removeTableMetaDataKey("description");
		named_table->absTable()->addTableMetaData("notes",notes);
		named_table->absTable()->addTableMetaData("description",description);
	}
	resolved_file_prefix = resolve_file_time(data_save_dir(),""); //sto data saved at the same time will have the same stamp
	return true;


}

void Ros::SaverNode::timeChecker() {
	ros::Time::init(); // Ensure ROS time is initialized
	while (ros::ok()) {
		ros::Time current_time = ros::Time::now();
		if (current_time.toSec() >= epoch_start) {
			recording_enabled.store(true);
		} else {
			recording_enabled.store(false);
		}

		// Sleep for a short period to avoid busy-waiting
		boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
	}
}
