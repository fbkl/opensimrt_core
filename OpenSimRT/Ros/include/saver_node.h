#ifndef ROS_HEADER_FBK_01022023
#define ROS_HEADER_FBK_01022023
#include "ros/node_handle.h"
#include "opensimrt_msgs/LabelsSrv.h"
#include "opensimrt_msgs/SetFileNameSrv.h"
#include "std_srvs/Empty.h"
#include "ros/service_client.h"
#include "ros/service_server.h"
#include <Common/TimeSeriesTable.h>
#include <boost/thread.hpp>
#include <atomic>
//idk whats going on
#include "Settings.h"

namespace Ros
{
	using NamedTable = std::pair<OpenSim::TimeSeriesTable*, std::string >; 
	using NamedTables = std::vector<NamedTable>;
	class SaverNode
	{
		public:
			SaverNode(bool Debug=false);
			virtual ~SaverNode();
			ros::NodeHandle nh{"~"};
			NamedTables loggers;
			bool recording = false;
			//
			// we need to replace every recording check with isRecording now, since we are using the threading thing now.
			bool isRecording() const {
				return recording_enabled.load() && recording;
			}
			uint32_t recording_count = 0;
			std::string resolved_file_prefix = "";
			std::string data_save_dir()
			{
				std::string data_save_dir_str; 
				nh.param<std::string>("data_save_dir",data_save_dir_str,"/tmp/");
				ROS_INFO_STREAM("current saving prefix: " << data_save_dir_str);
				return data_save_dir_str;

			};
			std::string data_save_filename()
			{
				std::string data_save_filename_str; 
				nh.param<std::string>("data_save_file",data_save_filename_str,"file");
				ROS_INFO_STREAM("current saving file: " << data_save_filename_str);
				return data_save_filename_str;

			};
			void onInit();
			bool at_least_one_logger_initialized = false;
			void saveStos();
			void saveCsvs();
			void clearLoggers();
			ros::ServiceServer write_csv, write_sto;
		protected:
			ros::ServiceServer startRecordingSrv, stopRecordingSrv, clearLoggersSrv, setNamePathSrv;
			bool writeCsv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
			bool writeSto(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
			bool startRecording(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
			bool stopRecording(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
			bool clearLoggers(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
			bool setNamePath(opensimrt_msgs::SetFileNameSrv::Request &req, opensimrt_msgs::SetFileNameSrv::Response &res);
			void initializeLoggers(std::string logger_name, OpenSim::TimeSeriesTable *logger);

		private:
			std::atomic<bool> recording_enabled;
			double epoch_start;
			boost::thread check_thread;
			void timeChecker();
			std::string notes, description;
	};


}

#endif
