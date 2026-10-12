#ifndef ROS_HEADER_FBK_01022023
#define ROS_HEADER_FBK_01022023
#include "ros/node_handle.h"
#include "opensimrt_msgs/LabelsSrv.h"
#include "opensimrt_msgs/SetFileNameSrv.h"
#include "std_srvs/Empty.h"
#include "ros/service_client.h"
#include "ros/service_server.h"
#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/CSVFileAdapter.h>
#include <boost/thread.hpp>
#include <atomic>
#include <memory>
#include <vector>
#include <string>
#include <memory>
//idk whats going on
#include "Settings.h"

namespace Ros
{
	/** One logger: a table, plus the name it gets saved under.
	 *
	 * WHY THERE ARE TWO CLASSES HERE: the loggers are not all the same type.
	 * q/qDot/imu hold doubles; the marker table holds SimTK::Vec3. A vector can
	 * only hold one type, and NamedTable<double> and NamedTable<Vec3> are two
	 * unrelated types, so SaverNode keeps them as NamedTableBase pointers.
	 * Everything that needs to know the element type is a virtual, implemented
	 * once in the template below. That virtual call is the single place where the
	 * type is recovered at runtime, which no amount of templating can avoid.
	 */
	class NamedTableBase
	{
		public:
			virtual ~NamedTableBase() = default;   // we delete through this pointer

			std::string name;

			/// read-only-ish view, for the things AbstractDataTable answers by itself
			/// (getNumRows, column labels, table metadata)
			virtual OpenSim::AbstractDataTable* absTable() = 0;

			/// empty the table, keeping its column labels
			virtual void clear() = 0;
			/// write the table to `filename` (extension included) as .sto
			virtual void writeSto(const std::string& filename) = 0;
			/// write the table to `filename` (extension included) as .csv
			virtual void writeCsv(const std::string& filename) = 0;
	};

	template<class T>
	class NamedTable : public NamedTableBase
	{
		public:
			NamedTable(std::string logger_name, OpenSim::TimeSeriesTable_<T>* t)
				: table(t)
			{
				name = std::move(logger_name);
			}

			OpenSim::TimeSeriesTable_<T>* table = nullptr;

			OpenSim::AbstractDataTable* absTable() override { return table; }

			void clear() override
			{
				// OpenSim tables have no clear(). clearAndDestroy() belongs to
				// OpenSim::Set<T> (MarkerSet &co), a different hierarchy entirely.
				// The only API is removeRowAtIndex, and calling it on index 0 in a
				// loop is QUADRATIC -- it shifts every later row down on each call.
				// So: assign a fresh empty table over the old one and put the labels
				// back. Assignment leaves the object at the same address, so the
				// pointers the owning node handed us stay valid.
				const bool had_labels = table->hasColumnLabels();
				std::vector<std::string> labels;
				if (had_labels) labels = table->getColumnLabels();

				*table = OpenSim::TimeSeriesTable_<T>{};

				if (had_labels) table->setColumnLabels(labels);
			}

			void writeSto(const std::string& filename) override
			{
				// STOFileAdapter_<T>::write is a template: it exists for double,
				// Vec3 and Quaternion, so this one line covers every logger.
				OpenSim::STOFileAdapter_<T>::write(*table, filename);
			}

			void writeCsv(const std::string& filename) override;
	};

	/// Generic case: OpenSim only ships a CSV adapter for double
	/// (CSVFileAdapter derives from DelimFileAdapter<double>), so a Vec3 or
	/// Quaternion table cannot be written as CSV. Say so and write the .sto
	/// instead of throwing halfway through a save.
	template<class T>
	void NamedTable<T>::writeCsv(const std::string& filename)
	{
		ROS_WARN_STREAM_ONCE("No CSV adapter exists in OpenSim for this table's element "
				"type, so [" << filename << "] will be written as .sto instead.");
		OpenSim::STOFileAdapter_<T>::write(*table, filename + ".sto");
	}

	/// double is the one element type with a real CSV adapter.
	template<>
	inline void NamedTable<double>::writeCsv(const std::string& filename)
	{
		OpenSim::CSVFileAdapter::write(*table, filename);
	}

	using NamedTables = std::vector<std::unique_ptr<NamedTableBase>>;

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
			template<class T>
				void initializeLoggers(std::string logger_name, OpenSim::TimeSeriesTable_<T>* logger)
				{
					// T is deduced, so every existing call site is unchanged. The
					// concrete type is captured here, in the NamedTable<T> we build,
					// and is never asked for again.
					at_least_one_logger_initialized = true;
					ROS_INFO_STREAM("Logger initialized, saveable by service: " << logger_name);
					loggers.push_back(
							std::make_unique<NamedTable<T>>(std::move(logger_name), logger));
				}

		private:
			std::atomic<bool> recording_enabled;
			double epoch_start;
			boost::thread check_thread;
			void timeChecker();
			std::string notes, description;
	};


}

#endif
