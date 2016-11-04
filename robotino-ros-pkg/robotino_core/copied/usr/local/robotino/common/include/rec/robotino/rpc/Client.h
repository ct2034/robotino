#ifndef _REC_ROBOTINO_RPC_CLIENT_H_
#define _REC_ROBOTINO_RPC_CLIENT_H_

#include <QtCore>

#include "rec/robotino/rpc/defines.h"

#include "rec/robotino/rpc/messages/image.h"
#include "rec/robotino/rpc/messages/camera_settings.h"
#include "rec/robotino/rpc/messages/camera_control.h"
#include "rec/robotino/rpc/messages/camera_capabilities.h"
#include "rec/robotino/rpc/messages/camera_calibration.h"
#include "rec/robotino/rpc/messages/motor_setpoint.h"
#include "rec/robotino/rpc/messages/motor_reset_position.h"
#include "rec/robotino/rpc/messages/motor_readings.h"
#include "rec/robotino/rpc/messages/omnidrive.h"
#include "rec/robotino/rpc/messages/odometry.h"
#include "rec/robotino/rpc/messages/pose.h"
#include "rec/robotino/rpc/messages/northstar.h"
#include "rec/robotino/rpc/messages/gyroscope.h"
#include "rec/robotino/rpc/messages/emergency_bumper.h"
#include "rec/robotino/rpc/messages/display_text.h"
#include "rec/robotino/rpc/messages/display_backlight.h"
#include "rec/robotino/rpc/messages/display_buttons.h"
#include "rec/robotino/rpc/messages/display_vbar.h"
#include "rec/robotino/rpc/messages/display_hbar.h"
#include "rec/robotino/rpc/messages/display_progress.h"
#include "rec/robotino/rpc/messages/display_clear.h"
#include "rec/robotino/rpc/messages/parameters.h"
#include "rec/robotino/rpc/messages/disconnect_client.h"
#include "rec/robotino/rpc/messages/ea_version.h"
#include "rec/robotino/rpc/messages/ea09_ip4address.h"
#include "rec/robotino/rpc/messages/clients_connected.h"
#include "rec/robotino/rpc/messages/digital_input.h"
#include "rec/robotino/rpc/messages/analog_input.h"
#include "rec/robotino/rpc/messages/bumper.h"
#include "rec/robotino/rpc/messages/battery.h"
#include "rec/robotino/rpc/messages/distance_sensors.h"
#include "rec/robotino/rpc/messages/set_digital_output.h"
#include "rec/robotino/rpc/messages/set_relay.h"
#include "rec/robotino/rpc/messages/set_shutdown.h"
#include "rec/robotino/rpc/messages/power_button.h"
#include "rec/robotino/rpc/messages/request_shutdown.h"
#include "rec/robotino/rpc/messages/set_pid_parameters.h"
#include "rec/robotino/rpc/messages/gripper_state.h"
#include "rec/robotino/rpc/messages/scan.h"
#include "rec/robotino/rpc/messages/cbha_readings.h"
#include "rec/robotino/rpc/messages/cbha_set_pressure.h"
#include "rec/robotino/rpc/messages/cbha_set_compressors_enabled.h"
#include "rec/robotino/rpc/messages/cbha_set_water_drain_valve.h"
#include "rec/robotino/rpc/messages/cbha_set_gripper_valve1.h"
#include "rec/robotino/rpc/messages/cbha_set_gripper_valve2.h"
#include "rec/robotino/rpc/messages/grappler_store_positions.h"
#include "rec/robotino/rpc/messages/grappler_set_power.h"
#include "rec/robotino/rpc/messages/grappler_set_positions.h"
#include "rec/robotino/rpc/messages/grappler_servos.h"
#include "rec/robotino/rpc/messages/grappler_readings.h"
#include "rec/robotino/rpc/messages/grappler_toggle_torque.h"
#include "rec/robotino/rpc/messages/log.h"
#include "rec/robotino/rpc/messages/custom_message.h"
#include "rec/robotino/rpc/messages/kinect.h"
#include "rec/robotino/rpc/messages/process.h"
#include "rec/robotino/rpc/messages/api1.h"
#include "rec/robotino/rpc/messages/charger.h"
#include "rec/robotino/rpc/messages/sensors.h"

/*Factory 4***************************/
#include "rec/robotino/rpc/messages/json_message.h"
#include "rec/robotino/rpc/messages/json_message_with_data.h"
#include "rec/robotino/rpc/messages/fleetcom_request.h"
#include "rec/robotino/rpc/messages/map.h"
#include "rec/robotino/rpc/messages/string_message.h"

#include "rec/rpc/Client.h"

#define JSON_TOPIC_PRIVATE_DEFINITIONS(TOPICNAME) \
	DECLARE_TOPICLISTENER(rec_robotino_rpc_##TOPICNAME) \
	DECLARE_TOPICINFOCHANGED(rec_robotino_rpc_##TOPICNAME);

#define JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(TOPICNAME) \
	DECLARE_TOPICLISTENER(rec_robotino_rpc_##TOPICNAME) \
	DECLARE_TOPICINFOCHANGED(rec_robotino_rpc_##TOPICNAME);

namespace rec
{
	namespace robotino
	{
		namespace rpc
		{
			class REC_ROBOTINO_RPC_EXPORT Client : public rec::rpc::Client
			{
				Q_OBJECT
			public:
				Client( QObject* parent = 0 );
				~Client();

				/**
				@param address IP of Robotino or Robotino SIM.
				               Syntax: "<ipaddress>:<port>"
							   Example: "127.0.0.1:12080"
							   If no port is given the default port 12080 is used so in the above example "127.0.0.1" would do the same job.
							   Set the address to "RPCD_SERVER_ADDRESS" to use the value from the environment variable RPCD_SERVER_ADDRESS.
							   If RPCD_SERVER_ADDRESS is not defined "127.0.0.1" is used as fallback.
				*/
				void setAddress( const QString& address );

				/**
				@return Returns the current server address. The default port 12080 is omitted.
				*/
				QString address() const;

				void readParameters( const QString& path, const QString& userpath );

				bool setParameter( const QString& key, const QVariant& value );
				QVariant getLocalParameter( const QString& key, const QVariant& defaultValue = QVariant() );
				QVariant getParameter( const QString& key, const QVariant& defaultValue = QVariant() );

				QMap< QString, QVariant > getLocalParameters() const;

				QMap< QString, QVariant > getParameters();
				bool containsParameter( const QString& key );

				bool disconnectClient( const QHostAddress& address, quint16 port );

				int launchProcess( const QString& command, const QStringList& parameters, const QString& workingdirectory = QString() );
				int terminateProcess( int pid );
				int killProcess( int pid );
				QVector< int > getProcessIds();
				rec::robotino::rpc::ProcessStatus getProcessStatus( int id );

				QStringList allTopics() const { return _enableMap.keys(); }
				
				QString json_message_with_data_topics_alias(int topic) const { return _json_message_with_data_topics_alias_map.value(topic); }

			public Q_SLOTS:
				bool setTopicEnabled(const QString& topicName, bool enable);
				bool set_json_message_with_data_topic_Enabled(int topic, bool enable);

				bool set_disconnect_api1_clients();
				bool set_api1_setstate_received();

				bool set_image( unsigned int cameraNumber,
									const QByteArray& data,
									unsigned int width,
									unsigned int height,
									unsigned int step,
									const QString& format );

				bool set_image0( const QByteArray& data,
									unsigned int width,
									unsigned int height,
									unsigned int step,
									const QString& format );
				
				bool set_image1( const QByteArray& data,
									unsigned int width,
									unsigned int height,
									unsigned int step,
									const QString& format );

				bool set_image2( const QByteArray& data,
									unsigned int width,
									unsigned int height,
									unsigned int step,
									const QString& format );

				bool set_image3( const QByteArray& data,
									unsigned int width,
									unsigned int height,
									unsigned int step,
									const QString& format );

				bool set_camera_settings( unsigned int cameraNumber, unsigned int width, unsigned int height, const QString& format );
				bool set_camera0_settings( unsigned int width, unsigned int height, const QString& format );
				bool set_camera1_settings( unsigned int width, unsigned int height, const QString& format );
				bool set_camera2_settings( unsigned int width, unsigned int height, const QString& format );
				bool set_camera3_settings( unsigned int width, unsigned int height, const QString& format );

				bool set_set_camera_control( unsigned int cameraNumber, const QString& name, int value );
				bool set_set_camera0_control( const QString& name, int value );
				bool set_set_camera1_control( const QString& name, int value );
				bool set_set_camera2_control( const QString& name, int value );
				bool set_set_camera3_control( const QString& name, int value );

				bool set_set_camera_settings( unsigned int cameraNumber, unsigned int width, unsigned int height, const QString& format );
				bool set_set_camera0_settings( unsigned int width, unsigned int height, const QString& format );
				bool set_set_camera1_settings( unsigned int width, unsigned int height, const QString& format );
				bool set_set_camera2_settings( unsigned int width, unsigned int height, const QString& format );
				bool set_set_camera3_settings( unsigned int width, unsigned int height, const QString& format );

				bool set_camera_capabilities( unsigned int cameraNumber, const QString& cameraName, const QMap<QString, QVector<QSize> >& capabilities, const QStringList& controls );
				bool set_camera0_capabilities( const QString& cameraName, const QMap<QString, QVector<QSize> >& capabilities, const QStringList& controls );
				bool set_camera1_capabilities( const QString& cameraName, const QMap<QString, QVector<QSize> >& capabilities, const QStringList& controls );
				bool set_camera2_capabilities( const QString& cameraName, const QMap<QString, QVector<QSize> >& capabilities, const QStringList& controls );
				bool set_camera3_capabilities( const QString& cameraName, const QMap<QString, QVector<QSize> >& capabilities, const QStringList& controls );

				bool set_camera_calibration( unsigned int cameraNumber, const QVector<double>& calibration );
				bool set_camera0_calibration( const QVector<double>& calibration );
				bool set_camera1_calibration( const QVector<double>& calibration );
				bool set_camera2_calibration( const QVector<double>& calibration );
				bool set_camera3_calibration( const QVector<double>& calibration );

				bool set_motor0_setpoint( float speed );
				bool set_motor0_reset_position( int position );
				bool set_set_motor0_mode( const QString& mode );

				bool set_motor1_setpoint( float speed );
				bool set_motor1_reset_position( int position );
				bool set_set_motor1_mode( const QString& mode );

				bool set_motor2_setpoint( float speed );
				bool set_motor2_reset_position( int position );
				bool set_set_motor2_mode( const QString& mode );

				bool set_motor3_setpoint( float speed );
				bool set_motor3_reset_position( int position );
				bool set_set_motor3_mode( const QString& mode );

				bool set_motor_setpoints( float m0, float m1, float m2 );

				bool set_motor_readings( const QVector< float >& speeds, const QVector< int >& positions, const QVector< float >& currents, float time_delta );

				bool set_omnidrive( float vx, float vy, float omega );

				bool set_odometry( double x, double y, double phi, float vx, float vy, float omega, unsigned int sequence );

				bool set_set_odometry( double x, double y, double phi );

				bool set_pose(double x, double y, double phi, double errx, double erry, double errphi, unsigned int sequence);

				bool set_northstar( unsigned int sequenceNumber,
										unsigned int roomId,
										unsigned int numSpotsVisible,
										float posX,
										float posY,
										float posTheta,
										unsigned int magSpot0,
										unsigned int magSpot1 );

				bool set_set_northstar_parameters( unsigned int roomId, float ceilingCal );

				bool set_gyroscope( double phi, double rate );

				bool set_gyroscope_ext(double phi, double rate);

				/**
				Used by controld2 to inform other clients about the emergency bumper state
				*/
				bool set_emergency_bumper( bool enable );

				/**
				Used by clients to inform the controld2 about new settings for emergency bumper
				*/
				bool set_set_emergency_bumper( bool enable );

				bool set_display_text( const QString& text, unsigned int row = 0, unsigned int col = 0, bool clear_before = true, bool clear_after = true );

				bool set_display_backlight( bool on );

				bool set_display_buttons( bool up, bool down, bool back, bool enter );

				bool set_display_vbar( float value, unsigned int col, unsigned int start_row, unsigned int end_row );

				bool set_display_hbar( float value, unsigned int row, unsigned int start_col, unsigned int end_col );

				bool set_display_progress( unsigned int step, unsigned int row );

				bool set_display_clear();

				/**
				Used by controld2 to inform other clients about the ip address of the EA09
				*/
				bool set_ea09_ip4address( const QHostAddress& address, const QHostAddress& netmask );
				
				/**
				Used by clients to inform the controld2 about new settings for the EA09 ip address
				*/
				bool set_set_ea09_ip4address( const QHostAddress& address, const QHostAddress& netmask );

				bool set_ea_version( const QString& board, quint16 firmware_major, quint16 firmware_minor, quint16 firmware_patch );

				bool set_digital_input( const QVector< bool >& );

				bool set_analog_input( const QVector< float >& );

				bool set_distance_sensors( const QVector< float >& );

				bool set_bumper( bool );

				bool set_battery( float battery_voltage, float system_current, bool ext_power, int num_chargers, const QString& batteryType, bool batteryLow, int batteryLowShutdownCounter );

				bool set_set_digital_output( unsigned int index, bool state );

				bool set_set_digital_output_array( const QVector< bool >& data );

				bool set_set_relay( unsigned int index, bool state );

				bool set_set_relay_array( const QVector< bool >& data );

				bool set_set_shutdown( bool shutdown );

				bool set_power_button( bool pressed );

				bool set_request_shutdown( bool doit );

				bool set_set_pid_parameters( unsigned int motor, float kp, float ki, float kd );

				bool set_gripper_state( int state );

				bool set_scan( unsigned int laserNumber, QVector< float > ranges, QVector< float > intensities, QVariantMap parameters);
				bool set_scan0( QVector< float > ranges, QVector< float > intensities, QVariantMap parameters );
				bool set_scan1( QVector< float > ranges, QVector< float > intensities, QVariantMap parameters );
				bool set_scan2( QVector< float > ranges, QVector< float > intensities, QVariantMap parameters );
				bool set_scan3( QVector< float > ranges, QVector< float > intensities, QVariantMap parameters );

				bool set_cbha_readings( const QVector<float>& pressures, bool pressureSensor, const QVector<float>& stringPots, float foilPot );
				bool set_cbha_set_pressure( const QVector<float>& pressures );
				bool set_cbha_set_compressors_enabled( bool enabled );
				bool set_cbha_set_water_drain_valve( bool enabled );
				bool set_cbha_set_gripper_valve1( bool enabled );
				bool set_cbha_set_gripper_valve2( bool enabled );

				bool set_grappler_store_positions( const QVector< rec::robotino::rpc::GrapplerServoInfo>& data );
				bool set_grappler_set_power( int line, bool power );
				bool set_grappler_set_positions( const QVector< rec::robotino::rpc::GrapplerServoInfo>& data );
				bool set_grappler_servos( const QVector< rec::robotino::rpc::GrapplerServoInfo>& data );
				bool set_grappler_readings( const QVector< rec::robotino::rpc::GrapplerServoInfo>& data );
				bool set_grappler_toggle_torque();
				bool set_log( const QString& message, int level );
				bool set_set_log_level( const QString& publisher, int level );

				bool set_custom_message( unsigned int id, const QByteArray& data );
				bool set_custom_message0( const QByteArray& data );
				bool set_custom_message1( const QByteArray& data );
				bool set_custom_message2( const QByteArray& data );
				bool set_custom_message3( const QByteArray& data );

				bool set_kinect_set_tilt( unsigned int id, double tilt );
				bool set_kinect0_set_tilt( double tilt );
				bool set_kinect1_set_tilt( double tilt );
				bool set_kinect2_set_tilt( double tilt );
				bool set_kinect3_set_tilt( double tilt );

				bool set_kinect_tilt( unsigned int id, double tilt );
				bool set_kinect0_tilt( double tilt );
				bool set_kinect1_tilt( double tilt );
				bool set_kinect2_tilt( double tilt );
				bool set_kinect3_tilt( double tilt );

				bool set_kinect_accel( unsigned int id, double x, double y, double z );
				bool set_kinect0_accel( double x, double y, double z );
				bool set_kinect1_accel( double x, double y, double z );
				bool set_kinect2_accel( double x, double y, double z );
				bool set_kinect3_accel( double x, double y, double z );

				bool set_kinect_set_led( unsigned int id, unsigned int state );
				bool set_kinect0_set_led( unsigned int state );
				bool set_kinect1_set_led( unsigned int state );
				bool set_kinect2_set_led( unsigned int state );
				bool set_kinect3_set_led( unsigned int state );

				bool set_kinect_set_video_format( unsigned int id, unsigned int format );
				bool set_kinect0_set_video_format( unsigned int format );
				bool set_kinect1_set_video_format( unsigned int format );
				bool set_kinect2_set_video_format( unsigned int format );
				bool set_kinect3_set_video_format( unsigned int format );

				bool set_kinect_set_depth_format( unsigned int id, unsigned int format );
				bool set_kinect0_set_depth_format( unsigned int format );
				bool set_kinect1_set_depth_format( unsigned int format );
				bool set_kinect2_set_depth_format( unsigned int format );
				bool set_kinect3_set_depth_format( unsigned int format );

				bool set_kinect_depth( unsigned int id, const QByteArray& data, const QByteArray& object_data, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );
				bool set_kinect0_depth( const QByteArray& data, const QByteArray& object_data, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );
				bool set_kinect1_depth( const QByteArray& data, const QByteArray& object_data, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );
				bool set_kinect2_depth( const QByteArray& data, const QByteArray& object_data, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );
				bool set_kinect3_depth( const QByteArray& data, const QByteArray& object_data, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );

				bool set_kinect_video( unsigned int id, const QByteArray& data, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );
				bool set_kinect0_video( const QByteArray& data, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );
				bool set_kinect1_video( const QByteArray& data, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );
				bool set_kinect2_video( const QByteArray& data, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );
				bool set_kinect3_video( const QByteArray& data, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );

				bool set_poseOnMap( const QPointF&, double);
				
				bool set_initialPose( const QPointF&, double);

				bool set_navGoal( const QPointF&, double);

				bool set_charger0_clear_error();
				bool set_charger1_clear_error();
				bool set_charger2_clear_error();

				bool set_charger0_info( unsigned int time, float batteryVoltage, float chargingCurrent, float bat1temp, float bat2temp, int state_number, const QString& state );
				bool set_charger1_info( unsigned int time, float batteryVoltage, float chargingCurrent, float bat1temp, float bat2temp, int state_number, const QString& state );
				bool set_charger2_info( unsigned int time, float batteryVoltage, float chargingCurrent, float bat1temp, float bat2temp, int state_number, const QString& state );
		
				bool set_charger0_get_version();
				bool set_charger1_get_version();
				bool set_charger2_get_version();

				bool set_charger0_version( int major, int minor, int patch );
				bool set_charger1_version( int major, int minor, int patch );
				bool set_charger2_version( int major, int minor, int patch );

				bool set_charger0_error( unsigned int time, const QString& message );
				bool set_charger1_error( unsigned int time, const QString& message );
				bool set_charger2_error( unsigned int time, const QString& message );

				bool set_sensors( const QVector< QString >& names, const QVector< float >& values, const QVector< QString >& units );

				/*Factory 4*/

				bool set_fleetcom_request( const QString& message );

				bool set_mclayout(const QByteArray& jsonData);
				bool set_mcstatus(const QByteArray& jsonData);
				bool set_pathnetwork(const QByteArray& jsonData);
				bool set_pathnetwork_edited(const QByteArray& jsonData);
				bool set_localizationMode(const QByteArray& jsonData);
				bool set_smartlog(const QByteArray& jsonData);
				bool set_smartnavigationplan(const QByteArray& jsonData);
				bool set_smartlocations(const QByteArray& jsonData);
				bool set_smartlocations_edited(const QByteArray& jsonData);
				bool set_smartrobotinfo(const QByteArray& jsonData);
				bool set_smartmyrobotid(const QByteArray& jsonData);
				bool set_smartjoblist(const QByteArray& jsonData);
				bool set_smartlicense(const QByteArray& jsonData);
				bool set_smartdockingvis(const QByteArray& jsonData);
				bool set_mapDir(const QByteArray& jsonData);
				bool set_fleetcom_response(const QByteArray& jsonData);

				bool set_mapDir_request(const QByteArray& jsonData, const QByteArray& data);
				bool set_mapDir_response(const QByteArray& infoData, const QByteArray& data);
				bool set_map(const QByteArray&, int width, int height, float resolution, float offsetx, float offsety);
				bool set_mapPlanner(const QByteArray&, int width, int height, float resolution, float offsetx, float offsety);
				bool set_mapPlannerEdited(const QByteArray&, int width, int height, float resolution, float offsetx, float offsety);

				bool set_json_message_with_data(int topic, const QByteArray& jsonData, const QByteArray& data);

			Q_SIGNALS:
				void is_connected();
				void is_disconnected();
				void logLevelChanged( int verbosity );

				void camera0_settings_changed( unsigned int width, unsigned int height, const QString& format );
				void camera0_settings_info_changed( const rec::rpc::ClientInfoSet& );

				void set_camera0_control_changed( const QString& name, int value );
				void set_camera0_control_info_changed( const rec::rpc::ClientInfoSet& );

				void set_camera0_settings_changed( unsigned int width, unsigned int height, const QString& format );
				void set_camera0_settings_info_changed( const rec::rpc::ClientInfoSet& );

				void camera0_capabilities_changed( const QString& cameraName, const QMap<QString, QVector<QSize> >& capabilities, const QStringList& controls );
				void camera0_capabilities_info_changed( const rec::rpc::ClientInfoSet& );

				void camera0_calibration_changed( const QVector<double>& calibration );
				void camera0_calibration_info_changed( const rec::rpc::ClientInfoSet& );

				void image0_changed( const QByteArray& data,
									unsigned int width,
									unsigned int height,
									unsigned int step,
									const QString& format );
				void image0_info_changed( const rec::rpc::ClientInfoSet& );

				void camera1_settings_changed( unsigned int width, unsigned int height, const QString& format );
				void camera1_settings_info_changed( const rec::rpc::ClientInfoSet& );

				void set_camera1_control_changed( const QString& name, int value );
				void set_camera1_control_info_changed( const rec::rpc::ClientInfoSet& );

				void set_camera1_settings_changed( unsigned int width, unsigned int height, const QString& format );
				void set_camera1_settings_info_changed( const rec::rpc::ClientInfoSet& );

				void camera1_capabilities_changed( const QString& cameraName, const QMap<QString, QVector<QSize> >& capabilities, const QStringList& controls );
				void camera1_capabilities_info_changed( const rec::rpc::ClientInfoSet& );

				void camera1_calibration_changed( const QVector<double>& calibration );
				void camera1_calibration_info_changed( const rec::rpc::ClientInfoSet& );

				void image1_changed( const QByteArray& data,
									unsigned int width,
									unsigned int height,
									unsigned int step,
									const QString& format );
				void image1_info_changed( const rec::rpc::ClientInfoSet& );

				void camera2_settings_changed( unsigned int width, unsigned int height, const QString& format );
				void camera2_settings_info_changed( const rec::rpc::ClientInfoSet& );

				void set_camera2_control_changed( const QString& name, int value );
				void set_camera2_control_info_changed( const rec::rpc::ClientInfoSet& );

				void set_camera2_settings_changed( unsigned int width, unsigned int height, const QString& format );
				void set_camera2_settings_info_changed( const rec::rpc::ClientInfoSet& );

				void camera2_capabilities_changed( const QString& cameraName, const QMap<QString, QVector<QSize> >& capabilities, const QStringList& controls );
				void camera2_capabilities_info_changed( const rec::rpc::ClientInfoSet& );

				void camera2_calibration_changed( const QVector<double>& calibration );
				void camera2_calibration_info_changed( const rec::rpc::ClientInfoSet& );

				void image2_changed( const QByteArray& data,
									unsigned int width,
									unsigned int height,
									unsigned int step,
									const QString& format );
				void image2_info_changed( const rec::rpc::ClientInfoSet& );

				void camera3_settings_changed( unsigned int width, unsigned int height, const QString& format );
				void camera3_settings_info_changed( const rec::rpc::ClientInfoSet& );

				void set_camera3_control_changed( const QString& name, int value );
				void set_camera3_control_info_changed( const rec::rpc::ClientInfoSet& );

				void set_camera3_settings_changed( unsigned int width, unsigned int height, const QString& format );
				void set_camera3_settings_info_changed( const rec::rpc::ClientInfoSet& );

				void camera3_capabilities_changed( const QString& cameraName, const QMap<QString, QVector<QSize> >& capabilities, const QStringList& controls );
				void camera3_capabilities_info_changed( const rec::rpc::ClientInfoSet& );

				void camera3_calibration_changed( const QVector<double>& calibration );
				void camera3_calibration_info_changed( const rec::rpc::ClientInfoSet& );

				void image3_changed( const QByteArray& data,
									unsigned int width,
									unsigned int height,
									unsigned int step,
									const QString& format );
				void image3_info_changed( const rec::rpc::ClientInfoSet& );
				
				void motor0_setpoint_changed( float speed, const rec::rpc::ClientInfo& );
				void motor0_setpoint_info_changed( const rec::rpc::ClientInfoSet& );

				void motor0_reset_position_changed( int position, const rec::rpc::ClientInfo& );
				void motor0_reset_position_info_changed( const rec::rpc::ClientInfoSet& );

				void set_motor0_mode_changed( const QString& mode );
				void set_motor0_mode_info_changed(  const rec::rpc::ClientInfoSet& );

				void motor0_mode_changed( const QString& mode );
				void motor0_mode_info_changed(  const rec::rpc::ClientInfoSet& );

				void motor1_setpoint_changed( float speed, const rec::rpc::ClientInfo& );
				void motor1_setpoint_info_changed( const rec::rpc::ClientInfoSet& );

				void motor1_reset_position_changed( int position, const rec::rpc::ClientInfo& );
				void motor1_reset_position_info_changed( const rec::rpc::ClientInfoSet& );

				void set_motor1_mode_changed( const QString& mode );
				void set_motor1_mode_info_changed( const rec::rpc::ClientInfoSet& );

				void motor1_mode_changed( const QString& mode );
				void motor1_mode_info_changed( const rec::rpc::ClientInfoSet& );

				void motor2_setpoint_changed( float speed, const rec::rpc::ClientInfo& );
				void motor2_setpoint_info_changed( const rec::rpc::ClientInfoSet& );

				void motor2_reset_position_changed( int position, const rec::rpc::ClientInfo& );
				void motor2_reset_position_info_changed( const rec::rpc::ClientInfoSet& );

				void set_motor2_mode_changed( const QString& mode );
				void set_motor2_mode_info_changed( const rec::rpc::ClientInfoSet& );

				void motor2_mode_changed( const QString& mode );
				void motor2_mode_info_changed( const rec::rpc::ClientInfoSet& );

				void motor3_setpoint_changed( float speed, const rec::rpc::ClientInfo& );
				void motor3_setpoint_info_changed( const rec::rpc::ClientInfoSet& );

				void motor3_reset_position_changed( int position, const rec::rpc::ClientInfo& );
				void motor3_reset_position_info_changed( const rec::rpc::ClientInfoSet& );

				void set_motor3_mode_changed( const QString& mode );
				void set_motor3_mode_info_changed( const rec::rpc::ClientInfoSet& );

				void motor3_mode_changed( const QString& mode );
				void motor3_mode_info_changed( const rec::rpc::ClientInfoSet& );

				void motor_setpoints_changed( float m0, float m1, float m2, const rec::rpc::ClientInfo& );
				void motor_setpoints_info_changed( const rec::rpc::ClientInfoSet& );

				void motor_readings_changed( const QVector< float >& speeds, const QVector< int >& positions, const QVector< float >& currents, float time_delta );
				void motor_readings_info_changed( const rec::rpc::ClientInfoSet& );

				void omnidrive_changed( float vx, float vy, float omega, const rec::rpc::ClientInfo& );
				void omnidrive_info_changed( const rec::rpc::ClientInfoSet& );

				void odometry_changed(double x, double y, double phi, float vx, float vy, float omega, unsigned int sequence);
				void odometry_info_changed( const rec::rpc::ClientInfoSet& );

				void set_odometry_changed( double x, double y, double phi );
				void set_odometry_info_changed( const rec::rpc::ClientInfoSet& );

				void pose_changed(double x, double y, double phi, double errx, double erry, double errphi, unsigned int sequence);
				void pose_info_changed(const rec::rpc::ClientInfoSet&);

				void northstar_changed( unsigned int sequenceNumber,
										unsigned int roomId,
										unsigned int numSpotsVisible,
										float posX,
										float posY,
										float posTheta,
										unsigned int magSpot0,
										unsigned int magSpot1 );
				void northstar_info_changed( const rec::rpc::ClientInfoSet& );

				void set_northstar_parameters_changed( unsigned int roomId, float ceilingCal );
				void set_northstar_parameters_info_changed( const rec::rpc::ClientInfoSet& );

				void gyroscope_changed( double phi, double rate, const rec::rpc::ClientInfo& );
				void gyroscope_info_changed( const rec::rpc::ClientInfoSet& );

				void gyroscope_ext_changed(double phi, double rate, const rec::rpc::ClientInfo&);
				void gyroscope_ext_info_changed(const rec::rpc::ClientInfoSet&);

				void emergency_bumper_changed( bool enable );
				void emergency_bumper_info_changed( const rec::rpc::ClientInfoSet& );

				void set_emergency_bumper_changed( bool enable );
				void set_emergency_bumper_info_changed( const rec::rpc::ClientInfoSet& );

				void display_text_changed( const QString& text, unsigned int row, unsigned int col, bool clear_before, bool clear_after );
				void display_text_info_changed( const rec::rpc::ClientInfoSet& );

				void display_backlight_changed( bool on );
				void display_backlight_info_changed( const rec::rpc::ClientInfoSet& );

				void display_buttons_changed( bool up, bool down, bool back, bool enter );
				void display_buttons_info_changed( const rec::rpc::ClientInfoSet& );

				void display_vbar_changed( float value, unsigned int col, unsigned int start_row, unsigned int end_row );
				void display_vbar_info_changed( const rec::rpc::ClientInfoSet& );

				void display_hbar_changed( float value, unsigned int row, unsigned int start_col, unsigned int end_col );
				void display_hbar_info_changed( const rec::rpc::ClientInfoSet& );

				void display_progress_changed( unsigned int step, unsigned int row );
				void display_progress_info_changed( const rec::rpc::ClientInfoSet& );

				void display_clear_changed();
				void display_clear_info_changed( const rec::rpc::ClientInfoSet& );

				void parameters_changed( const QMap< QString, QVariant >& values );
				void parameters_info_changed( const rec::rpc::ClientInfoSet& );

				void current_controller_changed( const rec::rpc::ClientInfoMap& );
				void current_controller_info_changed( const rec::rpc::ClientInfoSet& );

				void ea_version_changed( const QString& board, quint16 firmware_major, quint16 firmware_minor, quint16 firmware_patch );
				void ea_version_info_changed( const rec::rpc::ClientInfoSet& );

				void ea09_ip4address_changed( const QHostAddress& address, const QHostAddress& netmask );
				void ea09_ip4address_info_changed( const rec::rpc::ClientInfoSet& );

				void set_ea09_ip4address_changed( const QHostAddress& address, const QHostAddress& netmask );
				void set_ea09_ip4address_info_changed( const rec::rpc::ClientInfoSet& );

				void clients_connected_changed( const QVector< rec::rpc::ClientInfo >& clients );
				void clients_connected_info_changed( const rec::rpc::ClientInfoSet& );

				void digital_input_changed( const QVector< bool >& );
				void digital_input_info_changed( const rec::rpc::ClientInfoSet& );

				void analog_input_changed( const QVector< float >& );
				void analog_input_info_changed( const rec::rpc::ClientInfoSet& );

				void distance_sensors_changed( const QVector< float >& );
				void distance_sensors_info_changed( const rec::rpc::ClientInfoSet& );

				void bumper_changed( bool );
				void bumper_info_changed( const rec::rpc::ClientInfoSet& );

				void battery_changed( float battery_voltage, float system_current, bool ext_power, int num_chargers, const QString& batteryType, bool batteryLow, int batteryLowShutdownCounter );
				void battery_info_changed( const rec::rpc::ClientInfoSet& );

				void set_digital_output_changed( unsigned int index, bool state );
				void set_digital_output_info_changed( const rec::rpc::ClientInfoSet& );

				void set_digital_output_array_changed( const QVector< bool >& );
				void set_digital_output_array_info_changed( const rec::rpc::ClientInfoSet& );

				void set_relay_changed( unsigned int index, bool state );
				void set_relay_info_changed( const rec::rpc::ClientInfoSet& );

				void set_relay_array_changed( const QVector< bool >& );
				void set_relay_array_info_changed( const rec::rpc::ClientInfoSet& );

				void set_shutdown_changed( bool shutdown );
				void set_shutdown_info_changed( const rec::rpc::ClientInfoSet& );

				void power_button_changed( bool pressed );
				void power_button_info_changed( const rec::rpc::ClientInfoSet& );

				void request_shutdown_changed( bool pressed );
				void request_shutdown_info_changed( const rec::rpc::ClientInfoSet& );

				void set_pid_parameters_changed( unsigned int motor, float kp, float ki, float kd );
				void set_pid_parameters_info_changed( const rec::rpc::ClientInfoSet& );

				void gripper_state_changed( int state );
				void gripper_state_info_changed( const rec::rpc::ClientInfoSet& );

				void scan0_changed( const QVector< float >& ranges,	const QVector< float >& intensities, const QVariantMap& parameters );
				void scan0_info_changed( const rec::rpc::ClientInfoSet& );

				void scan1_changed( const QVector< float >& ranges,	const QVector< float >& intensities, const QVariantMap& parameters );
				void scan1_info_changed( const rec::rpc::ClientInfoSet& );

				void scan2_changed( const QVector< float >& ranges,	const QVector< float >& intensities, const QVariantMap& parameters );
				void scan2_info_changed( const rec::rpc::ClientInfoSet& );

				void scan3_changed( const QVector< float >& ranges,	const QVector< float >& intensities, const QVariantMap& parameters );
				void scan3_info_changed( const rec::rpc::ClientInfoSet& );

				void cbha_readings_changed( const QVector<float>& pressures, bool pressureSensor, const QVector<float>& stringPots, float foilPot );
				void cbha_readings_info_changed( const rec::rpc::ClientInfoSet& );

				void cbha_set_pressure_changed( const QVector<float>& pressures );
				void cbha_set_pressure_info_changed( const rec::rpc::ClientInfoSet& );

				void cbha_set_compressors_enabled_changed( bool enabled );
				void cbha_set_compressors_enabled_info_changed( const rec::rpc::ClientInfoSet& );

				void cbha_set_water_drain_valve_changed( bool enabled );
				void cbha_set_water_drain_valve_info_changed( const rec::rpc::ClientInfoSet& );

				void cbha_set_gripper_valve1_changed( bool enabled );
				void cbha_set_gripper_valve1_info_changed( const rec::rpc::ClientInfoSet& );

				void cbha_set_gripper_valve2_changed( bool enabled );
				void cbha_set_gripper_valve2_info_changed( const rec::rpc::ClientInfoSet& );

				void grappler_store_positions_changed( const QVector< rec::robotino::rpc::GrapplerServoInfo>& data );
				void grappler_store_positions_info_changed( const rec::rpc::ClientInfoSet& );

				void grappler_set_power_changed( int line, bool power );
				void grappler_set_power_info_changed( const rec::rpc::ClientInfoSet& );

				void grappler_set_positions_changed( const QVector< rec::robotino::rpc::GrapplerServoInfo>& data );
				void grappler_set_positions_info_changed( const rec::rpc::ClientInfoSet& );

				void grappler_servos_changed( const QVector< rec::robotino::rpc::GrapplerServoInfo>& data );
				void grappler_servos_info_changed( const rec::rpc::ClientInfoSet& );

				void grappler_readings_changed( const QVector< rec::robotino::rpc::GrapplerServoInfo>& data );
				void grappler_readings_info_changed( const rec::rpc::ClientInfoSet& );

				void grappler_toggle_torque_changed();
				void grappler_toggle_torque_info_changed( const rec::rpc::ClientInfoSet& );

				void log_changed( const QString& publisher, const QString& message, int level );
				void log_info_changed( const rec::rpc::ClientInfoSet& );

				void set_log_level_changed( const QString& publisher, int level );
				void set_log_level_info_changed( const rec::rpc::ClientInfoSet& );

				void custom_message_changed( unsigned int id, const QByteArray& data );
				void custom_message_info_changed( const rec::rpc::ClientInfoSet& );

				void custom_message0_changed( const QByteArray& data );
				void custom_message0_info_changed( const rec::rpc::ClientInfoSet& );

				void custom_message1_changed( const QByteArray& data );
				void custom_message1_info_changed( const rec::rpc::ClientInfoSet& );

				void custom_message2_changed( const QByteArray& data );
				void custom_message2_info_changed( const rec::rpc::ClientInfoSet& );

				void custom_message3_changed( const QByteArray& data );
				void custom_message3_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect0_set_tilt_changed( double tilt );
				void kinect0_set_tilt_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect1_set_tilt_changed( double tilt );
				void kinect1_set_tilt_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect2_set_tilt_changed( double tilt );
				void kinect2_set_tilt_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect3_set_tilt_changed( double tilt );
				void kinect3_set_tilt_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect0_tilt_changed( double tilt );
				void kinect0_tilt_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect1_tilt_changed( double tilt );
				void kinect1_tilt_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect2_tilt_changed( double tilt );
				void kinect2_tilt_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect3_tilt_changed( double tilt );
				void kinect3_tilt_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect0_accel_changed( double x, double y, double z );
				void kinect0_accel_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect1_accel_changed( double x, double y, double z );
				void kinect1_accel_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect2_accel_changed( double x, double y, double z );
				void kinect2_accel_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect3_accel_changed( double x, double y, double z );
				void kinect3_accel_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect0_set_led_changed( unsigned int state );
				void kinect0_set_led_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect1_set_led_changed( unsigned int state );
				void kinect1_set_led_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect2_set_led_changed( unsigned int state );
				void kinect2_set_led_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect3_set_led_changed( unsigned int state );
				void kinect3_set_led_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect0_led_changed( unsigned int state );
				void kinect0_led_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect1_led_changed( unsigned int state );
				void kinect1_led_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect2_led_changed( unsigned int state );
				void kinect2_led_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect3_led_changed( unsigned int state );
				void kinect3_led_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect0_set_video_format_changed( unsigned int format );
				void kinect0_set_video_format_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect1_set_video_format_changed( unsigned int format );
				void kinect1_set_video_format_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect2_set_video_format_changed( unsigned int format );
				void kinect2_set_video_format_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect3_set_video_format_changed( unsigned int format );
				void kinect3_set_video_format_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect0_video_format_changed( unsigned int format );
				void kinect0_video_format_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect1_video_format_changed( unsigned int format );
				void kinect1_video_format_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect2_video_format_changed( unsigned int format );
				void kinect2_video_format_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect3_video_format_changed( unsigned int format );
				void kinect3_video_format_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect0_set_depth_format_changed( unsigned int format );
				void kinect0_set_depth_format_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect1_set_depth_format_changed( unsigned int format );
				void kinect1_set_depth_format_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect2_set_depth_format_changed( unsigned int format );
				void kinect2_set_depth_format_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect3_set_depth_format_changed( unsigned int format );
				void kinect3_set_depth_format_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect0_depth_format_changed( unsigned int format );
				void kinect0_depth_format_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect1_depth_format_changed( unsigned int format );
				void kinect1_depth_format_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect2_depth_format_changed( unsigned int format );
				void kinect2_depth_format_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect3_depth_format_changed( unsigned int format );
				void kinect3_depth_format_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect0_depth_changed( const QByteArray& data, const QByteArray& object_data, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );
				void kinect0_depth_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect1_depth_changed( const QByteArray& data, const QByteArray& object_data, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );
				void kinect1_depth_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect2_depth_changed( const QByteArray& data, const QByteArray& object_data, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );
				void kinect2_depth_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect3_depth_changed( const QByteArray& data, const QByteArray& object_data, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );
				void kinect3_depth_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect0_video_changed( const QByteArray& data, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );
				void kinect0_video_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect1_video_changed( const QByteArray& data, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );
				void kinect1_video_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect2_video_changed( const QByteArray& data, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );
				void kinect2_video_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect3_video_changed( const QByteArray& data, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );
				void kinect3_video_info_changed( const rec::rpc::ClientInfoSet& );

				void process_status_changed( const rec::robotino::rpc::ProcessStatus& status );
				void process_status_info_changed( const rec::rpc::ClientInfoSet& );

				void process_output_changed( const rec::robotino::rpc::ProcessOutput& output );
				void process_output_info_changed( const rec::rpc::ClientInfoSet& );

				void disconnect_api1_clients_changed();
				void disconnect_api1_clients_info_changed( const rec::rpc::ClientInfoSet& );

				void api1_setstate_received_changed();
				void api1_setstate_received_info_changed( const rec::rpc::ClientInfoSet& );

				void poseOnMap_changed( const QPointF&, double );
				void poseOnMap_info_changed(  const rec::rpc::ClientInfoSet& );

				void initialPose_changed( const QPointF&, double );
				void initialPose_info_changed(  const rec::rpc::ClientInfoSet& );

				void navGoal_changed( const QPointF&, double );
				void navGoal_info_changed(  const rec::rpc::ClientInfoSet& );

				//***************CHARGER**********************

				void charger0_info_changed( unsigned int time, float batteryVoltage, float chargingCurrent, float bat1temp, float bat2temp, int state_number, const QString& state );
				void charger0_info_info_changed( const rec::rpc::ClientInfoSet& );

				void charger0_version_changed( int major, int minor, int patch );
				void charger0_version_info_changed( const rec::rpc::ClientInfoSet& );

				void charger0_error_changed( unsigned int time, const QString& message );
				void charger0_error_info_changed( const rec::rpc::ClientInfoSet& );

				void charger0_clear_error_changed();
				void charger0_clear_error_info_changed( const rec::rpc::ClientInfoSet& );

				void charger1_info_changed( unsigned int time, float batteryVoltage, float chargingCurrent, float bat1temp, float bat2temp, int state_number, const QString& state );
				void charger1_info_info_changed( const rec::rpc::ClientInfoSet& );

				void charger1_version_changed( int major, int minor, int patch );
				void charger1_version_info_changed( const rec::rpc::ClientInfoSet& );

				void charger1_error_changed( unsigned int time, const QString& message );
				void charger1_error_info_changed( const rec::rpc::ClientInfoSet& );

				void charger1_clear_error_changed();
				void charger1_clear_error_info_changed( const rec::rpc::ClientInfoSet& );

				void charger2_info_changed( unsigned int time, float batteryVoltage, float chargingCurrent, float bat1temp, float bat2temp, int state_number, const QString& state );
				void charger2_info_info_changed( const rec::rpc::ClientInfoSet& );

				void charger2_version_changed( int major, int minor, int patch );
				void charger2_version_info_changed( const rec::rpc::ClientInfoSet& );

				void charger2_error_changed( unsigned int time, const QString& message );
				void charger2_error_info_changed( const rec::rpc::ClientInfoSet& );

				void charger2_clear_error_changed();
				void charger2_clear_error_info_changed( const rec::rpc::ClientInfoSet& );

				void sensors_changed( const QVector< QString >& names, const QVector< float >& values, const QVector< QString >& units );
				void sensors_info_changed( const rec::rpc::ClientInfoSet& );


				/*Factory 4 ********************************************************/

				void mclayout_changed(const QByteArray& jsonData);
				void mclayout_info_changed(const rec::rpc::ClientInfoSet&);

				void mcstatus_changed(const QByteArray& jsonData);
				void mcstatus_info_changed(const rec::rpc::ClientInfoSet&);

				void pathnetwork_changed(const QByteArray& jsonData);
				void pathnetwork_info_changed(const rec::rpc::ClientInfoSet&);

				void pathnetwork_edited_changed(const QByteArray& jsonData);
				void pathnetwork_edited_info_changed(const rec::rpc::ClientInfoSet&);

				void localizationMode_changed(const QByteArray& jsonData);
				void localizationMode_info_changed(const rec::rpc::ClientInfoSet&);

				void smartlog_changed(const QByteArray& jsonData);
				void smartlog_info_changed(const rec::rpc::ClientInfoSet&);

				void smartnavigationplan_changed(const QByteArray& jsonData);
				void smartnavigationplan_info_changed(const rec::rpc::ClientInfoSet&);

				void smartlocations_changed(const QByteArray& jsonData);
				void smartlocations_info_changed(const rec::rpc::ClientInfoSet&);

				void smartlocations_edited_changed(const QByteArray& jsonData);
				void smartlocations_edited_info_changed(const rec::rpc::ClientInfoSet&);

				void smartrobotinfo_changed(const QByteArray& jsonData);
				void smartrobotinfo_info_changed(const rec::rpc::ClientInfoSet&);

				void smartmyrobotid_changed(const QByteArray& jsonData);
				void smartmyrobotid_info_changed(const rec::rpc::ClientInfoSet&);

				void smartjoblist_changed(const QByteArray& jsonData);
				void smartjoblist_info_changed(const rec::rpc::ClientInfoSet&);

				void smartlicense_changed(const QByteArray& jsonData);
				void smartlicense_info_changed(const rec::rpc::ClientInfoSet&);

				void smartdockingvis_changed(const QByteArray& jsonData);
				void smartdockingvis_info_changed(const rec::rpc::ClientInfoSet&);

				void mapDir_changed(const QByteArray& jsonData);
				void mapDir_info_changed(const rec::rpc::ClientInfoSet&);

				void fleetcom_response_changed(const QByteArray& jsonData);
				void fleetcom_response_info_changed(const rec::rpc::ClientInfoSet&);

				void fleetcom_request_changed( const QString& message );
				void fleetcom_request_info_changed( const rec::rpc::ClientInfoSet& );

				void mapDir_request_changed(const QByteArray& jsonData, const QByteArray& data);
				void mapDir_request_info_changed(const rec::rpc::ClientInfoSet&);

				void mapDir_response_changed(const QByteArray& jsonData, const QByteArray& data);
				void mapDir_response_info_changed(const rec::rpc::ClientInfoSet&);

				void map_changed(const QByteArray&, int width, int height, float resolution, float offsetx, float offsety);
				void map_info_changed(const rec::rpc::ClientInfoSet&);

				void mapPlanner_changed(const QByteArray&, int width, int height, float resolution, float offsetx, float offsety);
				void mapPlanner_info_changed(const rec::rpc::ClientInfoSet&);

				void mapPlannerEdited_changed(const QByteArray&, int width, int height, float resolution, float offsetx, float offsety);
				void mapPlannerEdited_info_changed(const rec::rpc::ClientInfoSet&);

				void json_message_with_data_changed(int topic, const QByteArray& jsonData, const QByteArray& data);
				void json_message_with_data_info_changed(int topic, const rec::rpc::ClientInfoSet&);

			private Q_SLOTS:
				void on_connected();
				void on_disconnected( rec::rpc::ErrorCode error );
				void on_stateChanged( QAbstractSocket::SocketState state );
				void on_error( QAbstractSocket::SocketError socketError, const QString& errorString );

			private:
				bool setParameters( const QMap< QString, QVariant >& values );
				void saveUserParameters();
				
				QString _parameters_path;
				QString _user_parameters_path;
				QMap< QString, QVariant > _parameters;
				QMap< QString, QVariant > _user_parameters;

				typedef void (rec::robotino::rpc::Client::*EnableFunc)(bool);

				QMap<QString, EnableFunc> _enableMap;
				QMap<int, EnableFunc> _json_message_with_data_topics_enableMap;
				QMap<int,QString> _json_message_with_data_topics_alias_map;
				QMap<QString,int> _json_message_with_data_topics_alias_reverse_map;

				DECLARE_TOPICLISTENER( rec_robotino_rpc_image0 )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_image0 );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_camera0_settings )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_camera0_settings );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_camera0_control )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_camera0_control );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_camera0_settings )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_camera0_settings );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_camera0_capabilities)
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_camera0_capabilities );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_camera0_calibration)
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_camera0_calibration );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_image1 )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_image1 );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_camera1_settings )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_camera1_settings );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_camera1_control )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_camera1_control );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_camera1_settings )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_camera1_settings );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_camera1_capabilities)
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_camera1_capabilities );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_camera1_calibration)
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_camera1_calibration );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_image2 )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_image2 );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_camera2_settings )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_camera2_settings );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_camera2_control )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_camera2_control );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_camera2_settings )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_camera2_settings );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_camera2_capabilities)
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_camera2_capabilities );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_camera2_calibration)
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_camera2_calibration );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_image3 )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_image3 );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_camera3_settings )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_camera3_settings );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_camera3_control )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_camera3_control );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_camera3_settings )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_camera3_settings );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_camera3_capabilities)
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_camera3_capabilities );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_camera3_calibration)
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_camera3_calibration );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_motor0_setpoint )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_motor0_setpoint );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_motor0_reset_position )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_motor0_reset_position );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_motor0_mode )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_motor0_mode );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_motor0_mode )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_motor0_mode );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_motor1_setpoint )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_motor1_setpoint );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_motor1_reset_position )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_motor1_reset_position );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_motor1_mode )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_motor1_mode );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_motor1_mode )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_motor1_mode );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_motor2_setpoint )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_motor2_setpoint );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_motor2_reset_position )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_motor2_reset_position );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_motor2_mode )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_motor2_mode );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_motor2_mode )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_motor2_mode );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_motor3_setpoint )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_motor3_setpoint );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_motor3_reset_position )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_motor3_reset_position );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_motor3_mode )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_motor3_mode );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_motor3_mode )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_motor3_mode );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_motor_setpoints )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_motor_setpoints );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_motor_readings )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_motor_readings );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_omnidrive )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_omnidrive );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_odometry )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_odometry );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_odometry )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_odometry );

				DECLARE_TOPICLISTENER(rec_robotino_rpc_pose)
				DECLARE_TOPICINFOCHANGED(rec_robotino_rpc_pose);

				DECLARE_TOPICLISTENER( rec_robotino_rpc_northstar )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_northstar );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_northstar_parameters )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_northstar_parameters );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_gyroscope )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_gyroscope );

				DECLARE_TOPICLISTENER(rec_robotino_rpc_gyroscope_ext)
				DECLARE_TOPICINFOCHANGED(rec_robotino_rpc_gyroscope_ext);

				DECLARE_TOPICLISTENER( rec_robotino_rpc_emergency_bumper )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_emergency_bumper );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_emergency_bumper )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_emergency_bumper );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_display_text )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_display_text );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_display_backlight)
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_display_backlight );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_display_buttons )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_display_buttons );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_display_vbar )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_display_vbar );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_display_hbar )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_display_hbar );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_display_progress )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_display_progress );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_display_clear )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_display_clear );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_parameters )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_parameters );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_ea_version )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_ea_version );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_ea09_ip4address )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_ea09_ip4address );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_ea09_ip4address )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_ea09_ip4address );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_clients_connected )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_clients_connected );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_digital_input )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_digital_input );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_analog_input )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_analog_input );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_distance_sensors )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_distance_sensors );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_bumper )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_bumper );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_battery )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_battery );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_digital_output )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_digital_output );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_digital_output_array )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_digital_output_array );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_relay )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_relay );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_relay_array )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_relay_array );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_shutdown )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_shutdown );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_power_button )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_power_button );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_request_shutdown )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_request_shutdown );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_pid_parameters)
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_pid_parameters );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_gripper_state )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_gripper_state );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_scan0 )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_scan0 );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_scan1 )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_scan1 );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_scan2 )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_scan2 );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_scan3 )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_scan3 );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_cbha_readings )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_cbha_readings );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_cbha_set_pressure )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_cbha_set_pressure );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_cbha_set_compressors_enabled )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_cbha_set_compressors_enabled );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_cbha_set_water_drain_valve )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_cbha_set_water_drain_valve );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_cbha_set_gripper_valve1 )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_cbha_set_gripper_valve1 );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_cbha_set_gripper_valve2 )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_cbha_set_gripper_valve2 );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_grappler_store_positions )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_grappler_store_positions );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_grappler_set_power )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_grappler_set_power );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_grappler_set_positions )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_grappler_set_positions );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_grappler_servos )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_grappler_servos );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_grappler_readings )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_grappler_readings );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_grappler_toggle_torque )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_grappler_toggle_torque );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_log )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_log );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_log_level )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_log_level );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_custom_message )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_custom_message );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_custom_message0 )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_custom_message0 );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_custom_message1 )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_custom_message1 );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_custom_message2 )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_custom_message2 );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_custom_message3 )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_custom_message3 );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect0_set_tilt )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect1_set_tilt )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect2_set_tilt )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect3_set_tilt )

				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect0_tilt )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect1_tilt )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect2_tilt )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect3_tilt )

				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect0_accel )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect1_accel )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect2_accel )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect3_accel )

				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect0_set_led )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect1_set_led )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect2_set_led )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect3_set_led )

				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect0_led )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect1_led )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect2_led )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect3_led )

				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect0_set_video_format )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect1_set_video_format )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect2_set_video_format )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect3_set_video_format )

				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect0_video_format )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect1_video_format )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect2_video_format )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect3_video_format )

				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect0_set_depth_format )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect1_set_depth_format )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect2_set_depth_format )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect3_set_depth_format )

				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect0_depth_format )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect1_depth_format )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect2_depth_format )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect3_depth_format )

				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect0_depth )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect1_depth )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect2_depth )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect3_depth )

				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect0_video )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect1_video )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect2_video )
				DECLARE_TOPICLISTENER( rec_robotino_rpc_kinect3_video )

				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect0_set_tilt )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect1_set_tilt )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect2_set_tilt )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect3_set_tilt )

				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect0_tilt )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect1_tilt )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect2_tilt )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect3_tilt )

				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect0_accel )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect1_accel )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect2_accel )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect3_accel )

				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect0_set_led )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect1_set_led )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect2_set_led )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect3_set_led )

				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect0_led )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect1_led )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect2_led )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect3_led )

				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect0_set_video_format )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect1_set_video_format )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect2_set_video_format )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect3_set_video_format )

				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect0_video_format )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect1_video_format )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect2_video_format )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect3_video_format )

				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect0_set_depth_format )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect1_set_depth_format )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect2_set_depth_format )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect3_set_depth_format )

				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect0_depth_format )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect1_depth_format )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect2_depth_format )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect3_depth_format )

				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect0_depth )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect1_depth )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect2_depth )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect3_depth )

				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect0_video )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect1_video )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect2_video )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect3_video )

				DECLARE_TOPICLISTENER( rec_robotino_rpc_process_status )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_process_status )

				DECLARE_TOPICLISTENER( rec_robotino_rpc_process_output )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_process_output )

				DECLARE_TOPICLISTENER( rec_robotino_rpc_disconnect_api1_clients )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_disconnect_api1_clients )

				DECLARE_TOPICLISTENER( rec_robotino_rpc_api1_setstate_received )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_api1_setstate_received )


				

				DECLARE_TOPICLISTENER( rec_robotino_rpc_poseOnMap )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_poseOnMap );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_initialPose )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_initialPose );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_navGoal )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_navGoal );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_charger0_info );
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_charger0_info );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_charger0_version );
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_charger0_version );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_charger0_error);
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_charger0_error );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_charger0_clear_error);
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_charger0_clear_error );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_charger1_info );
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_charger1_info );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_charger1_version );
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_charger1_version );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_charger1_error);
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_charger1_error );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_charger1_clear_error);
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_charger1_clear_error );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_charger2_info );
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_charger2_info );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_charger2_version );
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_charger2_version );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_charger2_error);
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_charger2_error );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_charger2_clear_error);
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_charger2_clear_error );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_sensors);
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_sensors );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_fleetcom_request);
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_fleetcom_request );

				/*Factory 4****************************************************/

				JSON_TOPIC_PRIVATE_DEFINITIONS(mclayout)
				JSON_TOPIC_PRIVATE_DEFINITIONS(mcstatus)
				JSON_TOPIC_PRIVATE_DEFINITIONS(pathnetwork)
				JSON_TOPIC_PRIVATE_DEFINITIONS(pathnetwork_edited);
				JSON_TOPIC_PRIVATE_DEFINITIONS(localizationMode);
				JSON_TOPIC_PRIVATE_DEFINITIONS(smartlog);
				JSON_TOPIC_PRIVATE_DEFINITIONS(smartnavigationplan)
				JSON_TOPIC_PRIVATE_DEFINITIONS(smartlocations)
				JSON_TOPIC_PRIVATE_DEFINITIONS(smartlocations_edited)
				JSON_TOPIC_PRIVATE_DEFINITIONS(smartrobotinfo)
				JSON_TOPIC_PRIVATE_DEFINITIONS(smartmyrobotid)
				JSON_TOPIC_PRIVATE_DEFINITIONS(smartjoblist)
				JSON_TOPIC_PRIVATE_DEFINITIONS(smartlicense)
				JSON_TOPIC_PRIVATE_DEFINITIONS(smartdockingvis)
				JSON_TOPIC_PRIVATE_DEFINITIONS(fleetcom_response)
				JSON_TOPIC_PRIVATE_DEFINITIONS(mapDir)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(mapDir_response)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(mapDir_request)

				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_0)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_1)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_2)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_3)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_4)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_5)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_6)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_7)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_8)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_9)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_10)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_11)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_12)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_13)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_14)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_15)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_16)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_17)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_18)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_19)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_20)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_21)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_22)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_23)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_24)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_25)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_26)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_27)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_28)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_29)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_30)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_31)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_32)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_33)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_34)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_35)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_36)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_37)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_38)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_39)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_40)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_41)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_42)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_43)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_44)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_45)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_46)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_47)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_48)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_49)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_50)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_51)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_52)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_53)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_54)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_55)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_56)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_57)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_58)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_59)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_60)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_61)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_62)
				JSON_WITH_DATA_TOPIC_PRIVATE_DEFINITIONS(json_message_with_data_topic_63)


				DECLARE_TOPICLISTENER(rec_robotino_rpc_map)
				DECLARE_TOPICINFOCHANGED(rec_robotino_rpc_map);

				DECLARE_TOPICLISTENER(rec_robotino_rpc_mapPlanner)
				DECLARE_TOPICINFOCHANGED(rec_robotino_rpc_mapPlanner);

				DECLARE_TOPICLISTENER(rec_robotino_rpc_mapPlannerEdited)
				DECLARE_TOPICINFOCHANGED(rec_robotino_rpc_mapPlannerEdited);

				
				
			};
		}
	}
}

#endif //_REC_ROBOTINO_RPC_CLIENT_H_
