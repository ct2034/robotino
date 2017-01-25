#ifndef _REC_ROBOTINO_RPC_SERVER_H_
#define _REC_ROBOTINO_RPC_SERVER_H_

#include <QtCore>
#include <QtNetwork>

#include "rec/rpc/Server.h"

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

namespace rec
{
	namespace robotino
	{
		namespace rpc
		{
			class REC_ROBOTINO_RPC_EXPORT Server : public rec::rpc::Server
			{
				Q_OBJECT
			public:
				Server( QObject* parent = 0 );
				virtual ~Server();

				void init();

				void readParameters( const QString& path, const QString& userpath );

				void setParameter( const QString& key, const QVariant& value );
				QVariant getParameter( const QString& key, const QVariant& defaultValue = QVariant() ) const;
				QMap< QString, QVariant > getParameters() const;
				bool containsParameter( const QString& key );

				QString clientName( const QHostAddress& address, quint16 port ) const;

			public Q_SLOTS:
				bool set_motor0_setpoint( float speed );
				bool set_motor1_setpoint( float speed );
				bool set_motor2_setpoint( float speed );
				bool set_motor3_setpoint( float speed );
				bool set_set_camera0_settings( unsigned int width, unsigned int height, const QString& format );
				bool set_odometry( double x, double y, double phi );
				bool set_set_odometry( double x, double y, double phi );
				bool set_pose(double x, double y, double phi, double errx, double erry, double errphi, unsigned int sequence);
				bool set_omnidrive( float vx, float vy, float omega );
				bool set_set_digital_output( unsigned int index, bool state );
				bool set_set_relay( unsigned int index, bool state );
				bool set_set_pid_parameters( unsigned int motor, float kp, float ki, float kd );
				bool set_set_motor3_mode( const QString& mode );
				bool set_northstar( unsigned int sequenceNumber,
										unsigned int roomId,
										unsigned int numSpotsVisible,
										float posX,
										float posY,
										float posTheta,
										unsigned int magSpot0,
										unsigned int magSpot1 );
				bool set_set_northstar_parameters( unsigned int roomId, float ceilingCal );
				bool set_request_shutdown( bool doit );
				bool set_log( const QString& message, int level );
				bool set_process_status( const rec::robotino::rpc::ProcessStatus& status );
				bool set_process_output( const rec::robotino::rpc::ProcessOutput& output );
				bool set_motor_readings( const QVector< float >& speeds, const QVector< int >& positions, const QVector< float >& currents, float time_delta );
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
				bool set_distance_sensors( const QVector< float >& );
				bool set_bumper( bool );

				bool set_scan( unsigned int laserNumber, QVector< float > ranges, QVector< float > intensities, QVariantMap parameters);
				bool set_scan0( QVector< float > ranges, QVector< float > intensities, QVariantMap parameters );
				bool set_scan1( QVector< float > ranges, QVector< float > intensities, QVariantMap parameters );
				bool set_scan2( QVector< float > ranges, QVector< float > intensities, QVariantMap parameters );
				bool set_scan3( QVector< float > ranges, QVector< float > intensities, QVariantMap parameters );

				bool set_kinect_depth( unsigned int id, const QByteArray& data, const QByteArray& object_data, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );
				bool set_kinect0_depth( const QByteArray& data, const QByteArray& object_data, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );
				bool set_kinect1_depth( const QByteArray& data, const QByteArray& object_data, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );
				bool set_kinect2_depth( const QByteArray& data, const QByteArray& object_data, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );
				bool set_kinect3_depth( const QByteArray& data, const QByteArray& object_data, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );

				bool set_digital_input( const QVector< bool >& );
				bool set_analog_input( const QVector< float >& );
				bool set_battery( float battery_voltage, float system_current );
				bool set_gripper_state( int state );
				bool set_grappler_servos( const QVector< rec::robotino::rpc::GrapplerServoInfo >& data );
				bool set_grappler_readings( const QVector< rec::robotino::rpc::GrapplerServoInfo >& data );

				bool set_cbha_readings( const QVector< float >& pressures, bool pressureSensor, const QVector< float >& stringPots, float foilPot );

			Q_SIGNALS:
				void serverError( const QString& );
				void logLevelChanged( int verbosity );

				void camera0_settings_changed( unsigned int width, unsigned int height, const QString& format );
				void camera0_settings_info_changed( const rec::rpc::ClientInfoSet& );

				void camera0_capabilities_changed( const QMap<QString, QVector<QSize> >& capabilities );
				void camera0_capabilities_info_changed( const rec::rpc::ClientInfoSet& );

				void image0_changed( const QByteArray& data,
									unsigned int width,
									unsigned int height,
									unsigned int step,
									const QString& format );
				void image0_info_changed( const rec::rpc::ClientInfoSet& );

				void motor_readings_changed( const QVector< float >& speeds, const QVector< int >& positions, const QVector< float >& currents, float time_delta );
				void motor_readings_info_changed( unsigned int motorId, const rec::rpc::ClientInfoSet& );

				void omnidrive_changed( float vx, float vy, float omega );
				void omnidrive_info_changed( const rec::rpc::ClientInfoSet& );

				void odometry_changed( double x, double y, double phi );
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

				void gyroscope_changed( double phi, double rate );
				void gyroscope_info_changed( const rec::rpc::ClientInfoSet& );

				void emergency_bumper_changed( bool enable );
				void emergency_bumper_info_changed( const rec::rpc::ClientInfoSet& );

				void set_emergency_bumper_changed( bool enable );
				void set_emergency_bumper_info_changed( const rec::rpc::ClientInfoSet& );

				void display_text_changed( const QString& text, unsigned int row, unsigned int col, bool clear_before, bool clear_after );
				void display_text_info_changed( const rec::rpc::ClientInfoSet& );

				void display_backlight_changed( bool on );
				void display_backlight_info_changed( const rec::rpc::ClientInfoSet& );

				bool display_buttons_changed( bool up, bool down, bool back, bool enter );
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

				void ea_version_changed( const QString& board, quint16 firmware_major, quint16 firmware_minor, quint16 firmware_patch );
				void ea_version_info_changed( const rec::rpc::ClientInfoSet& );

				void ea09_ip4address_changed( const QHostAddress& address, const QHostAddress& netmask );
				void ea09_ip4address_info_changed( const rec::rpc::ClientInfoSet& );

				void set_ea09_ip4address_changed( const QHostAddress& address, const QHostAddress& netmask );
				void set_ea09_ip4address_info_changed( const rec::rpc::ClientInfoSet& );

				void digital_input_changed( const QVector< bool >& );
				void digital_input_info_changed( const rec::rpc::ClientInfoSet& );

				void analog_input_changed( const QVector< float >& );
				void analog_input_info_changed( const rec::rpc::ClientInfoSet& );

				void distance_sensors_changed( const QVector< float >& );
				void distance_sensors_info_changed( const rec::rpc::ClientInfoSet& );

				void bumper_changed( bool );
				void bumper_info_changed( const rec::rpc::ClientInfoSet& );

				void battery_changed( float battery_voltage, float system_current );
				void battery_info_changed( const rec::rpc::ClientInfoSet& );

				void set_digital_output_changed( unsigned int index, bool state );
				void set_digital_output_info_changed( const rec::rpc::ClientInfoSet& );

				void set_digital_output_array_changed( const QVector< bool >& );
				void set_digital_output_array_info_changed( const rec::rpc::ClientInfoSet& );

				void set_relay_changed( unsigned int index, bool state );
				void set_relay_info_changed( const rec::rpc::ClientInfoSet& );

				void set_relay_array_changed( const QVector< bool >& );
				void set_relay_array_info_changed( const rec::rpc::ClientInfoSet& );

				void gripper_state_changed( int state );
				void gripper_state_info_changed( const rec::rpc::ClientInfoSet& );

				void power_button_changed( bool pressed );
				void power_button_info_changed( const rec::rpc::ClientInfoSet& );

				void set_log_level_changed( const QString& publisher, int level );

				void motor_setpoints_changed( float m0, float m1, float m2, const rec::rpc::ClientInfo& );
				void motor_setpoints_info_changed( const rec::rpc::ClientInfoSet& );

				void motor0_setpoint_changed( float speed, const rec::rpc::ClientInfo& );
				void motor0_setpoint_info_changed( const rec::rpc::ClientInfoSet& );

				void set_motor0_mode_changed( const QString& mode );
				void set_motor0_mode_info_changed( const rec::rpc::ClientInfoSet& );

				void motor0_mode_changed( const QString& mode );
				void motor0_mode_info_changed( const rec::rpc::ClientInfoSet& );

				void motor1_setpoint_changed( float speed, const rec::rpc::ClientInfo& );
				void motor1_setpoint_info_changed( const rec::rpc::ClientInfoSet& );

				void set_motor1_mode_changed( const QString& mode );
				void set_motor1_mode_info_changed( const rec::rpc::ClientInfoSet& );

				void motor1_mode_changed( const QString& mode );
				void motor1_mode_info_changed( const rec::rpc::ClientInfoSet& );

				void motor2_setpoint_changed( float speed, const rec::rpc::ClientInfo& );
				void motor2_setpoint_info_changed( const rec::rpc::ClientInfoSet& );

				void set_motor2_mode_changed( const QString& mode );
				void set_motor2_mode_info_changed( const rec::rpc::ClientInfoSet& );

				void motor2_mode_changed( const QString& mode );
				void motor2_mode_info_changed( const rec::rpc::ClientInfoSet& );

				void motor3_setpoint_changed( float speed, const rec::rpc::ClientInfo& );
				void motor3_setpoint_info_changed( const rec::rpc::ClientInfoSet& );

				void set_motor3_mode_changed( const QString& mode );
				void set_motor3_mode_info_changed( const rec::rpc::ClientInfoSet& );

				void motor3_mode_changed( const QString& mode );
				void motor3_mode_info_changed( const rec::rpc::ClientInfoSet& );

				void grappler_set_positions_changed( const QVector< rec::robotino::rpc::GrapplerServoInfo >& servoInfo );
				void grappler_set_positions_info_changed( const rec::rpc::ClientInfoSet& );

				void grappler_set_power_changed( unsigned int line, bool power );
				void grappler_set_power_info_changed( const rec::rpc::ClientInfoSet& );

				void grappler_toggle_torque_changed();
				void grappler_toggle_torque_info_changed( const rec::rpc::ClientInfoSet& );

				void cbha_set_pressure_changed( const QVector< float >& pressures );
				void cbha_set_pressure_info_changed( const rec::rpc::ClientInfoSet& );

				void cbha_set_compressors_enabled_changed( bool enabled );
				void cbha_set_compressors_enabled_info_changed( const rec::rpc::ClientInfoSet& );

				void cbha_set_water_drain_valve_changed( bool value );
				void cbha_set_water_drain_valve_info_changed( const rec::rpc::ClientInfoSet& );

				void cbha_set_gripper_valve1_changed( bool value );
				void cbha_set_gripper_valve1_info_changed( const rec::rpc::ClientInfoSet& );

				void cbha_set_gripper_valve2_changed( bool value );
				void cbha_set_gripper_valve2_info_changed( const rec::rpc::ClientInfoSet& );

				void kinect0_depth_info_changed( const rec::rpc::ClientInfoSet& );
				void kinect1_depth_info_changed( const rec::rpc::ClientInfoSet& );
				void kinect2_depth_info_changed( const rec::rpc::ClientInfoSet& );
				void kinect3_depth_info_changed( const rec::rpc::ClientInfoSet& );

				void fleetcom_request_changed( const QString& message, const QVariantMap& pairs, unsigned int sequence );
				void fleetcom_request_info_changed( const rec::rpc::ClientInfoSet& );

				void fleetcom_response_changed( const QString& message, const QVariantMap& pairs, unsigned int sequence );
				void fleetcom_response_info_changed( const rec::rpc::ClientInfoSet& );

			protected:
				virtual void customHandler( const QByteArray& request, QByteArray* response, const QHostAddress& address, quint16 port ){};
				virtual int launchProcess( const QString& command, const QStringList& parameters, const QString& workingdirectory ){ return -1; };
				virtual int terminateProcess( int pid ){ return -1; };
				virtual int killProcess( int pid ){ return -1; };
				virtual QVector< int > getProcessIds() { return QVector<int>(); }
				virtual rec::robotino::rpc::ProcessStatus getProcessStatus( int id ) { return rec::robotino::rpc::ProcessStatus(); }

			private Q_SLOTS:
				void on_listening();
				void on_closed();
				void on_serverError( QAbstractSocket::SocketError error, const QString& errorString );
				void on_clientConnected( const rec::rpc::ClientInfo& );
				void on_clientDisconnected( const rec::rpc::ClientInfo& );

			private:
				QMap< QString, QVariant > _parameters;

				QString _name;
				QString _local_parameters_path;
				QString _local_user_parameters_path;
				QMap< QString, QVariant > _local_parameters;
				QMap< QString, QVariant > _local_user_parameters;
				QVector< rec::rpc::ClientInfo > _clients;

				void saveUserParameters();
				void publishParameters();
				void setParameters( const QMap< QString, QVariant >& values );

				DECLARE_FUNCTION( rec_robotino_rpc_set_parameter );
				DECLARE_FUNCTION( rec_robotino_rpc_set_parameters );
				DECLARE_FUNCTION( rec_robotino_rpc_get_parameter );
				DECLARE_FUNCTION( rec_robotino_rpc_get_parameters );
				DECLARE_FUNCTION( rec_robotino_rpc_remove_parameter );
				DECLARE_FUNCTION( rec_robotino_rpc_contains_parameter );


				DECLARE_FUNCTION( rec_robotino_rpc_disconnect_client );

				DECLARE_FUNCTION( rec_robotino_rpc_process_launch );
				DECLARE_FUNCTION( rec_robotino_rpc_process_terminate );
				DECLARE_FUNCTION( rec_robotino_rpc_process_kill );
				DECLARE_FUNCTION( rec_robotino_rpc_process_getids );
				DECLARE_FUNCTION( rec_robotino_rpc_process_getstatus );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_image0 );
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_image0 );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_camera0_settings )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_camera0_settings );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_camera0_capabilities)
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_camera0_capabilities );

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

				DECLARE_TOPICLISTENER( rec_robotino_rpc_display_progress)
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_display_progress );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_display_clear )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_display_clear );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_ea_version )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_ea_version );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_ea09_ip4address )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_ea09_ip4address );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_ea09_ip4address )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_ea09_ip4address );

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

				DECLARE_TOPICLISTENER( rec_robotino_rpc_gripper_state )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_gripper_state );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_power_button )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_power_button );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_log_level )

				DECLARE_TOPICLISTENER( rec_robotino_rpc_motor_setpoints )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_motor_setpoints );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_motor0_setpoint )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_motor0_setpoint );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_motor0_mode )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_motor0_mode );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_motor0_mode )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_motor0_mode );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_motor1_setpoint )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_motor1_setpoint );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_motor1_mode )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_motor1_mode );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_motor1_mode )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_motor1_mode );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_motor2_setpoint )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_motor2_setpoint );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_motor2_mode )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_motor2_mode );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_motor2_mode )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_motor2_mode );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_motor3_setpoint )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_motor3_setpoint );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_set_motor3_mode )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_set_motor3_mode );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_motor3_mode )
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_motor3_mode );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_grappler_toggle_torque );
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_grappler_toggle_torque );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_grappler_set_power );
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_grappler_set_power );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_grappler_set_positions );
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_grappler_set_positions );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_cbha_set_pressure );
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_cbha_set_pressure );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_cbha_set_compressors_enabled );
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_cbha_set_compressors_enabled );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_cbha_set_water_drain_valve );
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_cbha_set_water_drain_valve );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_cbha_set_gripper_valve1 );
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_cbha_set_gripper_valve1 );

				DECLARE_TOPICLISTENER( rec_robotino_rpc_cbha_set_gripper_valve2 );
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_cbha_set_gripper_valve2 );

				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect0_depth );
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect1_depth );
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect2_depth );
				DECLARE_TOPICINFOCHANGED( rec_robotino_rpc_kinect3_depth );
			};
		}
	}
}

#endif // _REC_ROBOTINO_RPC_SERVER_H_
