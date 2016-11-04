#ifndef _REC_ROBOTINO3_FLEETCOM_COMMAND_H_
#define _REC_ROBOTINO3_FLEETCOM_COMMAND_H_

#include <QtCore>

#include "rec/robotino3/fleetcom/CommandFwd.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class Command
			{
			public:
				typedef enum
				{
					RequestCommandType = 0
					, ResponseCommandType
				} CommandType;

				Command( CommandType commandType )
					: type( commandType )
				{
				}

				virtual ~Command() {};

				const CommandType type;
			};

			class Request : public Command
			{
			public:
				typedef enum
				{
					InvalidRequestId = 0
					, GotoPositionRequestId
					, DockToRequestId
					, UndockRequestId
					, LoadBoxRequestId
					, UnloadBoxRequestId
					, StopRequestId
					, ContinueRequestId
					, EndTaskRequestId
					, TeachPositionRequestId
					, DeletePositionRequestId
					, TeachCurrentPositionRequestId
					, GetAllPositionRequestId
					, ReloadPositionStorageRequestId
					, StartMappingRequestId
					, StopMappingRequestId
					, ReloadDefaultMapRequestId
                    , ReloadDefaultPathsRequestId
					, SetRobotPoseRequestId
					, PushJobRequestId
					, PushCommandRequestId
					, SetOperationModeRequestId
					, GetJobInfoRequestId
					, GetJobErrorRequestId
					, ClearErrorRequestId
					, AbortJobAndClearErrorRequestId
					, ManualAcknowledgeRequestId
					, GetAllRobotinoIdRequestId
					, GetMyRobotinoIdRequestId
					, GetRobotFleetTypeRequestId
					, GetFleetStateRequestId
					, GetRobotInfoRequestId
					, GetMapListRequestId
					, SetDefaultMapRequestId
					, UploadMapRequestId
					, DownloadMapRequestId
					, RenameMapRequestId
					, DeleteMapRequestId
                    , SavePathRequestId
                    , LoadPathRequestId
					, AbortMappingRequestId
				} RequestId;

				Request( RequestId requestId )
					: Command( Command::RequestCommandType )
					, id( requestId )
				{
				}

				virtual ~Request() {};

				virtual QString encode() const { return QString::null; }

				virtual QString print() const = 0;

				const RequestId id;
			};

			class Response : public Command
			{
			public:
				typedef enum
				{
					InvalidResponseId = 0
					, FleetStateResponseId
					, AllPositionResponseId
					, RobotFleetTypeResponseId
					, MyRobotinoIdResponseId
					, AllRobotinoIdResponseId
					, JobInfoResponseId
					, JobErrorResponseId
					, RobotInfoResponseId
					, MapListResponseId
					, DownloadMapResponseId
                    , LoadPathResponseId
					, ErrorResponseId
				} ResponseId;

				Response( ResponseId responseId, unsigned int sequence_ )
					: Command( Command::ResponseCommandType )
					, id( responseId )
					, sequence( sequence_ )
				{
				}

				virtual QString print() const = 0;
				virtual void encode(QString* message, QVariantMap* pairs) const {};

				const ResponseId id;

				const unsigned int sequence;
			};
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_COMMAND_H_
