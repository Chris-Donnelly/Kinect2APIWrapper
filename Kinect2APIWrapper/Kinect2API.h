#pragma once

#include "Kinectafx.h"
#include "KinectV2BodyFrame.h"
#include "stdafx.h"

#include <map>
#include <array>

#include "KinectV2IRFrame.h"
#include "essentials.h"

#define JOINT_COUNT							JointType::JointType_Count	// Defined API Joint types
#define KINECT_BODY_CONFIDENCE_SCORE		JOINT_COUNT * 2				// 100% confidence (no inferred joints, etc)
#define KINECT_SENSOR_MIN_HEIGHT_EPSILON	20.0f						// Sensor height unit base is 9.9mm; ensure sensor >= 20mm from floor */

class Material;	// Material classes were removed in this repository, so simply declaring here.

class Kinect2API {

	private:

		static int				s_PollRate;
		static int				s_successfulReads;
		static float			s_fHZCounter;
		static int				s_ReadAttempts;
		static int				s_FrameRate;
		static bool				s_FreshBodyDataAvailable;
		static bool				s_FreshIRDataAvailable;
		static int				s_bodyCount;
		static float			s_BodyConfidence;
		static Vec4				s_FloorPlane;
		static Vec3				s_KinectPosition;
		static bool				s_Status;
		static float			s_deltaTime;
		static int				s_reads;

		static HRESULT					s_lastError;
		static IKinectSensor*			s_pKinectSensor;
		static ICoordinateMapper*		s_pCoordinateMapper;
		static IBodyFrameReader*		s_pBodyFrameReader;
		static IInfraredFrameReader*	s_pIRFrameReader;

		static UINT32*		s_IRFrame_RGBA;
		static UINT16*		s_IRFrame_Raw;

		static std::map<JointType, Joint>				m_joints;
		static std::map<JointType, JointOrientation>	m_jointOrientations;
		static const std::map<int, char*>				KinectErrors;
		
		static std::string		GetErrorString(HRESULT hr);
		static void				UpdateStatus();
		static void				ProcessBody(int nBodyCount, IBody** ppBodies);
		static void				ProcessInfrared(const UINT16* pBuffer);
		static void				SetWorkingStatus(bool value);
		static void				SetFrameRate(int rate) { s_FrameRate = rate; }
		static void				SetPollRate(int rate) { s_PollRate = rate; }

	public:

		Kinect2API() {};
		~Kinect2API() {};

		static bool		Initialize();
		static void		Shutdown();
		static void		Update(float dTime, float wTime);

		static int		GetFrameRate() { return s_FrameRate; }
		static int		GetPollRate() { return s_PollRate; }

		static bool		HasFreshBodyData() { return s_FreshBodyDataAvailable; }
		static bool		HasFreshIRData() { return s_FreshIRDataAvailable; }
		static bool		StatusIsWorking();
		static int		GetBodiesCount() { return s_bodyCount; }
		static bool		GetJointData(std::map<JointType, Joint> & joints);
		static bool		GetOrientationData(std::map<JointType, JointOrientation> & joints);
		static float	GetBodyConfidence() { return s_BodyConfidence; }
		static void		SetSensorPosition(Vec3 position) { s_KinectPosition = position; }
		static Vec3		GetSensorPosition() { return s_KinectPosition; }
		static bool		GetDataFrame(KinectV2BodyFrame& destFrame);
		static IBody*	GetNearestBody(int dBodyCount, IBody ** ppBodies);
		static bool		RenderIRTo(Material* mtl);

};
