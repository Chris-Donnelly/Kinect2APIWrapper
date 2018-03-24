#include "Kinect2API.h"
#include <math.h>
#include <iostream>
#include <sstream>
#include <algorithm>

#include "GLMTypes.h"


int						Kinect2API::s_PollRate;
int						Kinect2API::s_successfulReads;
float					Kinect2API::s_fHZCounter;
int						Kinect2API::s_ReadAttempts;
int						Kinect2API::s_FrameRate;
bool					Kinect2API::s_Status{ false };
int						Kinect2API::s_bodyCount{ 0 };
bool					Kinect2API::s_FreshBodyDataAvailable{ false };
bool					Kinect2API::s_FreshIRDataAvailable{ false };
float					Kinect2API::s_BodyConfidence{ 0.0f };
UINT16*					Kinect2API::s_IRFrame_Raw;
UINT32*					Kinect2API::s_IRFrame_RGBA;

Vec4					Kinect2API::s_FloorPlane{ Vec4(0.0f) };
Vec3					Kinect2API::s_KinectPosition{ Vec3(0.0f, 0.0f, 0.0f) };
HRESULT					Kinect2API::s_lastError{ S_OK };
IKinectSensor*			Kinect2API::s_pKinectSensor{ nullptr };
ICoordinateMapper*		Kinect2API::s_pCoordinateMapper{ nullptr };
IBodyFrameReader*		Kinect2API::s_pBodyFrameReader{ nullptr };
IInfraredFrameReader*	Kinect2API::s_pIRFrameReader{ nullptr };

std::map<JointType, Joint>					Kinect2API::m_joints;
std::map<JointType, JointOrientation>		Kinect2API::m_jointOrientations;

// Basic map of error codes associated with Kinect2
const std::map<int, char*> Kinect2API::KinectErrors{

	{ E_PENDING,		"[E_PENDING] Pending - Data not available now" },
	{ E_NOTIMPL,		"[E_NOTIMPL] Not implemented" },
	{ E_NOINTERFACE,	"[E_NOINTERFACE] No such interface supported" },
	{ E_POINTER,		"[E_POINTER] Invalid Pointer" },
	{ E_ABORT,			"[E_ABORT] Operation aborted" }

};

bool Kinect2API::Initialize() {

	HRESULT hr;
	
	// Get the default sensor object
	hr = GetDefaultKinectSensor(&s_pKinectSensor);

	if (FAILED(hr)) {

		return false;

	}

	if (s_pKinectSensor) {

		// Initialize the Kinect and get coordinate mapper and the body reader
		IBodyFrameSource* pBodyFrameSource = NULL;

		hr = s_pKinectSensor->Open();

		if (SUCCEEDED(hr)) {

			// Get the coordinate mapper
			hr = s_pKinectSensor->get_CoordinateMapper(&s_pCoordinateMapper);

		} else {

			// Failed to get Kinect V2 Coordinate Mapper

			Shutdown();

			return false;

		}

		if (SUCCEEDED(hr)) {

			// Get the body frame source reader
			hr = s_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);

		} else {

			// Failed to get Kinect V2 Body frame source

			Shutdown();

			return false;

		}

		if (SUCCEEDED(hr)) {

			// Open the frame source reader
			hr = pBodyFrameSource->OpenReader(&s_pBodyFrameReader);

		} else {

			//Failed to get Kinect V2 Body frame reader

			Shutdown();

			return false;

		}

		CleanRelease(pBodyFrameSource);

		// Open the IR frame source
		IInfraredFrameSource* pIRFrameSource{ nullptr };
		hr = s_pKinectSensor->get_InfraredFrameSource(&pIRFrameSource);

		if (FAILED(hr)) {

			// Failed to get IR Frame Source

			Shutdown();

			return false;

		}

		// Open the IR Frame Reader
		hr = pIRFrameSource->OpenReader(&s_pIRFrameReader);

		if (FAILED(hr)) {

			// Failed to open Kinect V2 IR Frame reader

			Shutdown();

			return false;

		}

		// Release frame source
		CleanRelease(pIRFrameSource);

	}

	// Create IR buffers
	s_IRFrame_Raw = new UINT16[Kinectafx::IRFRAME_TOTALSIZE];
	s_IRFrame_RGBA = new UINT32[Kinectafx::IRFRAME_TOTALSIZE];

	// If we haven't failed out...
	s_Status = true;

	return s_Status ;
}

void Kinect2API::Shutdown() {

	s_Status = false;

	// coordinate mapper
	CleanRelease(s_pCoordinateMapper);

	// body frame reader
	CleanRelease(s_pBodyFrameReader);

	// IR Reader
	CleanRelease(s_pIRFrameReader);

	// close / release the Kinect Sensor
	if (s_pKinectSensor) {

		if(FAILED(s_pKinectSensor->Close())) return;

		CleanRelease(s_pKinectSensor);

	}

	// Delete IR buffers
	CleanDelete(s_IRFrame_Raw);
	CleanDelete(s_IRFrame_RGBA);

}

// Get the body data
bool Kinect2API::GetJointData(std::map<JointType, Joint>& joints) {

	// Exit out if no fresh data is available
	if (!s_FreshBodyDataAvailable) {

		return false;

	}

	// Copy data into 'joints' map
	for (auto tmpJoint : m_joints) {

		joints[tmpJoint.first] = tmpJoint.second;

	}

	// This data is now dirty
	s_FreshBodyDataAvailable = false;

	return true;
}

bool Kinect2API::GetOrientationData(std::map<JointType, JointOrientation>& orientations) {

	// Exit out if no fresh data is available
	if (!s_FreshBodyDataAvailable)
		return false;

	// Copy data into 'orientations' map
	for (auto tmpJoint : m_jointOrientations) {

		orientations[tmpJoint.first] = tmpJoint.second;

	}

	// This data is now dirty
	s_FreshBodyDataAvailable = false;

	return true;

}

// Get the latest data from the sensor
void Kinect2API::Update(float dTime, float wTime) {

	// Increase time counter for frame rates by deltaTime
	s_fHZCounter += dTime;

	// Increment read attempts (polls)
	s_ReadAttempts++;

	// Update sensor status
	UpdateStatus();

	//don't bother if unavailable
	if (!s_pBodyFrameReader || !StatusIsWorking()) {

		return;

	}

	// ---- Check for latest Body Frame ----
	IBodyFrame* pBodyFrame{ nullptr };
	HRESULT hr_bodyFrame = s_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);

	//  Try to get a floor plane to get the sensor's height
	if (pBodyFrame) {

		// Get the floor plane (as detected)
		Vector4 floor{ 0.0f };
		pBodyFrame->get_FloorClipPlane(&floor);

		// If it's valid?
		s_FloorPlane = Vec4(floor.x, floor.y, floor.z, floor.w);
		s_KinectPosition.y = floor.w * Kinectafx::KINECT_METRES_TO_MM;
	}

	// If we have a body frame
	if (SUCCEEDED(hr_bodyFrame)) {

		// Successful data read. Increment read count
		s_successfulReads++;

		s_FreshBodyDataAvailable = true;

		IBody* ppBodies[BODY_COUNT] = { 0 };

		// Get new data
		if (SUCCEEDED(hr_bodyFrame)) {

			// Get the body data
			hr_bodyFrame = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);

		}

		if (SUCCEEDED(hr_bodyFrame)) {

			// Process the body data
			ProcessBody(BODY_COUNT, ppBodies);

		}

		// Release objects
		for (int i = 0; i < _countof(ppBodies); ++i) {

			CleanRelease(ppBodies[i]);

		}

	}
	// If the frame read did not succeed....
	else {

		// We may have an issue (E_PENDING is manageable, framerate slower than application)
		if (hr_bodyFrame == E_PENDING) {

			s_FreshBodyDataAvailable = false;

		}
		else {

			if (hr_bodyFrame != s_lastError) {

				// Error

			}

		}
			
	}

	CleanRelease(pBodyFrame);

	// ---- Check for latest IR Frame ----

	IInfraredFrame* pIRFrame{ nullptr };
	HRESULT hr_IRFrame = s_pIRFrameReader->AcquireLatestFrame(&pIRFrame);

	// If the frame read succeeded
	if (SUCCEEDED(hr_IRFrame)) {

		// Flag up
		s_FreshIRDataAvailable = true;

		UINT dBufferSize{ 0 };
		UINT16 *pBuffer{ nullptr };
		// Access the underlying buffer details
		HRESULT hr = pIRFrame->AccessUnderlyingBuffer(&dBufferSize, &pBuffer);

		if (FAILED(hr)) {

			// Failed to get IR frame buffer (AccessUnderlyingBuffer)

			return;

		}

		// Process the frame
		ProcessInfrared(pBuffer);

	}
	else {

		// We may have an issue (E_PENDING is manageable, framerate slower than application)
		if (hr_IRFrame == E_PENDING) {

			// Flag down
			s_FreshIRDataAvailable = false;

		}
		else {

			if (hr_IRFrame != s_lastError) {

				// AcquireLatestFrame (IR) Failed

			}

		}

	}

	// If the frame rate counter equals or exceeds 1sec, use data and reset counter(s)
	// Does accumulated delta time >= 1 second (0.95 sec, error margin)
	if (s_fHZCounter >= 0.95f) {

		// polling rate store/reset
		SetPollRate(s_ReadAttempts);
		s_ReadAttempts = 0;

		// frame rate store/reset
		SetFrameRate(s_successfulReads);
		s_successfulReads = 0;

		// Reset time counter
		s_fHZCounter = 0.0f;

	}

	// release body frame for new data
	CleanRelease(pIRFrame);

}

// Get a data frame
bool Kinect2API::GetDataFrame(KinectV2BodyFrame & destFrame) {

	// Build Kinect Frame (ignore sensor position and floor plane)
	destFrame = KinectV2BodyFrame(m_joints, m_jointOrientations, s_BodyConfidence, s_KinectPosition, s_FloorPlane);

	return true;

}

IBody * Kinect2API::GetNearestBody(int dBodyCount, IBody ** ppBodies) {

	IBody* result{ nullptr };
	float smallestZ{ 8000.0f };	// effectively out of bounds

	// Process the arrya of bodies
	for (int i = 0; i < dBodyCount; i++) {

		IBody* pBody = ppBodies[i];
		BOOLEAN bTracked = false;

		// Body is valid
		if (pBody) {

			// Is the body tracked?
			HRESULT hr = pBody->get_IsTracked(&bTracked);

			// If the body IS tracked
			if (SUCCEEDED(hr) && bTracked) {
			
				// Get a single joint
				Joint joints[JOINT_COUNT];
				HRESULT hr2 = pBody->GetJoints(JOINT_COUNT, joints);

				// We have a joint
				if (SUCCEEDED(hr2)) {

					// If this body has a spine base joint at lowest Z (closest to sensor)
					if (joints[JointType_SpineBase].Position.Z < smallestZ) {

						// store
						result = pBody;

					}
					
				}

				// Did not get joints successfully. Next.
			
			}

			// The body is not tracked. Next.

		}

		// Body is not valid. Next/exit

	}

	return result;
}

// Render IR data to texture/material
bool Kinect2API::RenderIRTo(Material* mtl) {

	// Exit out
	if (!HasFreshIRData()) {

		return false;

	}

	// Normalize data from S_IRFrame_Raw and store in S_IRFrame_RGBA
	for (long int i = 0; i < Kinectafx::IRFRAME_TOTALSIZE; i++) {

		// Kinect V2 SDK recommended filter technique
		float intensityRatio = (static_cast<float>(s_IRFrame_Raw[i]) / static_cast<float>(USHRT_MAX)) / (0.08f * 3.0f);
		intensityRatio = (1.0f < intensityRatio)? 1.0f : intensityRatio;
		intensityRatio = (0.01f > intensityRatio) ? 0.01f : intensityRatio;
		BYTE channel = static_cast<byte>(intensityRatio * 255.0f);

		// Build & store pixel
		s_IRFrame_RGBA[i] = static_cast<UINT32>(MAKELONG(MAKEWORD(channel, channel), MAKEWORD(channel, 255)));

	}

	// Update the material
	// ...

	// Flag back down; this frame is now old
	s_FreshIRDataAvailable = false;

	return true;

}

// Update operating status
void Kinect2API::UpdateStatus() {

	// Check sensors etc and update staus
#ifdef _DEBUG

	// Enables Kinect Studio recordings
	SetWorkingStatus(s_pKinectSensor != nullptr);

#else

	// Check for PHYSICAL sensor
	BOOLEAN isAvailable{ 0 };
	HRESULT hr = s_pKinectSensor->get_IsAvailable(&isAvailable);

	// Enables Kinect Studio recordings
	SetWorkingStatus(SUCCEEDED(hr) && (isAvailable != 0));

#endif

}

// Process bodies detected (from Kinect 2 SDK)
void Kinect2API::ProcessBody(int nBodyCount, IBody ** ppBodies) {

	// NOTE: Assumes the user body is the body NEAREST to the camera (lowest Z value in Camera/Depth space)

	int bodyCount{ 0 };
	// Cycle available bodies

	for (int i = 0; i < nBodyCount; ++i) {

		// Get current body (iterator)
		IBody* pCurrentBody = ppBodies[i];
		BOOLEAN bTracked = false;

		// Check body is tracked
		HRESULT hr = pCurrentBody->get_IsTracked(&bTracked);

		// If tracking check succeeded AND the body is tracked
		if (SUCCEEDED(hr) && bTracked) {

			// Increment counted bodies (if it is tracked)
			if (bTracked) bodyCount++;

		}

	}

	// If we have bodies to process
	if (bodyCount > 0) {

		// Get the nearest body to the camera
		IBody* Body = GetNearestBody(nBodyCount, ppBodies);

		// Get the joints
		Joint joints[JOINT_COUNT];
		HRESULT hr2 = Body->GetJoints(_countof(joints), joints);

		// Get the orientations
		JointOrientation jointOrientations[JOINT_COUNT];
		HRESULT hr3 = Body->GetJointOrientations(_countof(joints), jointOrientations);

		if (SUCCEEDED(hr2) && SUCCEEDED(hr3)) {
			s_FreshBodyDataAvailable = true;

			// Cycle the list of joints
			float iConfidence{ 0 };
			for (int j = 0; j < _countof(joints); ++j) {

				// Store joint locally
				m_joints[joints[j].JointType] = joints[j];

				// Compensate for higher Z position values by subtracting the Kinect's position

				m_joints[joints[j].JointType].Position = toPosition(toVec3(joints[j].Position) + (s_KinectPosition * Kinectafx::KINECT_MM_TO_METRES));

				// Store joint confidence for averaging
				iConfidence += static_cast<int>(joints[j].TrackingState);

				// Store orientation locally
				m_jointOrientations[joints[j].JointType] = jointOrientations[j];

			}

			// Calcluate joint confidence average as percentage
			s_BodyConfidence = ((iConfidence / JOINT_COUNT) / 2);

			// Body data is clean
			s_FreshBodyDataAvailable = true;
		}

		// Store processed bodies
		s_bodyCount = bodyCount;

	}

}

// Perform any necessary IR processing
void Kinect2API::ProcessInfrared(const UINT16* pBuffer) {

	// Chek validity
	if (!pBuffer) {

		// IR Frame has no data

		return;

	}

	// store data locally
	memcpy(s_IRFrame_Raw, pBuffer, Kinectafx::IRFRAME_TOTALSIZE * sizeof(UINT16));
	
}

// Set status
void Kinect2API::SetWorkingStatus(bool value) {

	s_Status = value;

}

// Return a text tescription of an associated HRESULT
std::string Kinect2API::GetErrorString(HRESULT hr) {

	// Look for value in map
	auto search = KinectErrors.find(hr);

	if (search != KinectErrors.end()) {

		// found!
		return search->second;

	}

	// Otherwise, output unknown code
	return "Unknown ";
}

// Status of sensor
bool Kinect2API::StatusIsWorking() {

	bool result{ false };

#ifdef NDEBUG
	
	// Check for PHYSICAL sensor
	BOOLEAN isAvailable{ 0 };
	s_pKinectSensor->get_IsAvailable(&isAvailable);

	// Enables Kinect Studio recordings
	result = s_Status && (isAvailable != 0);

#else

	// Enables Kinect Studio recordings
	result = s_Status;

#endif

	return result;

}