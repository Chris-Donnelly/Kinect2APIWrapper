# Kinect2APIWrapper
## C++ wrapper for Kinect V2 (Kinect for Windows) API

(Created for MSc Dissertation Project "Virtual Paintburhs")

Static API wrapper class to manage Kinect V2 sensor, which initializes and uses polling (via update/deltatime) to retrieve skeletal and IR (infrared camera) data from the sensor, as well as other attributes the sensor or API measures (such as percieved height, body count etc). This wrapper is partially complete, as implementations are missing for user identification, multiple bodies andexporting  Infrared data to targets/wrappers.

### Initialization
Using the following member, the sensor is initialized (detected). Returns a bool (true=success, false=fail)
```C++
bKinect = Kinect2API::Initialize();
```


### Polling
To poll the sensor for new information, use the **update** method (sending a delta time and world time as a float; these need not be precise values, however, this can break the FPS timing code)

```C++
Kinect2API::Update(deltaT, worldT);
```

The wrapper will now have the latest available information available regarding the sensor (and API states). From here, check the sensor is detected as still functioning (returned value is bool; true is connected and working, false is not):

```C++
Kinect2API::StatusIsWorking()
```

### Data 'Frames'

*If the sensor is no longer working or connected (detected by the above call), or the API/service has failed, the wrapper will automatically shut down, and need to be initialized again (above)*

If the result is true, we can use the KinectV2JointReader classes (see [KinectV2JointFiltering](https://github.com/Chris-Donnelly/KinectV2JointFiltering)) to extract skeletal data as a KinectV2BodyFrame (defaults to closest user, but this can be modified manually to use user identifiers etc).

Infrared frames can be retrieved as raw UINT16 data, or a simple wrapper class (**KinectV2IRFrame**) can be used to encapsulate the data.

*Several material structures and texture rendering sections have been removed to keep the code focused on the sensor itself instead of graphics APIs etc*
