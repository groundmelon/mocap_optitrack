In newer version of [NatNet](https://www.optitrack.com/products/natnet-sdk/), the data structure has changed:

In NatNetTypes.h: 178-193 (Version 2.9)

```c++
typedef struct sRigidBodyData
{
    int ID;                                 // RigidBody identifier
    float x, y, z;                          // Position
    float qx, qy, qz, qw;                   // Orientation
    int nMarkers;                           // Number of markers associated with this rigid body
    MarkerData* Markers;                    // Array of marker data ( [nMarkers][3] )
    int* MarkerIDs;                         // Array of marker IDs
    float* MarkerSizes;                     // Array of marker sizes
    float MeanError;                        // Mean measure-to-solve deviation
    short params;                           // Host defined tracking flags
    sRigidBodyData()
    {
        Markers = 0; MarkerIDs = 0; MarkerSizes = 0; params=0;
    }
} sRigidBodyData;
```

There is an extra `short params` field that indicates whether the rigid body is tracked or not.

However, in older version of NatNet Protocal, this field does not exist. See https://github.com/mkilling/PyNatNet/blob/master/libs/NatNetSDK/include/NatNetTypes.h#L161-L175 for reference. 

According to the [changelog](https://www.optitrack.com/downloads/developer-tools.html), this parameter is added since NatNet SDK v2.6

Usage of this variable:

In SampleClient.cpp: 395

```c++

for(i=0; i < data->nRigidBodies; i++)
	{
        // params
        // 0x01 : bool, rigid body was successfully tracked in this frame
        bool bTrackingValid = data->RigidBodies[i].params & 0x01;

		printf("Rigid Body [ID=%d  Error=%3.2f  Valid=%d]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError, bTrackingValid);

```
