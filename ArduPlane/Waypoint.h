#ifndef WAYPOINT_H
#define WAYPOINT_H

// Waypoint Class Definition
class Waypoint
{
  public:    
    // Constructor
    Waypoint(void);
    
    // Initialization Routine
    void Initialize(void);
    
 private:
    float mX;
    float mY;
    float mZ;
    float mCaptureRadius;
    float mCaptureState;
    float mCaptureLineSlope;
    uint32_t mPrevWaypointId;
    uint32_t mWaypointId;
    uint32_t mNextWaypointId;
    uint32_t mWaypointType;
};














#endif /* WAYPOINT_H */
