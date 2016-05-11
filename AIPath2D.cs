/*
Copyright (C) 2014 Felix Gladisch

AIPath2D is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

AIPath2D is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Pathfinding;
using Pathfinding.RVO;

/** AI for following paths.
 * This AI is a reimplementation of the movement script "AIPath" which comes with the A* Pathfinding Project for Unity, but for 2D games. 
 * As it was stated in the documentation of the original script "It is in no way required by the rest of the system, so feel free to write your own.", and that is what I did. 
 * Calculations and transformations are made on the XY plane, which is the default for 2D Games in Unity, instead of the XZ plane that the original script is using. 
 * Please visit http://arongranberg.com/astar for more information.
 */
[RequireComponent(typeof(Seeker))]
[AddComponentMenu("Pathfinding/AI/AIPath2D (generic)")]
public class AIPath2D : MonoBehaviour
{

    /** Determines how often it will search for new paths.
	 * If you have fast moving targets or AIs, you might want to set it to a lower value.
	 * The value is in seconds between path requests.
	 */
    public float repathRate = 0.5F;

    /** Target to move towards.
	 * The AI will try to follow/move towards this target.
	 * It can be a point on the ground where the player has clicked in an RTS for example, or it can be the player object in a zombie game.
	 */
    public Transform target;

    /** Enables or disables searching for paths.
	 * Setting this to false does not stop any active path requests from being calculated or stop it from continuing to follow the current path.
	 * \see #canMove
	 */
    public bool canSearch = true;

    /** Enables or disables movement.
	  * \see #canSearch */
    public bool canMove = true;

    /** Distance from the target point where the AI will start to slow down.
	 * Note that this doesn't only affect the end point of the path
 	 * but also any intermediate points, so be sure to set #forwardLook and #pickNextWaypointDist to a higher value than this
 	 */
    public float slowdownDistance = 0.6F;

    /** Determines within what range it will switch to target the next waypoint in the path */
    public float pickNextWaypointDist = 2;

    /** Target point is Interpolated on the current segment in the path so that it has a distance of #forwardLook from the AI.
	  * See the detailed description of AIPath for an illustrative image */
    public float forwardLook = 1;

    /** Distance to the end point to consider the end of path to be reached.
	 * When this has been reached, the AI will not move anymore until the target changes and OnTargetReached will be called.
	 */
    public float endReachedDistance = 0.2F;

    /** Do a closest point on path check when receiving path callback.
	 * Usually the AI has moved a bit between requesting the path, and getting it back, and there is usually a small gap between the AI
	 * and the closest node.
	 * If this option is enabled, it will simulate, when the path callback is received, movement between the closest node and the current
	 * AI position. This helps to reduce the moments when the AI just get a new path back, and thinks it ought to move backwards to the start of the new path
	 * even though it really should just proceed forward.
	 */
    public bool closestOnPathCheck = true;

    protected float minMoveScale = 0.05F;

    /** Cached Seeker component */
    protected Seeker seeker;

    /** Cached Transform component */
    protected Transform tr;

    /** Time when the last path request was sent */
    private float lastRepath = -9999;

    /** Current path which is followed */
    protected Path path;

    /** Cached Rigidbody component */
    protected Rigidbody rigid;

    /** Current index in the path which is current target */
    protected int currentWaypointIndex = 0;

    /** Holds if the end-of-path is reached
	 * \see TargetReached */
    protected bool targetReached = false;

    /** Only when the previous path has been returned should be search for a new path */
    protected bool canSearchAgain = true;

    protected Vector3 lastFoundWaypointPosition;
    protected float lastFoundWaypointTime = -9999;

    /** Holds if the Start function has been run.
     * Used to test if coroutines should be started in OnEnable to prevent calculating paths
     * in the awake stage (or rather before start on frame 0).
     */
    private bool startHasRun = false;

    /** Point to where the AI is heading.
      * Filled in by #CalculateVelocity */
    public Vector2 targetPoint;
    /** Relative direction to where the AI is heading.
     * Filled in by #CalculateVelocity */
    protected Vector2 targetDirection;

    public float speed = 2;

    /** Returns if the end-of-path has been reached
	 * \see targetReached */
    public bool TargetReached {
        get {
            return targetReached;
        }
    }

    /** Initializes reference variables.
	 * If you override this function you should in most cases call base.Awake () at the start of it.
	  * */
    protected virtual void Awake ()
    {
        seeker = GetComponent<Seeker> ();

        //This is a simple optimization, cache the transform component lookup
        tr = transform;

        rigid = rigidbody;
    }

    /** Starts searching for paths.
	 * If you override this function you should in most cases call base.Start () at the start of it.
	 * \see OnEnable
	 * \see RepeatTrySearchPath
	 */
    protected virtual void Start ()
    {
        startHasRun = true;
        OnEnable ();
    }

    /** Run at start and when reenabled.
	 * Starts RepeatTrySearchPath.
	 *
	 * \see Start
	 */
    protected virtual void OnEnable ()
    {

        lastRepath = -9999;
        canSearchAgain = true;

        lastFoundWaypointPosition = transform.position;

        if (startHasRun) {
            //Make sure we receive callbacks when paths complete
            seeker.pathCallback += OnPathComplete;

            StartCoroutine (RepeatTrySearchPath ());
        }
    }

    public void OnDisable ()
    {
        // Abort calculation of path
        if (seeker != null && !seeker.IsDone ())
            seeker.GetCurrentPath ().Error ();

        // Release current path
        if (path != null)
            path.Release (this);
        path = null;

        //Make sure we receive callbacks when paths complete
        seeker.pathCallback -= OnPathComplete;
    }

    /** Tries to search for a path every #repathRate seconds.
	  * \see TrySearchPath
	  */
    protected IEnumerator RepeatTrySearchPath ()
    {
        while (true) {
            float v = TrySearchPath ();
            yield return new WaitForSeconds (v);
        }
    }

    /** Tries to search for a path.
	 * Will search for a new path if there was a sufficient time since the last repath and both
	 * #canSearchAgain and #canSearch are true and there is a target.
	 *
	 * \returns The time to wait until calling this function again (based on #repathRate)
	 */
    public float TrySearchPath ()
    {
        if (Time.time - lastRepath >= repathRate && canSearchAgain && canSearch && target != null) {
            SearchPath ();
            return repathRate;
        } else {
            //StartCoroutine (WaitForRepath ());
            float v = repathRate - (Time.time - lastRepath);
            return v < 0 ? 0 : v;
        }
    }

    /** Requests a path to the target */
    public virtual void SearchPath ()
    {

        if (target == null)
            throw new System.InvalidOperationException ("Target is null");

        lastRepath = Time.time;
        //This is where we should search to
        Vector3 targetPosition = target.position;

        canSearchAgain = false;

        //Alternative way of requesting the path
        //ABPath p = ABPath.Construct (GetFeetPosition(),targetPoint,null);
        //seeker.StartPath (p);

        //We should search from the current position
        seeker.StartPath (transform.position, targetPosition);
    }

    public virtual void OnTargetReached ()
    {
        //End of path has been reached
        //If you want custom logic for when the AI has reached it's destination
        //add it here
        //You can also create a new script which inherits from this one
        //and override the function in that script
    }

    /** Called when a requested path has finished calculation.
	  * A path is first requested by #SearchPath, it is then calculated, probably in the same or the next frame.
	  * Finally it is returned to the seeker which forwards it to this function.\n
	  */
    public virtual void OnPathComplete (Path _p)
    {
        ABPath p = _p as ABPath;
        if (p == null)
            throw new System.Exception ("This function only handles ABPaths, do not use special path types");

        canSearchAgain = true;

        //Claim the new path
        p.Claim (this);

        // Path couldn't be calculated of some reason.
        // More info in p.errorLog (debug string)
        if (p.error) {
            p.Release (this);
            return;
        }

        //Release the previous path
        if (path != null)
            path.Release (this);

        //Replace the old path
        path = p;

        //Reset some variables
        currentWaypointIndex = 0;
        targetReached = false;

        //The next row can be used to find out if the path could be found or not
        //If it couldn't (error == true), then a message has probably been logged to the console
        //however it can also be got using p.errorLog
        //if (p.error)

        if (closestOnPathCheck) {
            Vector3 p1 = Time.time - lastFoundWaypointTime < 0.3f ? lastFoundWaypointPosition : p.originalStartPoint;
            Vector3 p2 = transform.position;
            Vector3 dir = p2 - p1;
            float magn = dir.magnitude;
            dir /= magn;
            int steps = (int)(magn / pickNextWaypointDist);


            for (int i=0; i<=steps; i++) {
                CalculateVelocity (p1);
                p1 += dir;
            }

        }
    }

    public virtual void Update ()
    {
        if (!canMove) {
            return;
        }

//        if (!targetReached) {
//            if (velocity < speed) {
//                velocity += acceleration * Time.smoothDeltaTime;
//            }
//        }

        float velocity = CalculateVelocity (tr.position);

        transform.position = Vector3.MoveTowards (transform.position, new Vector3 (targetPoint.x, targetPoint.y, transform.position.z), velocity * Time.smoothDeltaTime);
    }

    protected float XYSqrMagnitude (Vector2 a, Vector2 b)
    {
        float dx = b.x - a.x;
        float dy = b.y - a.y;
        return dx * dx + dy * dy;
    }

    /** Calculates desired velocity.
	 * Finds the target path segment and returns the forward direction, scaled with speed.
	 * A whole bunch of restrictions on the velocity is applied to make sure it doesn't overshoot, does not look too far ahead,
	 * and slows down when close to the target.
	 * /see speed
	 * /see endReachedDistance
	 * /see slowdownDistance
	 * /see CalculateTargetPoint
	 * /see targetPoint
	 * /see targetDirection
	 * /see currentWaypointIndex
	 */
    protected float CalculateVelocity (Vector2 currentPosition)
    {
        if (path == null || path.vectorPath == null || path.vectorPath.Count == 0)
            return 0f;

        List<Vector3> vPath = path.vectorPath;
        //Vector3 currentPosition = GetFeetPosition();

        if (vPath.Count == 1) {
            vPath.Insert (0, currentPosition);
        }

        if (currentWaypointIndex >= vPath.Count) {
            currentWaypointIndex = vPath.Count - 1;
        }

        if (currentWaypointIndex <= 1)
            currentWaypointIndex = 1;

        while (true) {
            if (currentWaypointIndex < vPath.Count - 1) {
                //There is a "next path segment"
                float dist = XYSqrMagnitude (vPath [currentWaypointIndex], currentPosition);
                //Mathfx.DistancePointSegmentStrict (vPath[currentWaypointIndex+1],vPath[currentWaypointIndex+2],currentPosition);
                if (dist < pickNextWaypointDist * pickNextWaypointDist) {
                    lastFoundWaypointPosition = currentPosition;
                    lastFoundWaypointTime = Time.time;
                    currentWaypointIndex++;
                } else {
                    break;
                }
            } else {
                break;
            }
        }

        Vector3 dir = vPath [currentWaypointIndex] - vPath [currentWaypointIndex - 1];
        Vector2 targetPosition = CalculateTargetPoint (currentPosition, vPath [currentWaypointIndex - 1], vPath [currentWaypointIndex]);
        //vPath[currentWaypointIndex] + Vector3.ClampMagnitude (dir,forwardLook);

        dir = targetPosition - currentPosition;
        float targetDist = dir.magnitude;

        float slowdown = Mathf.Clamp01 (targetDist / slowdownDistance);

        this.targetDirection = dir;
        this.targetPoint = targetPosition;

        if (currentWaypointIndex == vPath.Count - 1 && targetDist <= endReachedDistance) {
            if (!targetReached) {
                targetReached = true;
                OnTargetReached ();
            }

            //Send a move request, this ensures gravity is applied
            return 0f;
        }

        Vector3 forward = tr.forward;
        float dot = Vector3.Dot (dir.normalized, forward);
        float sp = speed * Mathf.Max (dot, minMoveScale) * slowdown;


        if (Time.deltaTime > 0) {
            sp = Mathf.Clamp (sp, 0, targetDist / (Time.deltaTime * 2));
        }
        return sp;
    }

    /** Calculates target point from the current line segment.
	 * \param p Current position
	 * \param a Line segment start
	 * \param b Line segment end
	 * The returned point will lie somewhere on the line segment.
	 * \see #forwardLook
	 * \todo This function uses .magnitude quite a lot, can it be optimized?
	 */
    protected Vector2 CalculateTargetPoint (Vector2 p, Vector2 a, Vector2 b)
    {
        float magn = (a - b).magnitude;
        if (magn == 0)
            return a;

		float closest = Mathf.Clamp01 (AstarMath.NearestPointFactor (a, b, p));
        Vector2 point = (b - a) * closest + a;
        float distance = (point - p).magnitude;

        float lookAhead = Mathf.Clamp (forwardLook - distance, 0.0F, forwardLook);

        float offset = lookAhead / magn;
        offset = Mathf.Clamp (offset + closest, 0.0F, 1.0F);
        return (b - a) * offset + a;
    }
}
