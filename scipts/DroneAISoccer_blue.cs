using System.Collections.Generic;
using UnityEngine;
using System;
using Panda;



[RequireComponent(typeof(DroneController))]
public class DroneAISoccer_blue : MonoBehaviour
{
    private DroneController m_Drone; // the drone controller we want to use

    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;

    public GameObject[] friends;
    public string friend_tag;
    public GameObject[] enemies;
    public string enemy_tag;

    public GameObject own_goal;
    public GameObject other_goal;
    public GameObject ball;
    private PandaBehaviour pandaBT; 
    private float maxSpeed = 15f;
    public float currentSpeed = 0f;
    private List<Vector3> ballCorners = new List<Vector3>();
    private GameObject closestEnemy;
    private Vector3 currentAccel;

    private float crashTimer = 0.4f;
    private float timer = -5f;
    private bool isBlockingEnemy = false;

    private float stadiumSize;
    
    bool isGoalie = false;
    private void Awake()
    {

    }

    private void Start()
    {
        // get the car controller

        pandaBT = GetComponent<PandaBehaviour>();
        m_Drone = GetComponent<DroneController>();
        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

        // Set the tags for friends and enemies
        friend_tag = gameObject.tag;
        if (friend_tag == "Blue")
        {
            enemy_tag = "Red";
        }
        else
        {
            enemy_tag = "Blue";
        }

        friends = GameObject.FindGameObjectsWithTag(friend_tag);
        enemies = GameObject.FindGameObjectsWithTag(enemy_tag);
        stadiumSize = Vector3.Distance(own_goal.transform.position, other_goal.transform.position);
        ball = GameObject.FindGameObjectWithTag("Ball");
    }

    private void OnCollisionEnter(Collision collision)
    {
        if(collision.gameObject.name=="Soccer Ball" && Vector3.Distance(ball.transform.position,other_goal.transform.position)<stadiumSize/2)
        {
            timer = Time.time;
        }
    }

    [Task]
    bool IsGoalieBlue()
    {
        float min = float.MaxValue;
        GameObject goalie = new GameObject();
        foreach(GameObject friend in friends)
        {
            float distanceToOwnGoal = Vector3.Distance(own_goal.transform.position, friend.transform.position);
            if(distanceToOwnGoal<min)
            {
                distanceToOwnGoal = min;
                goalie = friend; 
            }
        }
        if(gameObject==goalie)
        {
            isBlockingEnemy = false;
            maxSpeed = 100000000f;
            isGoalie = true;
        }
        return gameObject == goalie;
    }

    
   [Task]
   bool IsBlockingGoalieBlue()
   {
        return Vector3.Distance(ball.transform.position, other_goal.transform.position) < stadiumSize / 3;
   }

    [Task]
    bool IsShouldClearTheAreaBlue()
    {
        return false;
        Vector3 ballPosition = ball.transform.position;
        Vector3 myPosition = transform.position;
        if(Vector3.Distance(own_goal.transform.position,ballPosition)<stadiumSize/4) //if it is 'close' to the goal
        {
            float scale = own_goal.transform.localScale.z / 2;
            float offset = 10f;
            float highMargin = (own_goal.transform.position + scale * new Vector3(0f, 0f, 1f)).z + offset;
            float lowMargin = (own_goal.transform.position + scale * new Vector3(0f, 0f, -1f)).z - offset;
            
            if (ballPosition.z < highMargin && ballPosition.z > lowMargin)
            {
                Vector3 myGoalOrientation = (other_goal.transform.position - own_goal.transform.position).normalized;
                if (ball.GetComponent<Rigidbody>().velocity.x * myGoalOrientation.x < 0 || Vector3.Distance(own_goal.transform.position, ballPosition) < 25f) // if ball is coming towards me
                {
                    return true;
                }
            }
        }
        return false;
    }

    [Task]
    void ClearAreaBlue()
    {
        Vector3 ballPosition = ball.transform.position;
        Vector3 myPosition = transform.position;
        Vector3 ballVelocity = ball.GetComponent<Rigidbody>().velocity.normalized;
        ballVelocity.y = 0f;
        Vector3 myGoalOrientation = (other_goal.transform.position - own_goal.transform.position).normalized;
        Vector3 perpendicularClearVector = Vector3.Cross(Vector3.up, ballVelocity).normalized;
        if (Vector3.Dot(perpendicularClearVector, myGoalOrientation)<0)
        {
            perpendicularClearVector = -perpendicularClearVector;
        }

        //Vector3 intendedVelocity = ComputeCrashVelocity(20f);

        Vector3 anticipation = ballVelocity;
        Vector3 locationOnBall = ballPosition - 21 * ball.GetComponent<SphereCollider>().radius * perpendicularClearVector;
        Vector3 locationOnLine = ballPosition + 1.0f*ballVelocity * Vector3.Distance(myPosition, ballPosition); //intersectionOfTwoLines(ballVelocity.normalized);

        Debug.DrawLine(transform.position, locationOnBall, Color.yellow, 0.1f);
        Debug.DrawLine(transform.position, locationOnLine, Color.green, 0.1f);

        Vector3 location = (locationOnBall + locationOnLine) / 2;
        Debug.DrawLine(transform.position, location, Color.magenta, 0.1f);

        float k_v = 5f;
        //Vector3 acceleration = VelocityCorrection(intendedVelocity, k_v);

        //m_Drone.Move_vect(acceleration);
        MoveAgent(location, new Vector3(0f, 0f, 0f));
    }
    
    private Vector3 VelocityCorrection(Vector3 intendedVelocity, float k_v)
    {
        Vector3 acceleration = intendedVelocity;
        Vector3 currentVelocity = GetComponent<Rigidbody>().velocity;
        Vector3 speedDifference = intendedVelocity - currentVelocity;

        acceleration = acceleration + k_v * speedDifference;
        return acceleration;
    }
    
    private Vector3 ComputeCrashVelocity(float coefficient)
    {
        Vector3 ballVelocity = ball.GetComponent<Rigidbody>().velocity;
        Vector3 direction = ball.transform.position - transform.position;
        Vector3 redArrowVelocity = ballVelocity + coefficient * direction;
        Vector3 intendedVelocity = ballVelocity + redArrowVelocity;
        return intendedVelocity.normalized * maxSpeed ;
    }

    [Task]
    void DefendBlue(float coefficient)
    {
        float goalSize = own_goal.transform.localScale.z/2;
        Vector3 ballAndGoalCollisionPoint = own_goal.transform.position;       // We assume the ball is going to hit the centre of the goal
        Vector3 ballVelocity = ball.GetComponent<Rigidbody>().velocity;

        if (isGoalie)
        {
            ballAndGoalCollisionPoint = GetCollisionPointOfBallOnGoal(ballVelocity);
            Debug.DrawLine(ball.transform.position,ballAndGoalCollisionPoint, Color.red, 0.1f);    // Point of impact
        }
        
        Vector3 direction = ball.transform.position - ballAndGoalCollisionPoint;
        Vector3 offset = coefficient * direction;

        if(ballAndGoalCollisionPoint==own_goal.transform.position)
        {
            if (offset.magnitude < 1.2f * goalSize)
            {
                offset = offset.normalized * 1.2f * goalSize;
            }
        }
        else
        {
            ballVelocity.y = 0f;
            Vector3 ballAndGoalAreaIntersectPoint = new Vector3();
            RaycastHit[] hits = Physics.RaycastAll(ball.transform.position, ballVelocity, stadiumSize);
            foreach (RaycastHit hit in hits)
            {
                if (hit.collider.name.Equals(own_goal.name + "_Collider"))
                {
                    ballAndGoalAreaIntersectPoint = hit.point; // The ball is going to hit the goal keepers area in this point
                    break;
                }
            }

            Debug.DrawLine(ballAndGoalCollisionPoint, ballAndGoalAreaIntersectPoint, Color.cyan, 0.1f);
            
            float minimumOffset = Vector3.Distance(ballAndGoalAreaIntersectPoint, ballAndGoalCollisionPoint);
            if (offset.magnitude < minimumOffset)
            {
                offset = offset.normalized*minimumOffset;
            }
        }
        
        Vector3 location = ballAndGoalCollisionPoint + offset;
        Debug.DrawLine(transform.position, location, Color.white, 0.1f);
        MoveAgent(location, new Vector3(0f, 0f, 0f));
    }


    private Vector3 GetCollisionPointOfBallOnGoal( Vector3 ballVelocity)
    {
        ballVelocity.y = 0f;
        RaycastHit[] hits = Physics.RaycastAll(ball.transform.position, ballVelocity, stadiumSize);
        foreach(RaycastHit hit in hits)
        {
            if (hit.collider.name.Equals(own_goal.name))
            {
                return hit.point;         // The ball is going to hit the goal in this point
            }
        }
        // No collision on goal. We assume that the opponents will aim for the centre of the goal when they get the chance
        return own_goal.transform.position;  
    }

    [Task]
    bool IsChaserBlue()
    {
        return !IsGoalieBlue();
    }

    [Task]
    bool IsClosestToTheBallBlue()
    {
        GameObject closestFriend = new GameObject();
        float minDistance = float.MaxValue;
        foreach (GameObject friend in friends)
        {
            float distanceToBall = Vector3.Distance(friend.transform.position, ball.transform.position);
            if (distanceToBall < minDistance)
            {
                minDistance = distanceToBall;
                closestFriend = friend;
            }
        }
        return closestFriend == gameObject;
    }

    private Vector3 GetTargetLocation()
    {
        GameObject enemyGoalie = GetEnemyGoalie();
        Vector3 aimingTowards = other_goal.transform.position;
        Vector3 direction = new Vector3();
        if (ball.transform.position.x<enemyGoalie.transform.position.x)
        {
            float radius = 5f;
            Collider[] hitColliders = Physics.OverlapSphere(enemyGoalie.transform.position, radius);

            foreach (Collider collider in hitColliders)
            {
                if (collider.name.Equals("Sphere") && collider.transform.parent.tag.Equals(friend_tag) && !collider.transform.parent.Equals(gameObject))
                {
                    Debug.DrawLine(collider.transform.position, enemyGoalie.transform.position, Color.green, 0.1f);
                    Debug.Log("A friend is blocking the enemy goalie");
                    Vector3 point = collider.transform.position;
                    point.y = 0f;
                    direction = point - enemyGoalie.transform.position;
                    aimingTowards = enemyGoalie.transform.position + 5 * direction;
                    break;
                }
            }
        }
        Debug.DrawLine(enemyGoalie.transform.position, enemyGoalie.transform.position +5*direction, Color.black, 0.1f);
        return aimingTowards;
    }

    [Task]
    void DribbleBlue()
    {
        isBlockingEnemy = false;
        Vector3 ballVelocity = ball.GetComponent<Rigidbody>().velocity.normalized;
        ballVelocity.y = 0f;

        Vector3 aimingTowards = GetTargetLocation();
        Vector3 direction = aimingTowards- ball.transform.position;
        direction.y = 0f;
        Debug.DrawLine(ball.transform.position, ball.transform.position - 40 * ball.GetComponent<SphereCollider>().radius * direction.normalized, Color.magenta, 0.1f);

        Vector3 leftPost;
        Vector3 rightPost;
        (leftPost, rightPost) = GetOpponentsGoalPosts(aimingTowards);
        Debug.DrawLine(ball.transform.position, ball.transform.position + leftPost, Color.black, 0.1f) ;
        Debug.DrawLine(ball.transform.position, ball.transform.position + rightPost, Color.white, 0.1f);

        bool isCloserToLeftPost = Vector3.Angle(Vector3.Cross(Vector3.up, ballVelocity), direction) < 90f;

        if(Vector3.Distance(ball.transform.position,other_goal.transform.position)<stadiumSize/2)
        {
            if (isCloserToLeftPost)
            {
                if (Vector3.Angle(ballVelocity, direction) > Mathf.Max(Vector3.Angle(direction, leftPost), 30f))
                {
                    direction = direction.normalized - ballVelocity;
                }
            }
            else
            {
                if (Vector3.Angle(ballVelocity, direction) > Mathf.Max(Vector3.Angle(direction, rightPost), 30f))
                {
                    direction = direction.normalized - ballVelocity;
                }
            }
        }
       
        
        direction = -direction;         // Flip the direction towards ourselves
       
        // Apply force that enforce the drone to go to a better kick position
        Vector3 location = ball.transform.position + 21*ball.GetComponent<SphereCollider>().radius * direction.normalized;
        Debug.DrawLine(ball.transform.position, ball.transform.position + 10 * ball.GetComponent<Rigidbody>().velocity.normalized, Color.black, 0.1f);
        Debug.DrawLine(ball.transform.position, ball.transform.position + 40 * ball.GetComponent<SphereCollider>().radius * direction.normalized, Color.yellow, 0.1f);
        
        direction = location - transform.position;

        // We look if we can reach the point to hit the ball, if blocked we try to go around the ball
        List<String> obstacles = new List<string>(){"Soccer Ball", "Cube"};
        if (IsPathBlocked(location, direction, obstacles))
        {
            location = FindBestCorner(location);
        }
        
        Debug.DrawLine(transform.position, location, Color.cyan, 0.1f);
        MoveAgent(location, new Vector3(0, 0, 0));
    }

    private Tuple<Vector3, Vector3> GetOpponentsGoalPosts(Vector3 aimingTowards)
    {
        // Swaps the coefficient depending on which team is running the code. The opposite teams have mirrored world view
        float coef = 1f;
        if (enemy_tag == "Blue")
        {
            coef = -1f;
        }

        Vector3 leftPost;
        Vector3 rightPost;
        if(aimingTowards.Equals(other_goal.transform.position))
        {
            leftPost = (aimingTowards - coef*other_goal.transform.localScale.z / 2 * new Vector3(0, 0, 1)) - ball.transform.position;
            rightPost = (aimingTowards - coef*other_goal.transform.localScale.z / 2 * new Vector3(0, 0, -1)) - ball.transform.position;
        }
        else
        {
            leftPost = (aimingTowards - coef*other_goal.transform.localScale.z / 5 * new Vector3(0, 0, 1)) - ball.transform.position;
            rightPost = (aimingTowards - coef*other_goal.transform.localScale.z / 5 * new Vector3(0, 0, -1)) - ball.transform.position;
            
        }
        // We do not care about the y axis so we set them to 0
        leftPost.y = 0f;
        rightPost.y = 0f;
        return new Tuple<Vector3, Vector3>(leftPost, rightPost);
    }

    private bool IsPathBlocked(Vector3 location, Vector3 direction, List<String> obstacles)
    {
        List<Vector3> offsets = new List<Vector3>() { new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0) };
        foreach (Vector3 point in offsets)
        {
            Debug.DrawLine(transform.position + point, location + point, Color.red, 0.1f);
            RaycastHit[] hits = Physics.RaycastAll(transform.position + point, direction, direction.magnitude);
            foreach (RaycastHit hit in hits)
            {
                foreach (String obstacle in obstacles)
                {
                    if (hit.collider.name == obstacle)
                    {
                        return true;
                    }
                }
                return false;
            }
        }
        return false;
    }
    
    [Task]
    void DisturbEnemyBlue()
    {
        Vector3 ballPosition = ball.transform.position;
        isBlockingEnemy = true;

        DisturbGoalie();
        /*
        float fieldLength = (own_goal.transform.position - other_goal.transform.position).magnitude;
        if (Vector3.Distance(ballPosition, other_goal.transform.position) < fieldLength / 3)
        {
            DisturbGoalie();
        }
        else
        {
            DistrubNearestEnemy();
        }
        */
    }
   
    private void DisturbGoalie()
    {
        GameObject enemyGoalie = GetEnemyGoalie();
        Debug.DrawLine(transform.position, enemyGoalie.transform.position, Color.blue, 0.1f);

        Vector3 ballPosition = ball.transform.position;
        Vector3 targetLocation;
        
        Vector3 offset = new Vector3();
        float radius = 1.1f * 0.5f;
        if (ballPosition.z < other_goal.transform.position.z - other_goal.transform.localScale.z)
        {
            targetLocation = enemyGoalie.transform.position + radius*new Vector3(0f,0f,-1f);
            offset = radius * new Vector3(0f, 0f, -1f);
        }
        else if(ballPosition.z > other_goal.transform.position.z + other_goal.transform.localScale.z)
        {
            targetLocation = enemyGoalie.transform.position + radius * new Vector3(0f, 0f, 1f);
            offset = radius * new Vector3(0f, 0f, 1f);
        }
        else
        {
            Vector3 ballVelocity = ball.GetComponent<Rigidbody>().velocity;

            Vector3 center = other_goal.transform.position - ball.transform.position;

            if(Vector3.Angle(Vector3.Cross(Vector3.up,ballVelocity), center)>90f) // ball velocity is above center
            {
                targetLocation = enemyGoalie.transform.position + radius * new Vector3(0f, 0f, -1f);
                offset = radius * new Vector3(0f, 0f, -1f);
            }
            else
            {
                targetLocation = enemyGoalie.transform.position + radius * new Vector3(0f, 0f, 1f);
                offset = radius * new Vector3(0f, 0f, 1f);
            }
        }

        Debug.DrawLine(enemyGoalie.transform.position, enemyGoalie.transform.position + 10*offset, Color.magenta, 0.1f);
        MoveAgent(targetLocation, new Vector3(0, 0, 0));
    }

    private GameObject GetEnemyGoalie()
    {
        GameObject enemyGoalie = new GameObject();
        float minDistance = float.MaxValue;
        foreach (GameObject enemy in enemies)
        {
            float distanceToEnemyGoal = Vector3.Distance(enemy.transform.position, other_goal.transform.position);
            if (distanceToEnemyGoal < minDistance)
            {
                minDistance = distanceToEnemyGoal;
                enemyGoalie = enemy;
            }
        }
        return enemyGoalie;
    }

    private void DisruptNearestEnemy()
    {
        GameObject closestEnemy = new GameObject();
        float minDistance = float.MaxValue;
        foreach (GameObject enemy in enemies)
        {
            float distanceToEnemyGoal = Vector3.Distance(enemy.transform.position, ball.transform.position);
            if (distanceToEnemyGoal < minDistance)
            {
                minDistance = distanceToEnemyGoal;
                closestEnemy = enemy;
            }
        }

        Debug.DrawLine(transform.position, closestEnemy.transform.position, Color.yellow, 0.1f);
        MoveAgent(closestEnemy.transform.position, new Vector3(0, 0, 0));
    }

    private Vector3 FindBestCorner(Vector3 location)
    {
        float minDistance = float.MaxValue;
        Vector3 bestCorner = location;
        foreach(Vector3 corner in ballCorners)
        {
            float distance = Vector3.Distance(location, corner);
            Vector3 direction = corner - transform.position;
            
            List<String> obstacles = new List<string>(){"Soccer Ball", "Cube"};
             if (distance < minDistance && ! IsPathBlocked(location, direction, obstacles))
            {
                bestCorner = corner;
                minDistance = distance;
            }
        }
        return bestCorner;
    }

    private bool isNearBall(Vector3 location)
    {
        Vector3 normal = (location - ball.transform.position).normalized;
        Vector3 relativePosition = (transform.position - ball.transform.position).normalized;
        return (Vector3.Distance(transform.position, ball.transform.position) < 10f && Vector3.Dot(normal, relativePosition)>0f
            && Vector3.Distance(location,ball.transform.position)< 23 * ball.GetComponent<SphereCollider>().radius);

    }

    private bool IsNearWall()
    {
        float radius = 10f;
        Collider[] hitColliders = Physics.OverlapSphere(transform.position, radius);

        foreach(Collider collider in hitColliders)
        {
            if(collider.name.Equals("Cube"))
            { return true; }
            
        }
        return false;
    }

    private Vector3 WallCorrection()
    {
        float radius = 10f;
        Vector3 accelerationCorection = new Vector3();
        Collider[] hitColliders = Physics.OverlapSphere(transform.position, radius);
        foreach (Collider collider in hitColliders)
        {
            if (collider.name.Equals("Cube"))
            {
                Vector3 positionOfWall = collider.transform.position;
                int i = terrain_manager.myInfo.get_i_index(positionOfWall.x);
                int j = terrain_manager.myInfo.get_j_index(positionOfWall.z);
                Vector3 point = collider.ClosestPointOnBounds(transform.position);
                Debug.DrawLine(transform.position, point, Color.red, 0.1f);
                int zDim = terrain_manager.myInfo.z_N;
                if(j==zDim-1)
                {
                    accelerationCorection = new Vector3(0f, 0f, -1f);
                }
                else if (j == 0)
                {
                    accelerationCorection = new Vector3(0f, 0f, 1f);
                }
                else if (i == 0)
                {
                    accelerationCorection = new Vector3(1f, 0f, 0f);
                }
                else 
                {
                    accelerationCorection = new Vector3(-1f, 0f, 0f);
                }

                float magnitude = Mathf.Atan(5 / (point - transform.position).magnitude);
                if (isGoalie)
                {
                    magnitude = Mathf.Atan(1 / (point - transform.position).magnitude);
                }
                accelerationCorection = magnitude * accelerationCorection.normalized;
            }
        }
        return accelerationCorection;
    }

    private bool isNearBallOnWrongSide(Vector3 location)
    {
        Vector3 normal = (location - ball.transform.position).normalized;
        Vector3 relativePosition = (transform.position - ball.transform.position).normalized;
        normal.y = 0f;
        relativePosition.y = 0f;
        return (Vector3.Distance(transform.position, ball.transform.position) < 10f && Vector3.Angle(normal,relativePosition)>45f);
    }

    private Vector3 TryToCorrectSpeed(Vector3 location)
    {
        Vector3 normal = (location - ball.transform.position).normalized;
        normal.y = 0f;
        Vector3 relativePosition = (transform.position - ball.transform.position);
        relativePosition.y = 0f;
        Vector3 projection = Vector3.Dot(relativePosition, normal) * normal;
        Vector3 perpendicular = -(relativePosition - projection);

        float magnitude = Mathf.Atan(perpendicular.magnitude);
        return magnitude * perpendicular.normalized;

    }
    private Vector3 AvoidBall()
    {
        Vector3 relativePosition = transform.position - ball.transform.position;
        relativePosition.y = 0f;
        float magnitude = Mathf.Atan(3/relativePosition.magnitude);

        return magnitude * relativePosition.normalized;
    }
    private void MoveAgent(Vector3 targetLocation, Vector3 targetVelocity)
    {
        Vector3 currentVelocity = GetComponent<Rigidbody>().velocity;
        Vector3 currentLocation = transform.position;
        Tuple<Vector3, Vector3> targetState = new Tuple<Vector3, Vector3>(targetLocation, targetVelocity);
        float min = float.MaxValue;
        float best_u_x = 0f;
        float best_u_z = 0f;
        for (float u_x = -15; u_x<15; u_x+=1)
        {
            for (float u_z = -15; u_z < 15; u_z+=1)
            {
                Tuple<Vector3, Vector3> state = f(currentLocation, currentVelocity, u_x, u_z); 
                float distance = Compare(state, targetState);
                if(state.Item2.magnitude>maxSpeed)
                {
                    continue;
                }
                if (distance < min)
                {
                    min = distance;
                    best_u_x = u_x;
                    best_u_z = u_z;
                }
            }
        }
        currentSpeed = currentVelocity.magnitude;
        Vector3 acceleration = new Vector3(best_u_x, 0f, best_u_z).normalized;
        currentAccel = acceleration;
        
        if (IsBeingBlockedBlue() && !isBlockingEnemy)
        {
            // The agent is being blocked, try to unblock itself
            Debug.Log("I am being blocked! I am goalie: " + isGoalie); 
            Vector3 direction = closestEnemy.transform.position - transform.position;
            Vector3 avoidAccel = 3 * UnblockBlue();
            if(Vector3.Angle(avoidAccel,targetLocation-currentLocation)>90f)
            {
                avoidAccel = -avoidAccel;
            }
            Debug.DrawLine(transform.position, transform.position + 10 * avoidAccel, Color.magenta, 0.1f);

            bool enemyIsOnOurRight = Vector3.Angle(Vector3.Cross(Vector3.up, targetLocation - currentLocation), direction) < 90f;

            if(enemyIsOnOurRight)
            {
                avoidAccel = Quaternion.Euler(0f, -20f, 0f) * avoidAccel;
            }
            else
            {
                avoidAccel = Quaternion.Euler(0f, 20f, 0f) * avoidAccel;
            }

            Debug.DrawLine(transform.position, transform.position + 10 * avoidAccel, Color.blue, 0.1f);
            acceleration = (acceleration +avoidAccel).normalized;
        }

        
        if(Vector3.Distance(ball.transform.position,other_goal.transform.position) < stadiumSize/2)
        {
            if (isNearBall(targetLocation))
            {
                Vector3 velocityCorretion = TryToCorrectSpeed(targetLocation);
                acceleration = (acceleration + velocityCorretion).normalized;
            }
        }

        if (isNearBallOnWrongSide(targetLocation))
        {
            Vector3 avoidBallAccel = AvoidBall();
            acceleration = (acceleration + avoidBallAccel).normalized;
        }

        if (IsNearWall())
        {
            Vector3 avoidWallAccel = WallCorrection();
            Debug.DrawLine(transform.position, transform.position + 5 * avoidWallAccel, Color.white, 0.1f);
            acceleration = (acceleration + avoidWallAccel).normalized;
        }
        
        if (Time.time>timer+crashTimer)
        {
            m_Drone.Move_vect(acceleration);
        }
        else
        {
            m_Drone.Move_vect(new Vector3(0f,0f,0f));
        }
        
    }
    bool IsBeingBlockedBlue()
    {
        float min = float.MaxValue;
        closestEnemy = new GameObject();
       
        List<GameObject> allAgents = new List<GameObject>();
        allAgents.AddRange(enemies);
        allAgents.AddRange(friends);
        //if (isGoalie)
       // {
       //     allAgents.AddRange(friends);
       // }
        foreach (GameObject agent in allAgents)
        {
            float distance = Vector3.Distance(transform.position, agent.transform.position);
            if (distance < min && !agent.Equals(gameObject))
            {
                min = distance;
                closestEnemy = agent;
            }
        }
        return min < 5f;
    }
    
    Vector3 UnblockBlue()
    {
        IsBeingBlockedBlue();
        float distance = Vector3.Distance(closestEnemy.transform.position, transform.position);
        float magnitude = Mathf.Atan(3 / distance);
        Vector3 direction = closestEnemy.transform.position - transform.position;
        Vector3 escapeVelocity = Vector3.Cross(Vector3.up, direction).normalized;
        escapeVelocity = magnitude * escapeVelocity;
        return escapeVelocity;
    }

    private float Compare(Tuple<Vector3,Vector3> state, Tuple<Vector3, Vector3> targetState)
    {
        return Vector3.Distance(state.Item1, targetState.Item1);
    }

    private Tuple<Vector3,Vector3> f (Vector3 currentLocation, Vector3 currentVelocity, float u_x, float u_z)
    {
        // x(t+1) = x(t) + v(t) * Time.deltaTime;
        // v(t+1) = v(t) + a(t) * Time.deltaTime;
        Vector3 acceleration = new Vector3(u_x, 0f, u_z);
        Vector3 nextVelocity = currentVelocity + acceleration * Time.deltaTime;
        Vector3 nextPosition = currentLocation + nextVelocity * Time.deltaTime;
        Tuple<Vector3, Vector3> nextState = new Tuple<Vector3, Vector3>(nextPosition, nextVelocity);  
        return nextState;
    }
    
     private Tuple<bool, RaycastHit> KeepingSafeDistance()
     {
        float scale = 4f;
        var forward = transform.forward; // or currentVelocity
        Vector3 rayRight = Quaternion.Euler(0f, 20f, 0f)*forward;
        Vector3 rayLeft = Quaternion.Euler(0f, -20f, 0f) * forward;
        int mask = ~(1 << LayerMask.NameToLayer("Ignore Raycast"));

        Vector3 front = transform.position +2* forward + new Vector3(0f,0.5f,0f);
        bool ray = Physics.Linecast(front, front + scale*rayRight, out RaycastHit ray_hit,mask);

        if (ray)
        {
            Debug.DrawLine(front, front + scale * rayRight, Color.magenta, 0.1f);
            return new Tuple<bool, RaycastHit>(false,ray_hit);
        }
        else
        {
            Debug.DrawLine(front, front + scale * rayRight, Color.green, 0.1f);
        }
        ray = Physics.Linecast(front, front + scale* rayLeft,out ray_hit,mask);
        if (ray)
        {
            Debug.DrawLine(front, front + scale * rayLeft, Color.magenta, 0.1f);
            return new Tuple<bool, RaycastHit>(false, ray_hit);
        }
        else
        {
            Debug.DrawLine(front, front + scale * rayLeft, Color.green, 0.1f);
        }
        ray = Physics.Linecast(front, front + scale*transform.forward,out ray_hit,mask);
        if (ray)
        {
            Debug.DrawLine(front, front + scale * transform.forward, Color.magenta, 0.1f);
            return new Tuple<bool, RaycastHit>(false, ray_hit);
        }
        else
        {
            Debug.DrawLine(front, front + scale * transform.forward, Color.green, 0.1f);
        }
        return new Tuple<bool, RaycastHit>(true, ray_hit);

     }

    private void GenerateSquareCorners()
    {
        Vector3 ballCenter = ball.transform.position;
        float radius = 20 * ball.GetComponent<SphereCollider>().radius;
        float offset = 3;
        float boxSizeLength = 2 * radius + offset;
        // clockwise
        Vector3 northEast = ballCenter + new Vector3(boxSizeLength / 2, 0f, boxSizeLength / 2);
        Vector3 southEast = ballCenter + new Vector3(boxSizeLength / 2, 0f, -boxSizeLength / 2);
        Vector3 southWest = ballCenter + new Vector3(-boxSizeLength / 2, 0f, -boxSizeLength / 2);
        Vector3 northWest = ballCenter + new Vector3(-boxSizeLength / 2, 0f, boxSizeLength / 2);

        ballCorners.Clear();
        ballCorners.Add(northEast);
        ballCorners.Add(southEast);
        ballCorners.Add(southWest);
        ballCorners.Add(northWest);

        DrawBoxCorners();
    }

    private void DrawBoxCorners()
    {
        Debug.DrawLine(ballCorners[0], ballCorners[3], Color.red, 0.1f);
        for (int i = 1; i < ballCorners.Count; i++)
        {
            Debug.DrawLine(ballCorners[i-1], ballCorners[i], Color.red, 0.1f);
        }
    }

    private void Update()
    {
        GenerateSquareCorners();        // Create and visualize the box around the ball
        pandaBT.Reset();
        pandaBT.Tick();
    }
}