
![OctNav Window](https://cdn.discordapp.com/attachments/734864355992010792/1385949768320290906/image.png?ex=6857edce&is=68569c4e&hm=020b77609d2296e206c8c24911a2a2813c6d44884f33ab8ed3d5a5065c55a43e&)





# Octree Navigation

Install from package manager in Unity with `https://github.com/TheosCreation/OctNav.git`

## OctAgent Class Example

Here is an example of how to move the agent to a target transform:

```csharp
public void Update()
{
    // Sets the agent's target to a Unity Transform (e.g., a player object).
    // The agent will continuously try to pathfind to this transform's position.
    agent.SetTarget(player);
}

```

Here is an example of how to move the agent to a specific `Vector3` position:

```csharp
public void Start()
{
    // Sets a fixed destination point for the agent.
    // The agent will navigate to this exact Vector3 coordinate.
    agent.SetDestination(new Vector3(10f, 0f, 5f));
}

```

## Public Functions Reference

This section documents all public functions available in the `OctreeNavigationAgent` class.

### `SetDestination(Vector3 point)`

Sets a manual destination point for movement, resetting any target and forcing a repath.

-   **Parameters:**

    -   `point`: The target destination as a `Vector3`.

-   **Example Usage:**

    ```csharp
    // Move the agent to the coordinates (10, 0, 5)
    agent.SetDestination(new Vector3(10f, 0f, 5f));
    
    ```


### `SetTarget(Transform newTarget = null)`

Sets a new movement target using a `Transform`. Resets any previously set destination and forces a new path if the navigation graph is valid. If `newTarget` is null, the current target is cleared, and the agent will stop.

-   **Parameters:**

    -   `newTarget`: The target `Transform` to follow. Can be `null` to clear the target.

-   **Example Usage:**

    ```csharp
    // Set the player Transform as the agent's target
    public Transform playerTransform;
    void Start()
    {
        agent.SetTarget(playerTransform);
    }
    
    // To stop following the target:
    void StopFollowing()
    {
        agent.SetTarget(null);
    }
    
    ```


### `SetDestinationStraight(Vector3? point = null)`

Sets a straight-line movement destination. If `point` is not provided, it defaults to the agent's current `currentGoal` (if set). The agent will attempt to move directly to this point, stopping if a collision is detected.

-   **Parameters:**

    -   `point`: Optional destination point as a `Vector3?`. If `null`, uses the current goal if available.

-   **Example Usage:**

    ```csharp
    // Move the agent directly to (20, 0, 10) in a straight line
    agent.SetDestinationStraight(new Vector3(20f, 0f, 10f));
    
    // Or, if a target/destination is already set, move straight to it
    // (e.g., if you previously called SetTarget or SetDestination)
    agent.SetDestinationStraight();
    
    ```


### `PathToPoint(Vector3 goalPosition)`

Computes a path to the given goal position using A* pathfinding or straight path logic, depending on the `straightPath` setting. This function is typically called internally by `SetDestination` and `SetTarget`, but can be used directly for specific path recalculation needs.

-   **Parameters:**

    -   `goalPosition`: The `Vector3` position to navigate toward.

-   **Example Usage (less common for direct use, usually handled by SetDestination/SetTarget):**

    ```csharp
    // Manually calculate a path to a point (e.g., after an environmental change)
    // Note: You would usually want to call SetDestination or SetTarget to initiate movement
    agent.PathToPoint(new Vector3(5f, 0f, 5f));
    
    ```


### `Stop(bool fullStop = false)`

Stops the agent's movement and optionally resets its velocity.

-   **Parameters:**

    -   `fullStop`: If `true`, zeroes the velocity completely (including rigidbody velocity). If `false`, the agent will decelerate naturally.

-   **Example Usage:**

    ```csharp
    // Stop the agent gradually
    agent.Stop();
    
    // Immediately stop the agent and zero its velocity
    agent.Stop(true);
    
    ```


### `Pause()`

Pauses movement and stores the current velocity and angular velocity. The agent will freeze in place.

-   **Example Usage:**

    ```csharp
    // Pause the agent's movement
    agent.Pause();
    
    ```


### `Resume()`

Resumes movement using previously cached velocity and angular velocity. The agent will continue its path from where it left off.

-   **Example Usage:**

    ```csharp
    // Resume the agent's movement
    agent.Resume();
    
    ```


### `BeginMovement()`

Begins movement toward the current goal (either a set destination or target `Transform`). Returns a `PathSubscription` object that can be used to subscribe to the `OnDestinationReached` event.

-   **Returns:** A `PathSubscription` object if a goal is valid; otherwise, `null`.

-   **Example Usage:**

    ```csharp
    void Start()
    {
        // Set a destination
        agent.SetDestination(new Vector3(100f, 0f, 100f));
    
        // Start movement and subscribe to the destination reached event
        using (OctreeNavigationAgent.PathSubscription subscription = agent.BeginMovement())
        {
            if (subscription != null)
            {
                subscription.OnReached += HandleDestinationReached;
            }
        }
    }
    
    private void HandleDestinationReached()
    {
        Debug.Log("Agent has reached its destination!");
    }
    
    ```


### `ResetPath()`

Resets the internal path objects without triggering a full re-path or recalculation. This clears the current path data but does not stop or start movement.

-   **Example Usage:**

    ```csharp
    // Clear the agent's current calculated path
    agent.ResetPath();
    
    ```


### `GetAStarPathLength()`

Gets the number of nodes in the current A* path.

-   **Returns:** The length of the current A* path. Returns `0` if the path is `null`.

-   **Example Usage:**

    ```csharp
    int pathNodeCount = agent.GetAStarPathLength();
    Debug.Log($"Current A* path has {pathNodeCount} nodes.");
    
    ```


### `GetPathNode(int index)`

Retrieves the `OctNode` at the specified index in the current A* path.

-   **Parameters:**

    -   `index`: The index of the node to retrieve.

-   **Returns:** The `OctNode` at the given index, or `null` if the index is out of bounds or the path is invalid.

-   **Example Usage:**

    ```csharp
    if (agent.GetAStarPathLength() > 0)
    {
        OctNode firstNode = agent.GetPathNode(0);
        if (firstNode != null)
        {
            Debug.Log($"First node center: {firstNode.bounds.center}");
        }
    }
    
    ```

## OctVolume Class Example

Here is an example of how to build the octree volume dynamically:

```csharp
public OctVolume volume;

public void UpdateVolume()
{
    // Build the volume at runtime if major scene changes occur
    volume.Build();
}

```






# Octree Navigation

Install from package manager in Unity with [https://github.com/TheosCreation/OctNav.git](https://github.com/TheosCreation/OctNav.git)

## OctBuilder Class Example

Here is an example of how to use the `OctBuilder` component to build a navigation graph:

```csharp
using UnityEngine;
using OctNav;

public class MyBuilderScript : MonoBehaviour
{
    public OctBuilder octBuilder;

    void Start()
    {
        // Ensure the OctBuilder component is assigned in the inspector.
        // Or get it from the GameObject if it's on the same one.
        octBuilder = GetComponent<OctBuilder>();

        // Before calling Build(), you must ensure the BoundsHandles
        // component (which is required by OctBuilder) has its bounds
        // properly defined in the editor.
        // Once the bounds are set, you can trigger the build process:
        octBuilder.Build();
        Debug.Log("Octree Navigation Graph Built!");
    }
}

```


## Public Functions Reference

This section documents all public functions available in the `OctBuilder` class.

### `Build()`

Triggers the navigation graph build process using the bounds defined by the attached `BoundsHandles` component. This function initiates the creation of the octree-based navigation graph within the specified area.

-   **Parameters:** None.

-   **Example Usage:**

    ```csharp
    // Call this function to build the navigation graph once your bounds are set.
    public OctBuilder builder; // Assign this in the Unity Editor
    void Start()
    {
        builder.Build();
        Debug.Log("Navigation graph build initiated.");
    }
    
    ```



# Octree Navigation

Install from package manager in Unity with [https://github.com/TheosCreation/OctNav.git](https://github.com/TheosCreation/OctNav.git)

## OctVolume Class Example

The `OctVolume` component is responsible for generating and managing the octree navigation data for a specific area in your scene. It is often used in conjunction with `OctBuilder` (which has a `Build()` method to trigger this volume's build process) to define the bounds and build the navigation graph.

Typically, you would attach `OctVolume` to an empty GameObject and use the `BoundsHandles` component (which is automatically added as a requirement) to visually define the area for the octree.

Once the bounds are set, the octree can be built, loaded, or sections can be rebuilt.

## Public Functions Reference

This section documents all public functions available in the `OctVolume` class.

### `SaveOctree()`

Saves the current octree structure to disk. The octree data is stored in a JSON format within a specific directory relative to your Unity project (`SceneData/OctreeData/`).

-   **Parameters:** None.

-   **Example Usage:**

    ```csharp
    // Assuming 'octVolume' is a reference to your OctVolume component
    public OctVolume octVolume;
    
    void SaveCurrentOctree()
    {
        octVolume.SaveOctree();
        Debug.Log("Octree saved successfully.");
    }
    
    ```


### `LoadOctree()`

Attempts to load a previously saved octree from disk. If a saved octree is found and successfully loaded, it will rebuild the navigation graphs (both the general graph and the walkable grounded graph) based on the loaded data.

-   **Parameters:** None.

-   **Returns:** `bool` - `true` if the octree was successfully loaded; `false` otherwise.

-   **Example Usage:**

    ```csharp
    // Assuming 'octVolume' is a reference to your OctVolume component
    public OctVolume octVolume;
    
    void Start()
    {
        if (octVolume.LoadOctree())
        {
            Debug.Log("Octree loaded from file.");
        }
        else
        {
            Debug.LogWarning("No existing octree found or failed to load. Building a new one.");
            octVolume.BuildChildren(); // Fallback to building if load fails
        }
    }
    
    ```


### `BuildChildren()`

Builds a new octree from the geometry (defined by `geometryMask`) located inside the current bounds of the `OctVolume`. This is the primary function to generate the octree structure and its associated navigation graphs from scratch. It also handles setting colliders to convex temporarily during the build process if `setCollidersToConex` is enabled.

-   **Parameters:** None.

-   **Example Usage:**

    ```csharp
    // Assuming 'octVolume' is a reference to your OctVolume component
    public OctVolume octVolume;
    
    void RebuildNavigation()
    {
        Debug.Log("Starting octree rebuild...");
        octVolume.BuildChildren();
        Debug.Log("Octree rebuild complete.");
    }
    
    ```


### `GetEmptyLeaves(OctNode node)`

Recursively finds all empty (non-colliding) leaf nodes within the octree starting from a given `OctNode` and adds them to the `emptyLeaves` list. These empty leaves represent navigable space.

-   **Parameters:**

    -   `node`: The starting `OctNode` for the recursive search.

-   Example Usage (Internal/Advanced):

    This function is primarily called internally by OctVolume during the build process. You typically wouldn't need to call it directly.


### `BuildGraph()`

Adds empty leaf nodes and their connections (face links) to the main `OctNavigation.graph`. This function effectively constructs the primary navigation graph that agents can use for pathfinding.

-   **Parameters:** None.

-   Example Usage (Internal/Advanced):

    This function is primarily called internally by OctVolume during the build and load processes. You typically wouldn't need to call it directly.


### `BuildWalkableGraph()`

Builds the grounded navigation graph (`OctNavigation.groundGraph`) specifically for surface walking. It identifies "walkable" nodes (nodes with collision that have an empty node above them) and connects them, including handling steps and edges.

-   **Parameters:** None.

-   Example Usage (Internal/Advanced):

    This function is primarily called internally by OctVolume during the build and load processes. You typically wouldn't need to call it directly.


### `ResetOctree()`

Resets all internal octree data and navigation graphs associated with this volume. This clears all calculated nodes and connections, effectively preparing the volume for a new build or load operation. It also repaints the Scene View in the editor.

-   **Parameters:** None.

-   **Example Usage:**

    ```csharp
    // Useful for clearing data before a new build or for cleanup.
    public OctVolume octVolume;
    void ResetData()
    {
        octVolume.ResetOctree();
        Debug.Log("Octree data reset.");
    }
    
    ```


### `BuildSection(Bounds sectionBounds)`

Allows for rebuilding a specific section of the octree. This is useful for dynamic environments where only parts of the navigation mesh might change. It subdivides nodes within the `sectionBounds`, then rebuilds the affected graph parts.

-   **Parameters:**

    -   `sectionBounds`: The `Bounds` defining the area to rebuild within the octree.

-   **Example Usage:**

    ```csharp
    // Rebuild a specific 10x10x10 meter area around a point
    public OctVolume octVolume;
    public Vector3 centerOfChange;
    
    void UpdateDynamicArea()
    {
        Bounds changedArea = new Bounds(centerOfChange, new Vector3(10, 10, 10));
        octVolume.BuildSection(changedArea);
        Debug.Log($"Section around {centerOfChange} rebuilt.");
    }
    
    ```


### `FindNodeAtPoint(Vector3 point)`

Finds and returns the specific `OctNode` that contains the given `Vector3` point within this `OctVolume`'s octree.

-   **Parameters:**

    -   `point`: The `Vector3` position to search for within the octree.

-   **Returns:** `OctNode` - The `OctNode` that contains the point, or `null` if no node at that point is found or the root is null.

-   **Example Usage:**

    ```csharp
    // Find the octree node at the player's position
    public OctVolume octVolume;
    public Transform playerTransform;
    
    void Update()
    {
        OctNode nodeUnderPlayer = octVolume.FindNodeAtPoint(playerTransform.position);
        if (nodeUnderPlayer != null)
        {
            Debug.Log($"Player is in octree node at: {nodeUnderPlayer.bounds.center}");
        }
    }
    
    ```


### `DrawNodeGizmo(OctNode node, bool hitPass = false)`

This function is responsible for drawing gizmos for individual octree nodes in the Unity editor. It applies distance fading based on the camera's position and can draw either the overall octree structure or highlight collision nodes. This is an internal helper for the `OnDrawGizmos` and `OnDrawGizmosSelected` methods.

-   **Parameters:**

    -   `node`: The `OctNode` to draw gizmos for.

    -   `hitPass`: A `bool` indicating if this pass is for drawing "hit" (colliding) gizmos.

-   Example Usage (Internal/Advanced):

    This function is used internally by the OctVolume to visualize the octree in the editor. You typically wouldn't call this directly from your scripts.




