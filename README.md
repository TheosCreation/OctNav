# TMUtils

Install from package manager in unity with https://github.com/TheosCreation/TMUtils.git

## TMDebug class example
Here is a example of a melee attack that uses the Draw Debug Box function to display the damage hit box

```csharp

public void DamageTrigger()
{
    // Define the fire position
    Vector3 firePosition = transform.position;

    // Determine the forward direction
    Vector3 direction = transform.forward;
    
    // Define the box parameters
    Vector3 boxSize = new Vector3(1.5f, 1.0f, 1.5f);
    Vector3 boxCenter = firePosition + direction * 1.0f; // Offset to move forward slightly
    Quaternion boxOrientation = Quaternion.identity;

    // Perform a box cast or overlap box
    Collider[] hitColliders = Physics.OverlapBox(boxCenter, boxSize / 2, boxOrientation, damageMask);

    // Process the results
    foreach (var collider in hitColliders)
    {
        //IDamageable is a interface class used to damage a entity with health
        if (collider.TryGetComponent(out IDamageable target))
        {
            target.Damage(damage);
        }
        else if (collider.transform.root.TryGetComponent(out IDamageable target2))
        {
            target2.Damage(damage);
        }
    }

    TMDebug.DrawDebugBox(boxCenter, boxSize, boxOrientation, Color.blue, 0.5f);
}
```
