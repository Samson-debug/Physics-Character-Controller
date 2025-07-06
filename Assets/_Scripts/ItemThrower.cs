using UnityEditor.UIElements;
using UnityEngine;
using UnityEngine.Serialization;

public class ItemThrower : MonoBehaviour
{
    public GameObject itemPrefab = null;
    public float size = .4f;
    public float throwForce = 100f;

    private Camera cam;

    private void Awake()
    {
        cam = Camera.main;
    }

    private void Update()
    {
        if (Input.GetMouseButtonDown(0)){
            ThrowItem();
        }
    }

    private void ThrowItem()
    {
        GameObject item = Instantiate(itemPrefab, transform.position, Quaternion.identity);
        item.transform.localScale = new Vector3(size, size, size);
        item.transform.position = cam.transform.position;
        Rigidbody itemRB = item.GetComponent<Rigidbody>();

        if (itemRB == null){
            item.AddComponent(typeof(Rigidbody));
            itemRB = item.AddComponent<Rigidbody>();
        }
        
        Ray ray = cam.ScreenPointToRay(Input.mousePosition);

        bool itemThrown = false;
        
        if (Physics.Raycast(ray, out RaycastHit hit)){
            Vector3 torqueAxis = Random.insideUnitSphere;
                
            itemRB.AddTorque(torqueAxis * 10f);
            itemRB.AddForce((hit.point - cam.transform.position).normalized * throwForce, ForceMode.Impulse);
        }
        else{
            Vector3 dir = cam.ScreenToWorldPoint(Input.mousePosition) - cam.transform.position;
            dir.Normalize();
            
            itemRB.AddForce(dir * throwForce, ForceMode.Impulse);
        }
    }
}