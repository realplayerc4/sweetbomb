import requests
import json
import time

BASE_URL = "http://localhost:8000/api"

def test_waypoints():
    print("--- Testing Waypoints ---")
    # List waypoints
    r = requests.get(f"{BASE_URL}/waypoints/")
    print(f"List waypoints: {r.status_code}")
    print(json.dumps(r.json(), indent=2))
    
    # Create waypoint
    payload = {
        "name": "TestPoint",
        "pos": [1.0, 2.0, 0.5]
    }
    r = requests.post(f"{BASE_URL}/waypoints/", json=payload)
    print(f"Create waypoint: {r.status_code}")
    
    # Verify
    r = requests.get(f"{BASE_URL}/waypoints/")
    wps = r.json()
    if any(wp['name'] == 'TestPoint' for wp in wps):
        print("✅ Waypoint created successfully")
    else:
        print("❌ Waypoint creation failed")

def test_navigation_task():
    print("\n--- Testing Navigation Task ---")
    payload = {
        "task_type": "navigation",
        "params": {
            "target_name": "TestPoint",
            "speed": 1.0
        }
    }
    r = requests.post(f"{BASE_URL}/tasks/", json=payload)
    task_id = r.json()['task_id']
    print(f"Created Navigation Task: {task_id}")
    
    # Start task
    requests.post(f"{BASE_URL}/tasks/{task_id}/start")
    print("Task started")
    
    # Monitor progress
    for _ in range(5):
        time.sleep(1)
        r = requests.get(f"{BASE_URL}/tasks/{task_id}")
        info = r.json()
        print(f"Status: {info['status']}, Progress: {info['progress']['percentage']:.1f}%, Message: {info['progress']['message']}")
        if info['status'] == 'completed':
            break
    
    print("✅ Navigation task test finished")

def test_sequential_queue():
    print("\n--- Testing Sequential Task Queue ---")
    payload = {
        "task_type": "sequential_queue",
        "params": {
            "sub_tasks": [
                {"task_type": "return_to_origin", "params": {}},
                {"task_type": "object_detection", "params": {"max_frames": 10}}
            ]
        }
    }
    r = requests.post(f"{BASE_URL}/tasks/", json=payload)
    task_id = r.json()['task_id']
    print(f"Created Sequential Queue Task: {task_id}")
    
    # Start
    requests.post(f"{BASE_URL}/tasks/{task_id}/start")
    
    # Monitor briefly
    for _ in range(5):
        time.sleep(1)
        r = requests.get(f"{BASE_URL}/tasks/{task_id}")
        info = r.json()
        print(f"Queue Status: {info['status']}, Progress: {info['progress']['percentage']:.1f}%, Msg: {info['progress']['message']}")
    
    print("✅ Sequential queue test finished")

if __name__ == "__main__":
    try:
        test_waypoints()
        test_navigation_task()
        test_sequential_queue()
    except Exception as e:
        print(f"Test failed: {e}")
