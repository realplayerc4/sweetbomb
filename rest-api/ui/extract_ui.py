import json
import os

json_path = '/home/yq/ros2realsense/rest-api/ui/robot_ui_extracted/ai_chat.json'
target_base = '/home/yq/ros2realsense/rest-api/ui/frontend'

with open(json_path, 'r') as f:
    data = json.load(f)

threads = data.get('threads', [])
if not threads:
    print("No threads found")
    exit(1)

# Iterate through messages in the thread to find tool calls writing files
for msg in threads[0].get('messages', []):
    parts = msg.get('parts', [])
    for part in parts:
        # Looking for tool calls that presumably write files or tool usage
        # The key seems to be 'tool-call-json-DO-NOT-USE-IN-PROD' or similar structure
        # In the provided JSON view, we see 'toolName': 'write_tool' and 'argsJson' with 'path' and 'file_text'
        
        content_json_str = part.get('contentJson')
        if content_json_str:
            try:
                content = json.loads(content_json_str)
                if content.get('toolName') == 'write_tool':
                    args_json = content.get('argsJson')
                    if args_json:
                        args = json.loads(args_json)
                        path = args.get('path')
                        file_text = args.get('file_text')
                        
                        if path and file_text:
                             # Ensure path is relative and clean
                            if path.startswith('/src'):
                                full_path = os.path.join(target_base, path.lstrip('/'))
                                os.makedirs(os.path.dirname(full_path), exist_ok=True)
                                with open(full_path, 'w') as out_f:
                                    out_f.write(file_text)
                                print(f"Extracted: {full_path}")
            except json.JSONDecodeError:
                pass
            except Exception as e:
                print(f"Error extracting part: {e}")

print("Extraction complete.")
