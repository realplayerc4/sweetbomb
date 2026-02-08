#!/usr/bin/env python3
"""
Simple HTTP server for OctoMap Web Viewer.
Serves static files from the web/ directory.
"""

import os
import http.server
import socketserver
from ament_index_python.packages import get_package_share_directory

PORT = 8888

def main():
    # Get web directory path
    pkg_dir = get_package_share_directory('octomap_web_viewer')
    web_dir = os.path.join(pkg_dir, 'web')
    
    os.chdir(web_dir)
    
    handler = http.server.SimpleHTTPRequestHandler
    
    with socketserver.TCPServer(("", PORT), handler) as httpd:
        print(f"\n🌐 OctoMap Web Viewer 已启动")
        print(f"   打开浏览器访问: http://localhost:{PORT}")
        print(f"   按 Ctrl+C 停止\n")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n服务器已停止")

if __name__ == "__main__":
    main()
