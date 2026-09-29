"""Zip the ROS 2 package for uploading to The Construct (or any ROS machine).

    python ros2/make_upload_zip.py

Creates ros2/wafer_transfer_robot.zip containing the wafer_transfer_robot/
package folder, without Python caches.
"""

import os
import zipfile


here = os.path.dirname(os.path.abspath(__file__))
package = os.path.join(here, "src", "wafer_transfer_robot")
archive = os.path.join(here, "wafer_transfer_robot.zip")

count = 0

with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as zip_file:

    for folder, subfolders, files in os.walk(package):

        subfolders[:] = [name for name in subfolders if name != "__pycache__"]

        for name in files:
            path = os.path.join(folder, name)
            relative = os.path.relpath(path, os.path.dirname(package))
            zip_file.write(path, relative.replace(os.sep, "/"))
            count += 1

print(f"Saved {archive} ({count} files)")
