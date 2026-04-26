# Navigate to directory
# Create a limelightLog folder in the directory above this folder
# Get JSON of limelight photos from limelight
# For each JSON entry, download the photo from the web address
# After, delete all pictures
# Do same for Rewind

import os
import json
import requests

# Create a folder for the limelight pictures.
# This is the same directory as the code project.
logFolderDirectory = str(os.path.join(os.path.pardir, "limelightData"))

print(f"Creating folder - {logFolderDirectory}")
if not os.path.exists(logFolderDirectory):
    os.makedirs(logFolderDirectory)

# Use 'limelight.local', because that will work over ethernet and USB
# Otherwise, use 10.14.98.11
limelightAddress = "limelight.local"
restPort = "5807"
generalPort = "5800"
#url = f"http://{address}:{port}/{route}"

# Build the REST request for the current list of snapshots on the limelight.
snapshotManifest = f"http://{limelightAddress}:{restPort}/snapshotmanifest"

# DEBUG #
#snapshotFile = str(os.path.join(logFolderDirectory, "555.png"))
#print(f"Saving file - {snapshotFile}")
#open(snapshotFile, 'wb').write(snapshot)

# Send the REST request and retrieve the JSON result.
print(f"Sending GET Request - {snapshotManifest}")
imageList = requests.get(snapshotManifest)

# For each filename in the result, navigate to the URL of the snapshot and download.
for filename in imageList.json:
    print(f"Downloading file - {filename}")
    snapshotURL = f"http://{limelightAddress}:{generalPort}/snapshots/{filename}"
    snapshot = requests.get(snapshotURL).content
    snapshotFile = str(os.path.join(logFolderDirectory, filename))

    print(f"Saving file - {snapshotFile}")
    open(snapshotFile, 'wb').write(snapshot)

# Build the REST request to delete all of the snapshots.
deleteSnapshots = f"http://{limelightAddress}:{restPort}/delete-snapshots"
print(f"Sending DELETE Request - {deleteSnapshots}")
requests.delete(deleteSnapshots)
