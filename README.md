# 2026-BuildSeason-1498-B-Side

[![CI](https://github.com/frc1498/2026-BuildSeason-1498/actions/workflows/main.yml/badge.svg)](https://github.com/frc1498/2026-BuildSeason-1498/actions/workflows/main.yml)

Code for Team 1498's 2026 rebuilt robot, Aurora Zwei. <br>
[ADD ROBOT PICTURE HERE]

## General
I propose renaming this robot. <br>
I think it should still follow the team convention - Aurora Zwei - but it needs a better qualifier than '.II' or 'Mark 2'. <br>
The project is named Humpty-Dumpty because the rebuild is what the youths of today call a 'dumper bot', but I have another proposal. <br>
Enter 'Aurora Zwei: B-Side'. <br>
Granted, our team branding has no musical connection at all, but I think it's cleaner than Aurora Zwei: Electric Boogaloo.<br>
Think about it, won't you?

## Photonvision Class
I'm creating a reusable class for the photonvision cameras.
I want this to be a class that handles all of the common features for a photonvision camera.

### Current Photonvision Methods
setPhotonvisionPipeline <br>
isPhotonEstimateValid <br>
isResultAmbiguityBelowThreshold <br>
isResultAmbiguityInvalid <br>
arePhotonTagsSeen <br>
isPhotonDistanceClose <br>
takePhotonvisionSnapshot <br>
isPhotonvisionResultValid <br>
getCurrentLeftPhotonPose <br>
getCurrentRightPhotonPose <br>
processPhotonCameraResults <br>
updateEstimationStdDevs <br>
getEstimationStdDevs <br>

### The Process I Want To Follow
1. Read all unread results from the camera.
2. Limit the amount of results read per loop (there is a limit of 20 results per call, and that can be lowered for performance).
3. Preprocess the result - perform checks to determine if the result is worth turning into an estimate.  Skip the result if it fails the checks.
  - Check for tags.  Define a tag limit and discard any result under the limit.
  - Create an ambiguity limit.  Discard any result where the highest ambiguity is above a defined ambiguity threshold.
  - Check the distance to the tags.  The area of the tag in the result frame can be used to discard results from distant tags (these tags are small, but those tags are far away).  Distance to the tag requires a pose estimate, or at least I think it does.
  - Check the skew of the tags.  Optional, but a skewed tag is a tag being viewed at an extreme angle.
5. Create a pose estimate from the result.
  - Differentiate between multi-tag estimates and single tag estimates (if you allow results with a single tag).
6. Check the pose estimate for validity - perform checks to determine if the estimate is worth adding to the overall odometry.  Reject and throw away if it fails the checks.
  - Check for tags.  As a redundancy, discard any estimate with a number of tags under the limit.
  - Check the ambiguity.  As a redundancy, discard any estimate with an ambiguity above the ambiguity threshold.
  - Check the height of the estimate.  The robot should remain on the floor (outside of the ramp and climbing).  Reject any estimate where the robot may be floating.
  - Check for out of bound estimates.  The field has known boundaries, so reject any estimate that is outside of the field.  Could be expanded to reject estimates that are 'in' field elements.
7. Calculate the trust values for the estimate (the stddev value).  There's no 'set' formula, but consider:
  - A lower stddev value signifies a more trusted estimate.
  - Create a baseline stddev value, and scale the baseline by a calculated factor.
  - Trust estimates where there are multiple close tags.
  - Some cameras might be more trustworthy than others (depending on location, mounting, or camera type).
8. Add the resulting estimate to the overall odometry.

### Scope
Steps 1 - 5 should exist in the Photonvision class. <br>
Steps 6 - 8 should exist in the greater vision subsystem. <br>

### Notes
I think this process should work for the limelight as well.

