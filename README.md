# 2026-BuildSeason-1498-B-Side

[![CI](https://github.com/frc1498/2026-BuildSeason-1498/actions/workflows/main.yml/badge.svg)](https://github.com/frc1498/2026-BuildSeason-1498/actions/workflows/main.yml)

Code for Team 1498's 2026 rebuilt robot, Aurora Zwei. <br>
[ADD ROBOT PICTURE HERE]

## Battery Metadata Logging
In the never-ending quest for more data, we today turn our gaze to batteries.
The performance of a battery during competitions is extremely important.
As such, our batteries are inspected and profiled yearly.
This data allows us to rank our batteries so they can be prioritized during competitions.
This approach falls short during root cause investigations after matches.
Simply put, we don't remember which battery was used during a match.
The pit crew chooses a suitable battery, using a Battery Beak as a last minute sanity check for charge and internal resistance.
But after the match, it isn't a high priority to record which battery was used.
Even if it was, a scrap of paper or word document becomes an easy to lose data point.
A more reliable solution would be to record that information in the robot code, making that data available in the same data log as every other robot parameter.

### Battery Metadata
ID
Purchase Date
Manufacture Date
Preference Ranking
Rated Voltage
Rated Capacity (Wh)
Internal Resistance
Use Case (Competition, Practice, Development)

### Logging
The biggest problem with logging the battery metadata is determining which battery is connected to the robot.

### The Process
- Create a 'Battery' class of constants representing battery metadata.
- In the robot project, create a constants file with a battery object for each team battery.
- Create a sendable chooser that allows the driver to select the battery from a list.
- Once autonomous starts, log the metadata of the selected battery.
