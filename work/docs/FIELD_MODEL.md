# Field model: `work/field/missions.json`

The map of the mat, in **inches**, on a 1 in grid. Origin `(0, 0)` = bottom-left corner as seen
from your launch side; +x right, +y up. Headings: 0 = far wall, clockwise positive.

> Today the hub code does **not** read this file (the hub can't parse JSON). The simulator reads
> `mat` and `robot`; the planned editor/code generator will use the rest (see the roadmap).
> Keep it filled in anyway: it's the team's source of truth for coordinates.

## Sections
| Key | Contents |
|---|---|
| `season` | season name |
| `mat` | `width_in` (long side, default 93), `height_in` (default 45), `grid_in` (1) |
| `robot` | `wheel_diameter_mm`, `axle_track_mm`, `footprint_in {width, length}`, `center_offset_in {forward}` (tracked point, in front of the axle) |
| `launch_areas[]` | `id`, `name`, `polygon` (≥ 3 `[x, y]` corners), `default_start_pose {x, y, heading}` |
| `alignment_features[]` | walls / black lines the robot can square against: `id`, `type`, `segment [[x,y],[x,y]]`, `note` |
| `missions[]` | `id` (e.g. `M01`), `name`, `points`, `polygon` (footprint of the model), `approach {x, y, heading}` (where to stop before doing it), `attachment`, `notes` |
| `obstacles[]` | non-mission things to drive around: `id`, `polygon`, `note` |
| `runs[]` | `id`, `start` (launch area id), `missions` (ids), `waypoints`, `notes` |

## Placeholder values to replace
`season`, `robot.footprint_in` (0), `center_offset_in`, all `launch_areas` polygons (zeros), the
`M00_example` mission (delete it), the example obstacle. Measure real values (see the Robolyst
task list) and edit the file.

## Example mission entry
```json
{
  "id": "M03",
  "name": "Swing the crane",
  "points": 20,
  "polygon": [[40, 20], [46, 20], [46, 26], [40, 26]],
  "approach": {"x": 43, "y": 16, "heading": 0},
  "attachment": "A",
  "notes": "push the lever with attachment A"
}
```
Then in a run: `drive_to(43, 16, 8)`, `turn_to(0)`, `drive(4, 4)`.
