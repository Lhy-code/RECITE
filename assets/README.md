# README media

These assets illustrate the RECITE compiler and its Scene–Route Intermediate Representation (SR–IR).

| Asset | Source and role |
| --- | --- |
| `overview.png` | Raster export of the RECITE manuscript's Figure 1: the conceptual overview of structured scene families, SR–IR fields, three structural tiers, and matched reactive-controller evaluation. |
| `srir_fields.png` | Figure 3 with a six-item legend: route, occupied tube, interaction-candidate interval (ICI), portal, rejoin anchor, and goal cone. The lower panels show the accompanying route-progress profiles. |
| `srir_fields.gif` | Cropped field-by-field sequence from the RECITE supplementary demonstration. It retains the original scene and secondary view, removes narration subtitles and surrounding margins, and plays at 2.4× speed for a compact README preview. |
| `avoidance.gif` | Recorded CBF-QP + SR–IR rollout in a cabinet scene: lateral avoidance, recovery, and goal arrival. The original motion plays at 1× speed, with a short introductory sweep and terminal hold. Local nominal tube slices appear and fade over time rather than leaving the entire swept volume opaque. |

The images and animation are qualitative illustrations of the representation. Quantitative stage results are provided separately in [`../results/`](../results/).

The GIF uses **interaction-candidate interval (ICI)** for the route segment that guides dynamic-event placement during compilation. Rejoin anchors describe candidate recovery states, while the goal cone represents the final approach region.

This release provides the figures and README-ready animations listed above; their source videos and production scripts remain in the media-production workspace.

The avoidance preview preserves the saved robot and obstacle trajectories and their source clock. The tube shows reference-motion occupancy, and the recovery annotations follow the selected controller trace.
